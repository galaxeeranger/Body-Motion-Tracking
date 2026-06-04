
import numpy as np
import cv2


class Triangulator:
    """
    Multi-view triangulation for N cameras WITH lens distortion handling.

    Convention
    ----------
    R_list[i], T_list[i] are camera-from-world extrinsics, i.e.:
        X_cam_i = R_list[i] @ X_world + T_list[i]
    so the projection matrix is  P_i = K_i @ [R_i | T_i].

    Distortion
    ----------
    If `dist_list` is supplied, every detected 2D point is undistorted with
    cv2.undistortPoints(..., P=K) before being fed into the linear DLT.
    This is REQUIRED for any real lens — without it, scale and reprojection
    error are heavily biased.
    """

    def __init__(self, K_list, R_world, T_world,
                 dist_list=None, min_views=2, conf_thresh=0.3,
                 use_ransac=True, ransac_reproj_px=12.0, ransac_min_inliers=2):
        self.K_list = [np.asarray(K, dtype=np.float64) for K in K_list]
        self.R_list = [np.asarray(R, dtype=np.float64) for R in R_world]
        self.T_list = [
            np.asarray(T, dtype=np.float64).reshape(3, 1) for T in T_world
        ]

        if dist_list is None:
            print("[Triangulator] WARNING: no distortion coefficients given. "
                  "Reconstruction will be biased for any real lens.")
            self.dist_list = [np.zeros(5) for _ in K_list]
        else:
            self.dist_list = [
                np.asarray(d, dtype=np.float64).flatten() for d in dist_list
            ]

        self.min_views = min_views
        self.conf_thresh = conf_thresh

        # robust triangulation settings (SCALING_8_CAMERAS.md §2.3)
        self.use_ransac         = use_ransac
        self.ransac_reproj_px   = float(ransac_reproj_px)
        self.ransac_min_inliers = int(ransac_min_inliers)

        # P_i = K_i @ [R_i | T_i]
        self.P_list = self._build_projection_matrices()

    # ─────────────────────────────────────────────
    def _build_projection_matrices(self):
        P_list = []
        for K, R, T in zip(self.K_list, self.R_list, self.T_list):
            RT = np.hstack((R, T))   # (3,4)
            P_list.append(K @ RT)    # (3,4)
        return P_list

    # ─────────────────────────────────────────────
    def _undistort_all(self, keypoints_2d_all):
        """
        Vectorised undistortion per camera.
        Returns a list of (J, 3) arrays — same shape as input, but xy
        replaced with undistorted pixel coordinates.
        """
        out = []
        for cam_idx, kpts in enumerate(keypoints_2d_all):
            pts_xy = kpts[:, :2].astype(np.float64)

            # Undistort -> back to pixel space using the same K
            # (so we can keep using P = K [R|T])
            pts_un = cv2.undistortPoints(
                pts_xy.reshape(-1, 1, 2),
                self.K_list[cam_idx],
                self.dist_list[cam_idx],
                P=self.K_list[cam_idx],
            ).reshape(-1, 2)

            stacked = np.column_stack([pts_un, kpts[:, 2]])
            out.append(stacked)
        return out

    # ─────────────────────────────────────────────
    def triangulate_frame(self, keypoints_2d_all):
        """
        keypoints_2d_all : list of (num_joints, 3) arrays per camera
                           each row = (x_pixel, y_pixel, confidence)
        Returns          : (num_joints, 3) array, NaN where < min_views
        """
        num_cams = len(keypoints_2d_all)
        num_joints = keypoints_2d_all[0].shape[0]

        # 1. Undistort everything once per frame
        undist = self._undistort_all(keypoints_2d_all)

        points_3d = np.zeros((num_joints, 3), dtype=np.float32)

        for j in range(num_joints):
            obs = []
            for cam_idx in range(num_cams):
                x, y, conf = undist[cam_idx][j]
                if conf < self.conf_thresh:
                    continue
                obs.append((cam_idx, x, y, conf))

            if len(obs) < self.min_views:
                points_3d[j] = np.array([np.nan, np.nan, np.nan])
                continue

            points_3d[j] = self._triangulate_point(obs)

        return points_3d

    # ─────────────────────────────────────────────
    def _triangulate_point(self, observations):
        """
        Robust multi-view triangulation for a single joint.

        With 2 views there is nothing to vote on — fall back to a direct DLT.
        With 3+ views, run RANSAC: every camera pair proposes a 3D hypothesis,
        we keep the hypothesis with the most cameras agreeing (low reprojection
        error), then re-solve a weighted DLT using only those inliers. This
        rejects bad views — left/right swaps and occlusion guesses — that would
        otherwise corrupt the plain least-squares solution.
        """
        if not self.use_ransac or len(observations) <= 2:
            return self._dlt(observations)

        best_inliers = []
        n = len(observations)
        for a in range(n):
            for b in range(a + 1, n):
                cand = self._dlt([observations[a], observations[b]])
                if cand is None or not np.all(np.isfinite(cand)):
                    continue
                inliers = [
                    o for o in observations
                    if self._reproj_err(cand, o) < self.ransac_reproj_px
                ]
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers

        need = max(self.ransac_min_inliers, self.min_views)
        if len(best_inliers) >= need:
            return self._dlt(best_inliers)

        # no consensus — fall back to all observations rather than dropping
        return self._dlt(observations)

    # ─────────────────────────────────────────────
    def _dlt(self, observations):
        """Linear DLT — solves A X = 0 for homogeneous X (confidence-weighted)."""
        A = []
        for cam_idx, x, y, conf in observations:
            P = self.P_list[cam_idx]
            w = conf
            A.append(w * (x * P[2] - P[0]))
            A.append(w * (y * P[2] - P[1]))

        A = np.asarray(A)
        _, _, Vt = np.linalg.svd(A)
        X = Vt[-1]
        if abs(X[3]) < 1e-12:
            return np.array([np.nan, np.nan, np.nan])
        return X[:3] / X[3]

    # ─────────────────────────────────────────────
    def _reproj_err(self, X_world, observation):
        """Pixel reprojection error of a 3D point in one camera's view."""
        cam_idx, x_obs, y_obs, _ = observation
        R = self.R_list[cam_idx]
        T = self.T_list[cam_idx]
        K = self.K_list[cam_idx]

        Xc = R @ X_world.reshape(3, 1) + T
        if Xc[2, 0] <= 1e-9:                 # behind the camera
            return np.inf
        u = K[0, 0] * Xc[0, 0] / Xc[2, 0] + K[0, 2]
        v = K[1, 1] * Xc[1, 0] / Xc[2, 0] + K[1, 2]
        return float(np.hypot(u - x_obs, v - y_obs))