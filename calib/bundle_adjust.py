
import numpy as np
import cv2
from scipy.optimize import least_squares
from config import BA_MAX_ITER, BA_FTOL


class BundleAdjuster:
    """
    Fixed Bundle Adjustment:
    - Optimizes ONLY camera poses
    - Uses known ChArUco board geometry (metric scale)
    - Prevents scale drift / explosion
    """

    def __init__(self, board_config, state):
        self.cfg   = board_config
        self.state = state

    # ──────────────────────────────────────────────────────────────
    def optimise(self, session, K_list, dist_list, R_world_init, T_world_init):

        num_cams = len(K_list)
        self.state.log("\nBundle adjustment starting...")

        observations = self._build_observations(session, K_list, dist_list)

        if len(observations) < 10:
            self.state.log("WARNING: too few observations — skipping BA")
            return R_world_init, T_world_init

        self.state.log(f"  {len(observations)} observations, {num_cams} cameras")

        # ── pack ONLY camera params (cam0 fixed)
        x0 = self._pack_camera_params(R_world_init[1:], T_world_init[1:])

        result = least_squares(
            self._residuals,
            x0,
            args=(
                num_cams,
                observations,
                K_list,
                dist_list,
                R_world_init[0],
                T_world_init[0]
            ),
            method="trf",   # more stable than LM
            max_nfev=BA_MAX_ITER * len(x0),
            ftol=BA_FTOL,
            xtol=1e-6,
            gtol=1e-6,
        )

        self.state.log(
            f"  BA converged: {result.success}  "
            f"cost {result.cost:.4f}  "
            f"msg: {result.message}"
        )

        R_opt, T_opt = self._unpack_camera_params(
            result.x, num_cams, R_world_init[0], T_world_init[0]
        )

        self._report(R_world_init, T_world_init, R_opt, T_opt,
                     observations, K_list)

        return R_opt, T_opt

    # ──────────────────────────────────────────────────────────────
    def _build_observations(self, session, K_list, dist_list):

        observations = []

        for frame_idx in range(len(session.all_charuco_corners[0])):
            for cam_idx in range(session.num_cameras):

                ids     = session.all_charuco_ids[cam_idx][frame_idx]
                corners = session.all_charuco_corners[cam_idx][frame_idx]

                if ids is None or corners is None:
                    continue

                for k, cid in enumerate(ids.flatten()):
                    pt_2d = corners[k][0]

                    pt_undist = cv2.undistortPoints(
                        pt_2d.reshape(1, 1, 2),
                        K_list[cam_idx],
                        dist_list[cam_idx],
                        P=K_list[cam_idx]
                    ).reshape(2)

                    observations.append(
                        (cam_idx, int(cid), pt_undist[0], pt_undist[1])
                    )

        return observations

    # ──────────────────────────────────────────────────────────────
    def _pack_camera_params(self, R_list, T_list):
        parts = []
        for R, T in zip(R_list, T_list):
            rvec, _ = cv2.Rodrigues(R)
            parts.append(rvec.flatten())
            parts.append(T.flatten())
        return np.concatenate(parts)

    def _unpack_camera_params(self, x, num_cams, R0, T0):

        R_list = [R0]
        T_list = [T0]

        offset = 0
        for _ in range(num_cams - 1):
            rvec = x[offset:offset+3]; offset += 3
            tvec = x[offset:offset+3]; offset += 3

            R, _ = cv2.Rodrigues(rvec)
            T    = tvec.reshape(3, 1)

            R_list.append(R)
            T_list.append(T)

        return R_list, T_list

    # ──────────────────────────────────────────────────────────────
    def _residuals(self, x, num_cams,
                   observations, K_list, dist_list, R0, T0):

        R_list, T_list = self._unpack_camera_params(x, num_cams, R0, T0)

        board_pts = self.cfg.board.getChessboardCorners()

        residuals = np.empty(len(observations) * 2)

        for i, (cam_idx, cid, u_obs, v_obs) in enumerate(observations):

            X_world = board_pts[cid].reshape(3, 1)

            R = R_list[cam_idx]
            T = T_list[cam_idx]
            K = K_list[cam_idx]

            X_cam = R @ X_world + T

            if X_cam[2, 0] <= 0:
                residuals[2*i]   = 1000
                residuals[2*i+1] = 1000
                continue

            u = K[0, 0] * X_cam[0, 0] / X_cam[2, 0] + K[0, 2]
            v = K[1, 1] * X_cam[1, 0] / X_cam[2, 0] + K[1, 2]

            residuals[2*i]   = u - u_obs
            residuals[2*i+1] = v - v_obs

        return residuals

    # ──────────────────────────────────────────────────────────────
    def _report(self, R_init, T_init, R_opt, T_opt,
                observations, K_list):

        board_pts = self.cfg.board.getChessboardCorners()

        self.state.log("\n── Bundle adjustment results ──")

        for cam_idx in range(len(R_opt)):

            errs_before = []
            errs_after  = []

            for (ci, cid, u_obs, v_obs) in observations:
                if ci != cam_idx:
                    continue

                X = board_pts[cid].reshape(3, 1)

                def proj(R, T, K):
                    Xc = R @ X + T
                    if Xc[2, 0] <= 0:
                        return None
                    u = K[0,0]*Xc[0,0]/Xc[2,0] + K[0,2]
                    v = K[1,1]*Xc[1,0]/Xc[2,0] + K[1,2]
                    return u, v

                p0 = proj(R_init[cam_idx], T_init[cam_idx], K_list[cam_idx])
                p1 = proj(R_opt[cam_idx],  T_opt[cam_idx],  K_list[cam_idx])

                if p0:
                    errs_before.append((p0[0]-u_obs)**2 + (p0[1]-v_obs)**2)
                if p1:
                    errs_after.append((p1[0]-u_obs)**2 + (p1[1]-v_obs)**2)

            before = np.sqrt(np.mean(errs_before)) if errs_before else 0
            after  = np.sqrt(np.mean(errs_after))  if errs_after else 0

            T_mm = T_opt[cam_idx].flatten() * 1000

            self.state.log(
                f"  Cam{cam_idx}  RMS {before:.3f} -> {after:.3f} px  "
                f"T=[{T_mm[0]:.1f},{T_mm[1]:.1f},{T_mm[2]:.1f}] mm"
            )