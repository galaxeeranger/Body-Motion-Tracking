import numpy as np
from collections import deque


class PoseGraph:
    """
    Builds a connected pose graph from pairwise stereo transforms and
    resolves world-frame poses for every camera via BFS from camera 0.

    Camera 0 is the world origin:  R_world_0 = I,  T_world_0 = 0

    For every other camera k, the world pose (R_world_k, T_world_k) means:
        X_world  =  R_world_k @ X_cam_k  +  T_world_k

    The BFS result is the INITIAL estimate — it will be refined by
    BundleAdjuster afterwards to remove accumulated chain error.
    """

    def __init__(self, num_cameras, state):
        self.num_cameras = num_cameras
        self.state       = state

    # ──────────────────────────────────────────────────────────────
    def build_initial_poses(self, pair_transforms):
        """
        Parameters
        ----------
        pair_transforms : dict  (i, j) -> {"R": R_ij, "T": T_ij}
            R_ij, T_ij = transform FROM camera i TO camera j.
            i < j always (as produced by StereoCalibrator).

        Returns
        -------
        R_world : list of (3,3) arrays — world rotation per camera
        T_world : list of (3,1) arrays — world translation per camera
        """
        # report graph health (loops + weak links) before chaining
        self._report_graph_health(pair_transforms)

        # build adjacency: store both directions
        adj = {i: {} for i in range(self.num_cameras)}
        for (i, j), tr in pair_transforms.items():
            R_ij = tr["R"]
            T_ij = tr["T"]
            # i -> j  (direct)
            adj[i][j] = (R_ij, T_ij)
            # j -> i  (inverted)
            R_ji, T_ji = self._invert_transform(R_ij, T_ij)
            adj[j][i] = (R_ji, T_ji)

        # BFS from camera 0
        R_world = [None] * self.num_cameras
        T_world = [None] * self.num_cameras
        R_world[0] = np.eye(3)
        T_world[0] = np.zeros((3, 1))

        visited = {0}
        queue   = deque([0])

        while queue:
            src = queue.popleft()
            for dst, (R_src_dst, T_src_dst) in adj[src].items():
                if dst in visited:
                    continue
                # compose: world <- src <- dst
                # X_world = R_world_src @ (R_src_dst @ X_dst + T_src_dst) + T_world_src
                R_world[dst] = R_world[src] @ R_src_dst
                T_world[dst] = R_world[src] @ T_src_dst + T_world[src]
                visited.add(dst)
                queue.append(dst)

        # report any unreachable cameras
        unreachable = [i for i in range(self.num_cameras) if R_world[i] is None]
        if unreachable:
            self.state.log(
                f"WARNING: cameras {unreachable} are not connected to Cam0. "
                "They will be excluded from bundle adjustment and triangulation."
            )
            # fill with identity so downstream code does not crash
            for i in unreachable:
                R_world[i] = np.eye(3)
                T_world[i] = np.zeros((3, 1))
        else:
            self.state.log("BFS complete — all cameras initialised in world frame")

        self._log_poses(R_world, T_world)
        return R_world, T_world

    # ──────────────────────────────────────────────────────────────
    # Graph diagnostics  (SCALING_8_CAMERAS.md §2.2)
    # ──────────────────────────────────────────────────────────────
    def _report_graph_health(self, pair_transforms):
        """
        Reports loop count and flags weak-link cameras.

        For a ring of N cameras the spanning tree needs N-1 edges; any edges
        beyond that form independent loops, which give bundle adjustment the
        redundancy it needs to balance error around the ring. A camera touched
        by only one edge has no redundancy — if that single stereo pair is bad,
        the camera's pose is unrecoverable.
        """
        degree = {i: 0 for i in range(self.num_cameras)}
        for (i, j) in pair_transforms.keys():
            degree[i] += 1
            degree[j] += 1

        num_edges = len(pair_transforms)
        # independent loops = edges - (nodes - 1) for a connected graph
        loops = num_edges - (self.num_cameras - 1)

        self.state.log("\n── Pose graph health ──")
        self.state.log(f"  Cameras: {self.num_cameras}  Edges (pairs): {num_edges}")
        if loops > 0:
            self.state.log(f"  Independent loops: {loops} (good — gives BA redundancy)")
        elif loops == 0:
            self.state.log("  Independent loops: 0 (tree only — no loop closure; "
                           "consider closing the ring for a robust solve)")
        else:
            self.state.log("  WARNING: graph has fewer edges than a spanning tree "
                           "— some cameras cannot be reached.")

        weak = [c for c, d in degree.items() if d <= 1]
        if weak:
            self.state.log(
                f"  WARNING: weak-link cameras {weak} are held by a single pair. "
                "Capture more overlap so each has >=2 connecting pairs."
            )
        else:
            self.state.log("  All cameras have >=2 connecting pairs (no weak links)")

    # ──────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────
    @staticmethod
    def _invert_transform(R, T):
        """Invert a rigid body transform.  R_inv = R.T,  T_inv = -R.T @ T"""
        R_inv = R.T
        T_inv = -R.T @ T
        return R_inv, T_inv

    def _log_poses(self, R_world, T_world):
        self.state.log("\n── Initial world poses (before bundle adjustment) ──")
        for i in range(self.num_cameras):
            T_mm = T_world[i].flatten() * 1000
            self.state.log(
                f"  Cam{i}  T_world = [{T_mm[0]:.1f}, {T_mm[1]:.1f}, {T_mm[2]:.1f}] mm"
            )