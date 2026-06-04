import os
import numpy as np
from config import OUTPUT_PATH, CAMERA_COLORS


class CalibSaver:
    """Saves calibration results and pushes camera data to state for 3D viz."""

    def __init__(self, state):
        self.state = state

    def save(self, K_list, dist_list, R_world, T_world, image_size):
        """
        Parameters
        ----------
        K_list     : list of (3,3) intrinsic matrices
        dist_list  : list of dist coefficient arrays
        R_world    : list of (3,3) world-frame rotations  (BA-optimised)
        T_world    : list of (3,1) world-frame translations
        image_size : (width, height)
        """
        os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)

        np.savez(
            OUTPUT_PATH,
            K_list    = np.array(K_list,    dtype=object),
            dist_list = np.array(dist_list, dtype=object),
            R_list    = np.array(R_world,   dtype=object),
            T_list    = np.array(T_world,   dtype=object),
        )

        self.state.log(f"\nCalibration saved -> {OUTPUT_PATH}")

        # build camera data for 3D visualiser
        iw, ih = image_size
        cam_data = []
        for i in range(len(K_list)):
            T_mm = T_world[i].flatten() * 1000
            cam_data.append({
                "idx":   i,
                "name":  f"Cam {i}",
                "color": CAMERA_COLORS[i % len(CAMERA_COLORS)],
                "R":     R_world[i].tolist(),
                "T":     T_world[i].flatten().tolist(),
                "K":     K_list[i].tolist(),
                "imgW":  iw,
                "imgH":  ih,
                "T_mm":  T_mm.tolist(),
            })
            self.state.log(
                f"  Cam{i}  T_world=[{T_mm[0]:.1f},{T_mm[1]:.1f},{T_mm[2]:.1f}] mm"
            )

        self.state.update(phase="done", cameras=cam_data)
        self.state.log("CALIBRATION COMPLETE")