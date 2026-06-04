import cv2

class FrameDetector:
    """Detects ChArUco corners in a single grayscale frame."""

    def __init__(self, board_config):
        self.cfg = board_config

    def detect(self, gray):
        """Returns (success, charuco_corners, charuco_ids)."""
        aruco = cv2.aruco
        corners, ids, _ = aruco.detectMarkers(gray, self.cfg.dictionary)

        if ids is None or len(ids) == 0:
            return False, None, None

        _, ch_corners, ch_ids = aruco.interpolateCornersCharuco(
            corners, ids, gray, self.cfg.board
        )

        if ch_ids is not None and len(ch_ids) > 5:
            return True, ch_corners, ch_ids

        return False, None, None

    def draw_on_frame(self, frame, gray):
        """Draws detected markers onto frame in-place. Returns detection result."""
        aruco = cv2.aruco
        corners, ids, _ = aruco.detectMarkers(gray, self.cfg.dictionary)
        ok, ch_corners, ch_ids = self.detect(gray)

        if ok:
            aruco.drawDetectedMarkers(frame, corners, ids)
            aruco.drawDetectedCornersCharuco(frame, ch_corners, ch_ids)

        return ok, ch_corners, ch_ids