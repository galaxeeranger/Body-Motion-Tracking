import cv2
from config import CHARUCO_ROWS, CHARUCO_COLS, SQUARE_LENGTH, MARKER_LENGTH

class BoardConfig:
    """Owns ChArUco board creation and physical parameters."""

    def __init__(self):
        aruco            = cv2.aruco
        self.rows        = CHARUCO_ROWS
        self.cols        = CHARUCO_COLS
        self.square_length = SQUARE_LENGTH
        self.marker_length = MARKER_LENGTH
        self.dictionary  = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.board       = aruco.CharucoBoard(
            (self.cols, self.rows),
            self.square_length,
            self.marker_length,
            self.dictionary
        )

    def to_dict(self):
        return {
            "rows":          self.rows,
            "cols":          self.cols,
            "square_length": self.square_length
        }