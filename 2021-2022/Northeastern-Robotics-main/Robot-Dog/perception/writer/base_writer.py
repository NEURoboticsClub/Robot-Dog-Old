import cv2
import numpy as np


class Writer:
    """Video Writing wrapper around Opencv-Backend
    """

    # opencv fourcc mappings
    _EXT_TO_FOURCC = {".avi": "DIVX", ".mkv": "X264", ".mp4": "mp4v"}

    def __init__(self, path, fps, width, height, ext=".mp4"):
        """Initiate Writer object

        Args:
            path : str
                Path to output video
            fps : int
                Frames per second
            width : int
                Width of output video
            height : int
                Height of output video
            ext : str
                Extension of output video
        """
        self._video_writer = cv2.VideoWriter(path, self._fourcc(ext), fps, (width, height))

        # check if open
        if not self.is_open():
            raise AssertionError(
                "Failed to Create Writer for the given settings.")

    def _fourcc(self, ext) -> cv2.VideoWriter_fourcc:
        """Returns CV2 VideoWriter_fourcc for writer's ext

        Args:
            ext : str
                extension of output video
        Raises:
            NotImplementedError: raise if unsupported ext is used.
        Returns:
            cv2.VideoWriter_fourcc: fourcc of used ext
        """
        if ext not in self._EXT_TO_FOURCC:
            raise NotImplementedError(f"'{ext}'is not supported.")
        return cv2.VideoWriter_fourcc(*self._EXT_TO_FOURCC[ext])

    def is_open(self) -> bool:
        """Checks if writer is still open
        Returns:
            bool: True if writer is open, False otherwise
        """
        return self._video_writer.isOpened()

    def write(self, frame: np.ndarray) -> None:
        """Write frame to output video
        Args:
            frame : np.ndarray
                frame to write
        Raises:
            Exception: raised when method is called on a non-open writer.
        """
        # check if writer is open
        if not self.is_open():
            raise Exception(
                "[Error] Attempted writing with a non-open Writer.")

        self._video_writer.write(frame)

    def release(self) -> None:
        """Release Resources
        """
        if self._video_writer is not None:
            self._video_writer.release()

    def __del__(self) -> None:
        """Release Resources
        """
        self.release()
        self._video_writer = None

    def __exit__(self, exc_type: None, exc_value: None,
                 traceback: None) -> None:
        """Release resources before exiting the "with" block

        Args:
            exc_type : NoneType
                Exception type if any
            exc_value : NoneType
                Exception value if any
            traceback : NoneType
                Traceback of Exception
        """
        self.release()
