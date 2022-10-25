import depthai as dai
import numpy


class Reader:
    """Video reading wrapper around OAK-D using DepthAI API.
    """
    def __init__(self, color=True, camera="rgb", resolution=1080, video_size=(1920, 1080), fps=30):
        """Initiate a Reader class.

        Args:
            color : bool
                true for color image else mono. [default: True]
            camera : str
                use the rgb, left, or right camera. [default: "rgb]
            resolution : int
                resolution to read from the camera. [default: 1080]
            video_size : tuple
                video size to read (width, height). [default: (1920, 1080)]
            fps : int
                frames per second. [default: 30]
        """
        self._device = None
        self._color = color
        self._pipeline = dai.Pipeline()
        if self._color:
            self._cam = self._pipeline.create(dai.node.ColorCamera)
        self._out = self._pipeline.create(dai.node.XLinkOut)
        self._stream = "Video"
        self._set_properties(camera, resolution, video_size, fps)
        self._set_output_property()
        self._link()
        self._set_device()

    def _link(self) -> None:
        """Link the reader to the pipeline.
        """
        self._cam.video.link(self._out.input)

    def _set_output_property(self) -> None:
        """Set the output property of the reader.
        """
        self._out.setStreamName(self._stream)
        self._out.input.setBlocking(False)
        self._out.input.setQueueSize(1)

    def _set_properties(self, camera, resolution, video_size, fps) -> None:
        """Set the properties of the input stream.

        Args:
            camera : str
                use the rgb, left, or right camera. [default: "rgb]
            resolution : int
                resolution to read from the camera. [default: 1080]
            video_size : tuple
                video size to read (width, height). [default: (1920, 1080)]
            fps : int
                frames per second. [default: 30]
        """
        if camera == "rgb":
            self._cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        if resolution == 1080:
            self._cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        # set video size
        self._cam.setVideoSize(video_size)
        # set fps
        self._cam.setFps(fps)

    @property
    def pipeline(self) -> dai.Pipeline:
        """Get the DepthAI pipeline object.

        Returns:
            dai.Pipeline: pipeline object (DepthAI)
        """
        return self._pipeline

    @property
    def color(self) -> bool:
        """Get the color status of the feed (rgb or mono).

        Returns:
            bool: color status (RGB/MONO).
        """
        return self._color

    @property
    def stream(self) -> str:
        """Get the stream name.

        Returns:
            str: stream name.
        """
        return self._stream

    def __repr__(self) -> str:
        """Get the representation of the Reader object.

        Returns:
            str: representation of the Reader object.
        """
        return f"Reader(color={self._color}, camera={self._cam}, pipeline={self._pipeline})"

    def _set_device(self) -> None:
        """Set the device to the reader.
        """
        self._device = dai.Device(self._pipeline)

    def __iter__(self) -> "Reader":
        """Get the iterator of the Reader object.

        Returns:
            Reader: iterator of the Reader object.
        """
        return self

    def __next__(self) -> numpy.ndarray:
        """Get the next frame from the reader.

        Returns:
            numpy.ndarray: next frame from the reader.
        """
        return self._device.getOutputQueue(name=self._stream, maxSize=1, blocking=False).get().getCvFrame()

    def read(self) -> numpy.ndarray:
        """Get the next frame from the reader.

        Returns:
            numpy.ndarray: next frame from the reader.
        """
        return self.__next__()

    def get_fps(self) -> int:
        """Get the FPS of the reader.

        Returns:
            int: FPS of the reader.
        """
        return self._cam.getFps()

    def get_width(self) -> int:
        """Get the width of the reader.

        Returns:
            int: width of the reader.
        """
        return self._cam.getVideoWidth()

    def get_height(self) -> int:
        """Get the height of the reader.

        Returns:
            int: height of the reader.
        """
        return self._cam.getVideoHeight()
