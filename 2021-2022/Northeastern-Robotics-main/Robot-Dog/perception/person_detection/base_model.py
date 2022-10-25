# Modify documentation before submitting a PullRequest.
class PersonDetection:
    """Person Detection wrapper for detection using OAK-D and DepthAI API.
    """

    def __init__(self, pipeline=None, labels=None, confidence_threshold=None, weight_path=None):
        """Initiate PersonDetection class.

        Args:
            pipeline : Pipeline object
                Accepts a pipeline object constructed during the Reader initialization.
            labels : list
                Accepts a list of labels for detection.
            confidence_threshold : float
                Accepts a confidence threshold to filter detections.
            weight_path : str
                Accepts a file location to the model weights.
        """
        pass

    def detect(self) -> list:
        """Take a look at the API doc for this function.

        Returns:

        """
        pass
