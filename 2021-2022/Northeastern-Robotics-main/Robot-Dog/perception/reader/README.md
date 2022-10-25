# Reader for DepthAI and OAK-D
Common wrapper for reading images and video from the OAK-D camera using the DepthAI API.

## Read Video
```python
import cv2
from perception.reader import Reader

if __name__ == '__main__':
    # Create a reader object
    reader = Reader()
    while True:
        # Read frames in a while loop
        cv2.imshow("Video", reader.read())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Exiting...")
    cv2.destroyAllWindows()
```
