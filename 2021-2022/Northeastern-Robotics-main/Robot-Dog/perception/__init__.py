import os
import cv2
import pathlib
from reader import Reader
from writer import Writer


if __name__ == '__main__':
    path_of_output = os.path.join(pathlib.Path().resolve(), "test.mp4")
    reader = Reader()
    writer = None
    while True:
        if writer is None:
            writer = Writer(path_of_output, reader.get_fps(), reader.get_width(), reader.get_height())

        frame = reader.read()
        writer.write(frame)
        cv2.imshow("Video", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Exiting...")
    cv2.destroyAllWindows()
