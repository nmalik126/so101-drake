from multiprocessing.shared_memory import SharedMemory
from multiprocessing import resource_tracker
import numpy as np
import cv2
from pathlib import Path


def main():
    try:
        color_shm = SharedMemory(name="color_img")
        resource_tracker.unregister(color_shm._name, "shared_memory")

        color_image = np.ndarray((480, 848, 3), dtype=np.uint8, buffer=color_shm.buf)
        color_snapshot = color_image.copy()
        print(color_snapshot.shape)

        project_dir = Path("/home/noor/so101-drake")
        cv2.imwrite(project_dir / "assets" / "color_shm_example.png", color_snapshot)
    finally:
        color_shm.close()


if __name__ == "__main__":
    main()
