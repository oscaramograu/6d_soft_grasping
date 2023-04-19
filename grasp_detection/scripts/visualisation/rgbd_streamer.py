import data_preparation.imgs_dict_updaters as upd
import cv2
from typing import List

class Streamer():
    def __init__(self, strm_imgs: List[str], imgs_dict_u: upd.DictUpdater) -> None:
        self._stream_imgs = []
        for img in strm_imgs:
            if img in imgs_dict_u.imgs:
                self._stream_imgs.append(img)

        self._img_udtr = imgs_dict_u

    def __call__(self):
        imgs_dict = self._img_udtr()

        for img_key in self._stream_imgs:
            img_val = imgs_dict[img_key]
            cv2.imshow(img_key, img_val)

    def stream(self):
        while True:
            self()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        del self._img_udtr # Destroy img updater object to release the image caption

RGBD_streamer = Streamer(["rgb_l"], upd.RGBD_DictUpdater())
RGBD_streamer.stream()