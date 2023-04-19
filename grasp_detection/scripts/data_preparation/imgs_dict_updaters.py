import data_preparation.rgbd_imgs_ldr as ldr

class DictUpdater():
    def __init__(self) -> None:
        self.imgs = {}

    def __getitem__(self, key: str):
        return self.imgs[key]
    
    def __len__(self):
        return len(self.imgs)

class RGBD_DictUpdater(DictUpdater):
    def __init__(self) -> None:
        self.imgs = {
            "rgb_l": None,
            "rgb_r": None,
            "gs_l": None,
            "gs_r": None,
            "disp_map": None,
            "depth": None                        
        }

        self._rgb_gs_ldr = ldr.StereoImgsDictLoader()
        self._depth_lr = ldr.DepthImgDictLoader()

    def __call__(self):
        rgb_gs_imgs = self._rgb_gs_ldr()

        disp_depth_imgs = self._depth_lr(rgb_gs_imgs["gs_l"], rgb_gs_imgs["gs_r"])

        self.imgs.update(rgb_gs_imgs)
        self.imgs.update(disp_depth_imgs)

        return self.imgs

# du = RGBD_DictUpdater()
# imgs = du()
# print(type(imgs))