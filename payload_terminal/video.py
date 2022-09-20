import numpy as np
import dearpygui.dearpygui as dpg
import av


def load_video(fn):
    video = av.open(fn)
    fmt = 'rgb24'
    for f in video.decode():
        cf = f.to_ndarray(format=fmt)
        yield cf

    video.close()


# automatically create new texture names if multiple videos are registered
TAG_TEXTURE_BASE = 0


def get_texture_name():
    global TAG_TEXTURE_BASE
    TAG_TEXTURE_BASE += 1
    return f'video_texture_{TAG_TEXTURE_BASE}'


class GimbalVideoUI():
    def __init__(self, src: str,
                 width: int = 1280 * 2, height: int = 720 * 2, depth: int = 3):
        self.width = width
        self.height = height
        self.depth = depth

        self._raw_texture_data = np.zeros((height, width, depth), dtype=np.float32)
        self.video_src = load_video(src)
        self.texture_tag = None

    def register_texture(self):
        self.texture_tag = get_texture_name()
        with dpg.texture_registry(show=False):
            dpg.add_raw_texture(self.width, self.height,
                                self._raw_texture_data,
                                format=dpg.mvFormat_Float_rgb,
                                tag=self.texture_tag)

    def update_dynamic_texture(self, new_frame: np.ndarray):
        h2, w2, d2 = new_frame.shape  # get real frame size
        self._raw_texture_data[:h2, :w2] = new_frame[:, :] / 255

    def run(self):
        for frame in self.video_src:
            self.update_dynamic_texture(frame)
            yield
