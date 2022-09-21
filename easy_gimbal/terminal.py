import dearpygui.dearpygui as dpg
from easy_gimbal.video import GimbalVideoUI
from easy_gimbal.user_commands import GimbalControlUI
from easy_gimbal.telemetry import GimbalTelemetryUI

from typing import Optional
from enum import Enum
import abc


class GimbalBase:
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def run(self):
        raise NotImplemented


class GimbalConsole:
    def __init__(self, video: Optional[GimbalVideoUI] = None,
                 control: Optional[GimbalControlUI] = None,
                 telemetry: Optional[GimbalTelemetryUI] = None):
        self.video = video
        self.video_iter = None
        self.control = control
        self.telemetry = telemetry

        self.init_gui()

    def init_gui(self):
        dpg.create_context()

        if self.video is not None:
            with dpg.texture_registry(show=False):
                self.video.register_texture()

            with dpg.window(label="Video player"):
                dpg.add_image(self.video.texture_tag)

        self.control.init_gui()
        if self.telemetry is not None:
            self.telemetry.init_gui()

        dpg.create_viewport(title='Dashboard', width=800, height=600)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def loop(self):
        print('starting...')
        if self.video is not None:
            self.video.start()
        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()  # render GUI


if __name__ == '__main__':
    console = GimbalConsole(video=GimbalVideoUI('udp://localhost:20000'),
                            control=GimbalControlUI(),
                            )
    console.loop()
    dpg.destroy_context()
