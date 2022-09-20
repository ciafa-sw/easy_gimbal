import dearpygui.dearpygui as dpg
from payload_terminal.video import GimbalVideoUI
from payload_terminal.user_commands import GimbalControlUI
from payload_terminal.telemetry import GimbalTelemetryUI

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
            self.video_iter = self.video.run()

        self.control.init_gui()
        if self.telemetry is not None:
            self.telemetry.init_gui()

        dpg.create_viewport(title='Dashboard', width=800, height=600)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def loop(self):
        print('starting...')
        while dpg.is_dearpygui_running():
            if self.video is not None:
                next(self.video_iter)
            dpg.render_dearpygui_frame()  # render GUI


if __name__ == '__main__':
    console = GimbalConsole(video=GimbalVideoUI('udp://localhost:20000'),
                            control=GimbalControlUI(),
                            )
    console.loop()
    dpg.destroy_context()