import dearpygui.dearpygui as dpg
import numpy as np
import av

from typing import Optional
from enum import Enum
import abc

import rospy

from gimbal_uavision_msgs.msg import (PanSpeedCmd, TiltSpeedCmd, ZoomAbsoluteCmd,
                                      ZoomSpeedCmd, ChangeResolutionCmd, ChangeSensorCmd)
from feed4609_msgs.msg import GimbalTelemetry

def load_video(fn):
    video = av.open(fn)
    fmt = 'rgb24'
    for f in video.decode():
        cf = f.to_ndarray(format=fmt)
        yield cf

    video.close()


class GimbalBase:
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def run(self):
        raise NotImplemented

TAG_TEXTURE_BASE = 0


def get_texture_name():
    global TAG_TEXTURE_BASE
    TAG_TEXTURE_BASE += 1
    return f'video_texture_{TAG_TEXTURE_BASE}'


class GimbalVideoUI(GimbalBase):
    def __init__(self, src: str,
                       width: int = 1280*2, height: int = 720*2, depth: int = 3):
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


class GimbalTelemetryUI(GimbalBase):
    def __init__(self, gimbal_telemetry_topic: str = '/ground/gimbal/telemetry'):
        self.tele_sub = rospy.Subscriber(gimbal_telemetry_topic, GimbalTelemetry, self.update_telemetry)
        self.tele = ''

    def init_gui(self):
        with dpg.window(label='Telemetry'):
            self.tele_text_box = dpg.add_text('')

    def update_telemetry(self, msg):
        self.tele = str(msg)
        dpg.set_value(self.tele_text_box, self.tele)


def which_key(sender, x, y):
    print(sender, x, y)


class ControlKeys(Enum):
    ColorSensor=320  # number 0 in numpad
    IrSensor=321  # number 1 in numpad
    PanLeft=263  # left arrow
    PanRight=262  # right arrow
    TiltUp=265  # up arow
    TiltDown=264  # down arrow
    PanTiltStop = 32  # space
    ZoomIn=334  # plus in numpad
    ZoomOut=333  # minus in numpad
    ZoomStop = 335  # enter numpad




class GimbalControlUI(GimbalBase):
    def __init__(self,
                 pan_speed_topic: str = '/gimbal/pan/speed_cmd',
                 tilt_speed_topic: str = '/gimbal/tilt/speed_cmd',
                 zoom_abs_topic: str = '/gimbal/zoom/position_cmd',
                 zoom_speed_topic: str = '/gimbal/zoom/speed_cmd',
                 sensor_topic: str = '/gimbal/sensor/change_cmd',
                 resolution_topic: str = '/gimbal/sensor/resolution_cmd',
                 publish_queue_size: int = 10):
        self.publishers = {
            'pan_speed': rospy.Publisher(pan_speed_topic, PanSpeedCmd, queue_size=publish_queue_size),
            'tilt_speed': rospy.Publisher(tilt_speed_topic, TiltSpeedCmd, queue_size=publish_queue_size),
            'zoom_absolute': rospy.Publisher(zoom_abs_topic, ZoomAbsoluteCmd, queue_size=publish_queue_size),
            'zoom_speed': rospy.Publisher(zoom_speed_topic, ZoomSpeedCmd, queue_size=publish_queue_size),
            'resoluton': rospy.Publisher(resolution_topic, ChangeResolutionCmd, queue_size=publish_queue_size),
            'change_sensor': rospy.Publisher(sensor_topic, ChangeSensorCmd, queue_size=publish_queue_size),
        }


    def init_gui(self):
        with dpg.window(label='Tools'):
            dpg.add_text('Left arrow - pan left')
            dpg.add_text('Right arrow - pan right')

            dpg.add_text('Up arrow - tilt up')
            dpg.add_text('Down arrow - tilt down')

            dpg.add_text('1 numpad - visible spectrum sensor')
            dpg.add_text('2 numpad - ir sensor')

            dpg.add_text('+ plus numpad - zoom in')
            dpg.add_text('- minus numpad - zoom out')

            dpg.add_text('Commands are active during 1 second')

            dpg.add_slider_int(label="zoom speed", default_value=3, min_value=0, max_value=7,
                               callback=self.set_zoom_speed, tag='zoom_speed_slider')
            dpg.add_slider_int(label="pan/tilt speed", default_value=200, min_value=0, max_value=512,
                               callback=self.set_pan_tilt_speed, tag='pan_tilt_speed_slider')


        def change_text(sender, app_data):
            dpg.set_value("text_item", f"Mouse Button: {app_data[0]}, Down Time: {app_data[1]} seconds")

        with dpg.handler_registry():
            dpg.add_key_press_handler(callback=which_key)
            dpg.add_key_press_handler(key=ControlKeys.ColorSensor.value, callback=self.change_to_color)
            dpg.add_key_press_handler(key=ControlKeys.IrSensor.value, callback=self.change_to_ir)

            dpg.add_key_press_handler(key=ControlKeys.ZoomIn.value, callback=self.zoom_in)
            dpg.add_key_press_handler(key=ControlKeys.ZoomOut.value, callback=self.zoom_out)
            dpg.add_key_press_handler(key=ControlKeys.ZoomStop.value, callback=self.zoom_stop)

            dpg.add_key_press_handler(key=ControlKeys.PanLeft.value, callback=self.pan_left)
            dpg.add_key_press_handler(key=ControlKeys.PanRight.value, callback=self.pan_right)

            dpg.add_key_press_handler(key=ControlKeys.TiltUp.value, callback=self.tilt_up)
            dpg.add_key_press_handler(key=ControlKeys.TiltDown.value, callback=self.tilt_down)

            dpg.add_key_press_handler(key=ControlKeys.PanTiltStop.value, callback=self.pan_tilt_stop)

    def change_to_color(self):
        rospy.loginfo('changing sensor to visible spectrum')
        msg = ChangeSensorCmd(sensor_id=0)
        self.publishers['change_sensor'].publish(msg)

    def change_to_ir(self):
        rospy.loginfo('changing sensor to ir')
        msg = ChangeSensorCmd(sensor_id=1)
        self.publishers['change_sensor'].publish(msg)

    def zoom_in(self):
        s = dpg.get_value('zoom_speed_slider')
        rospy.loginfo(f'zoom in speed {s}')
        msg = ZoomSpeedCmd(speed=s)
        self.publishers['zoom_speed'].publish(msg)

    def zoom_out(self):
        s = dpg.get_value('zoom_speed_slider')
        rospy.loginfo(f'zoom out speed {s}')
        msg = ZoomSpeedCmd(speed=-s)
        self.publishers['zoom_speed'].publish(msg)

    def zoom_stop(self):
        rospy.loginfo('zoom stop')
        msg = ZoomSpeedCmd(speed=0)
        self.publishers['zoom_speed'].publish(msg)

    def pan_left(self):
        s = dpg.get_value('pan_tilt_speed_slider')
        msg = PanSpeedCmd(speed=-s)
        self.publishers['pan_speed'].publish(msg)

    def pan_right(self):
        s = dpg.get_value('pan_tilt_speed_slider')
        msg = PanSpeedCmd(speed=s)
        self.publishers['pan_speed'].publish(msg)

    def tilt_up(self):
        s = dpg.get_value('pan_tilt_speed_slider')
        msg = TiltSpeedCmd(speed=-s)
        self.publishers['tilt_speed'].publish(msg)

    def tilt_down(self):
        s = dpg.get_value('pan_tilt_speed_slider')
        msg = TiltSpeedCmd(speed=s)
        self.publishers['tilt_speed'].publish(msg)

    def pan_tilt_stop(self):
        s = dpg.get_value('pan_tilt_speed_slider')
        msg = TiltSpeedCmd(speed=0)
        self.publishers['tilt_speed'].publish(msg)
        msg = PanSpeedCmd(speed=0)
        self.publishers['pan_speed'].publish(msg)

    def set_zoom_speed(self, *args):
        rospy.loginfo(f'{args}')
        print(args)

    def set_pan_tilt_speed(self, *args):
        rospy.loginfo(f'{args}')
        print(args)


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

        while dpg.is_dearpygui_running():
            if self.video is not None:
                next(self.video_iter)
            dpg.render_dearpygui_frame()  # render GUI


if __name__ == '__main__':
    console = GimbalConsole(video= GimbalVideoUI('udp://localhost:20000'),
                            control = GimbalControlUI(),
                            )
    console.loop()
    dpg.destroy_context()
