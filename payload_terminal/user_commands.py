from enum import Enum
import dearpygui.dearpygui as dpg
import comm
import commands


def which_key(sender, x, y):
    print(sender, x, y)


class ControlKeys(Enum):
    ColorSensor = 320  # number 0 in numpad
    IrSensor = 321  # number 1 in numpad
    PanLeft = 263  # left arrow
    PanRight = 262  # right arrow
    TiltUp = 265  # up arow
    TiltDown = 264  # down arrow
    PanTiltStop = 32  # space
    ZoomIn = 334  # plus in numpad
    ZoomOut = 333  # minus in numpad
    ZoomStop = 335  # enter numpad


class GimbalControlUI:
    is_zoom_active: int  # 0 is false, >0 is zoom in, <0 is zoom out
    is_tilt_active: int  # 0 is false, >0 is tilt up, <0 is tilt down TODO check
    is_pan_active: int  # 0 is false, >0 is pan right, <0 is pan left TODO check

    def init_gui(self):
        with dpg.window(label='Tools'):
            dpg.add_text('Left arrow - pan left/stop pan right')
            dpg.add_text('Right arrow - pan right/stop pan left')

            dpg.add_text('Up arrow - tilt up/stop tilt down')
            dpg.add_text('Down arrow - tilt down/stop tilt up')

            dpg.add_text('0 numpad - visible spectrum sensor')
            dpg.add_text('1 numpad - ir sensor')

            dpg.add_text('+ plus numpad - zoom in/stop zoom out')
            dpg.add_text('- minus numpad - zoom out/stop zoom in')

            dpg.add_text('space bar - stop pan, tilt and zoom')

            #dpg.add_text('Commands are active during 1 second')

            dpg.add_slider_int(label="zoom speed", default_value=3, min_value=0, max_value=7,
                               callback=self.set_zoom_speed, tag='zoom_speed_slider')
            dpg.add_slider_int(label="zoom pos", default_value=3, min_value=0, max_value=7,
                               callback=self.set_zoom_speed, tag='zoom_speed_slider')
            dpg.add_slider_int(label="pan speed", default_value=200, min_value=0, max_value=512,
                               callback=self.set_pan_tilt_speed, tag='pan_speed_slider')
            dpg.add_slider_int(label="tilt speed", default_value=200, min_value=0, max_value=512,
                               callback=self.set_pan_tilt_speed, tag='tilt_speed_slider')
            dpg.add_slider_int(label="pan pos", default_value=200, min_value=0, max_value=512,
                               callback=self.set_pan_tilt_speed, tag='pan_pos_slider')
            dpg.add_slider_int(label="tilt pos", default_value=200, min_value=0, max_value=512,
                               callback=self.set_pan_tilt_speed, tag='tilt_pos_slider')

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
        # rospy.loginfo('changing sensor to visible spectrum')
        comm.send_cmd(commands.SensorChange(sensor_id=0))

    def change_to_ir(self):
        # rospy.loginfo('changing sensor to ir')
        comm.send_cmd(commands.SensorChange(sensor_id=1))

    def zoom_in(self):
        if self.is_zoom_active < 0:  # stop zoom if zooming out
            self.zoom_stop()
        else:
            s = dpg.get_value('zoom_speed_slider')
            comm.send_cmd(commands.ZoomSpeed(speed=+s))
            self.is_zoom_active = +s

    def zoom_out(self):
        if self.is_zoom_active > 0:  # stop zoom if zooming in
            self.zoom_stop()
        else:
            s = dpg.get_value('zoom_speed_slider')
            comm.send_cmd(commands.ZoomSpeed(speed=-s))
            self.is_zoom_active = -s

    def zoom_stop(self):
        self.is_zoom_active = 0
        comm.send_cmd(commands.ZoomSpeed(speed=0))

    def pan_stop(self):
        comm.send_cmd(commands.PanTiltSpeed(pan_speed=0,
                                            tilt_speed=self.is_tilt_active))
        self.is_pan_active = 0

    def pan_left(self):
        if self.is_pan_active > 0:
            self.pan_stop()
        else:
            s = dpg.get_value('pan_speed_slider')
            comm.send_cmd(commands.PanTiltSpeed(pan_speed=-s,
                                                tilt_speed=self.is_tilt_active))
            self.is_pan_active = -s

    def pan_right(self):
        if self.is_pan_active < 0:
            self.pan_stop()
        else:
            s = dpg.get_value('pan_speed_slider')
            comm.send_cmd(commands.PanTiltSpeed(pan_speed=+s,
                                                tilt_speed=self.is_tilt_active))
            self.is_pan_active = +s

    def tilt_stop(self):
        comm.send_cmd(commands.PanTiltSpeed(tilt_speed=0,
                                            pan_speed=self.is_pan_active))
        self.is_tilt_active = 0

    def tilt_up(self):
        if self.is_tilt_active < 0:
            self.tilt_stop()
        else:
            s = dpg.get_value('tilt_speed_slider')
            comm.send_cmd(commands.PanTiltSpeed(tilt_speed=+s,
                                                pan_speed=self.is_pan_active))
            self.is_tilt_active = +s

    def tilt_down(self):
        if self.is_tilt_active > 0:
            self.tilt_stop()
        else:
            s = dpg.get_value('tilt_speed_slider')
            comm.send_cmd(commands.PanTiltSpeed(tilt_speed=-s,
                                                pan_speed=self.is_pan_active))
            self.is_tilt_active = -s
