import dearpygui.dearpygui as dpg


class GimbalTelemetryUI():
    def __init__(self, gimbal_telemetry_src: str = 'telemetry'):
        #self.tele_sub = rospy.Subscriber(gimbal_telemetry_topic, GimbalTelemetry, self.update_telemetry)
        self.tele = ''

    def init_gui(self):
        with dpg.window(label='Telemetry'):
            self.tele_text_box = dpg.add_text('')

    def update_telemetry(self, msg):
        self.tele = str(msg)
        dpg.set_value(self.tele_text_box, self.tele)