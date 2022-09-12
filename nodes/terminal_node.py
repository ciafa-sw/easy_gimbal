import rospy

import dearpygui.dearpygui as dpg

from payload_terminal.terminal import GimbalConsole, GimbalControlUI, GimbalVideoUI, GimbalTelemetryUI


def main():
    rospy.init_node('payload_terminal', log_level=rospy.INFO)

    src = rospy.get_param("~video_src", None)
    telemetry_topic = rospy.get_param("/payload_terminal/telemetry_topic", None)

    rospy.loginfo(f'params | src={src}   topic={telemetry_topic}')

    video_module = None
    telemetry_module = None
    if src is not None:
        video_module = GimbalVideoUI(src)

    if telemetry_topic is not None:
        telemetry_module = GimbalTelemetryUI()

    console = GimbalConsole(video=video_module,
                            control=GimbalControlUI(),
                            telemetry=telemetry_module)
    console.loop()
    dpg.destroy_context()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass