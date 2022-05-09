import rospy

import dearpygui.dearpygui as dpg

from payload_terminal.terminal import GimbalConsole, GimbalControlUI, GimbalVideoUI, GimbalTelemetryUI


def main():
    rospy.init_node('payload_terminal', log_level=rospy.INFO)

    src = rospy.get_param("/payload_terminal/video_src")

    console = GimbalConsole(video=GimbalVideoUI(src),
                            control=GimbalControlUI(),
                            telemetry=GimbalTelemetryUI())
    console.loop()
    dpg.destroy_context()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass