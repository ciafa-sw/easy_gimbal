from easy_gimbal.video import GimbalVideoUI
from easy_gimbal.telemetry import GimbalTelemetryUI
from easy_gimbal.user_commands import GimbalControlUI
from easy_gimbal.terminal import GimbalConsole

import dearpygui.dearpygui as dpg


def main():

    src = 'udp://localhost:30000'
    telemetry_topic = None

    video_module = GimbalVideoUI(src) if src is not None else None
    telemetry_module = GimbalTelemetryUI() if telemetry_topic is not None else None

    console = GimbalConsole(video=video_module,
                            control=GimbalControlUI(),
                            telemetry=telemetry_module)
    console.loop()
    dpg.destroy_context()


if __name__ == '__main__':
    main()