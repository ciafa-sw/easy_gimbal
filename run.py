from payload_terminal.video import GimbalVideoUI
from payload_terminal.telemetry import GimbalTelemetryUI
from payload_terminal.user_commands import GimbalControlUI
from payload_terminal.terminal import GimbalConsole

import dearpygui.dearpygui as dpg


def main():

    src = 'localhost:30000'
    telemetry_topic = None

    video_module = GimbalVideoUI(src) if src is not None else None
    telemetry_module = GimbalTelemetryUI() if telemetry_topic is not None else None

    console = GimbalConsole(video=video_module,
                            control=GimbalControlUI(),
                            telemetry=telemetry_module)
    console.loop()
    dpg.destroy_context()
