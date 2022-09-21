Generic terminal for controlling a gimbal payload.
The terminal was not created to interact directly with a gimbal.
Instead, it's designed to be interoperable with any payload that implements a specific interface.
The idea is to have a specific implementation of this interface for any gimbal.
This terminal will interact with that implementation via network.

Accepted inputs:

- MPEGTS stream with video
- MISB ST 0601.8 formated telemetry (see https://github.com/ciafa-sw/pydroneklv)
- gimbal responses to commands

Outputs:

- gimbal commands
  - zoom in/out speed
  - zoom absolute position
  - sensor change
  - pan/tilt speed
  - pan/tilt absolute position
  - change resolution
  - position lock
  - target
    - target lock
    - keep target same size in image

GUI

- window with received video
- window with received telemetry
- window with gimbal commands and command response


TODO

- receive telemetry
- receive commands response
- send commands
- configure video reception
- configure telemetry reception
- configure command communication