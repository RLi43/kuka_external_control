# KUKA External Control

![Project Status](https://img.shields.io/badge/status-in%20development-orange?style=for-the-badge)


## Features

- Activate KUKA robotics application in AUT mode;
- Update KUKA Controller parameters (e.g. stiffness, damping);
- Fetch robot information which is not available in some FRI packages (e.g. external force and torque applied on the flange)

## Performance

- Latency and bandwidth [TODO]
    - [ ] Add timestamp in packets. 
    - [ ] Synchronize/Calibrate time between PC and Cabinet.

## Structures

The `KUKA_UDP` on the PC side implements
- `ExternalController` which activates KUKA robotics applications;
- `UDPClient` which communicates with UDP Parameter Server (`ControlParamUpdater`) on KUKA cabinet.

`protocol.md` describes the protocol used for communication.

Note: FRI Client should not exit when FRI session state drops from `COMMANDING_ACTIVE`.

## Credits

- FRI configuration in `ControlParamUpdater.java` is taken from [LBRServer.java](https://github.com/lbr-stack/fri/blob/fri-2.5/server_app/LBRServer.java) by [lbr-stack](https://github.com/lbr-stack/lbr_fri_ros2_stack/)