
0. Types
    - 'X', 'Y', 'Z'
    - 'T' for Translational
    - 'A', 'B', 'C'
    - 'R' for rotational
    - 'N' for null space
    - 'E' for all DoFs
1. Controller Parameters
    a. Position Control
        No parameters
    b. Cartesian Impedance Controller
        - Parameters
            - Damping
            - Stiffness
            - AdditionalControlForce
        - Safety Guards (Motion is aborted if violated) [Not Planned]
            - max cartesian velocity
            - max path deviation
            - max control force
    c. Cartesian Impedance Controller with overlaid force oscillation [Not planned]
    d. Axis-specific impedance controller 
        - Damping
        - Stiffness
2. Motion Parameters

    > Not available for PositionHold?

    a. Cartesian Velocity (mm/s) (not applied to joint overlay)
        - T1: 250 mm/s
    b. JointVelocityRel (%)
    c. CartAcceleration (mm/s^2)
    d. JointAccelerationRel (%)
    e. CartJerk (mm/s^3)
    f. JointJerkRel (%)
    g. OrientationVelocity (rad/s)
    h. OrientationAcceleration (rad/s^2)
    i. OrientationJerk (rad/s^3)
3. LBR status not available in FRI
    a. temperature [Not Planned]
        - Power Drive System
            - FCU of brakes
            - FCU of motors
            - CLB of PDSs
        - Joint
            - Torque sensor boards
            - Motor
    b. External Cartesian forces and torques acting on the robot flange
        - measure frame (statically fixed to the flange), orientation frame (as the coordinate system)
        - reliability check
    c. mastering state [Not Planned]
    d. ready for motion (only useful for background applications) [Not Planned]
        - No safety stop is active
        - The drives are in an error-free state
        - AUT
    e. Motion command activity
    f. Safety Signals [Not Planned]
    g. Referencing state [Not Planned]
4. Operations not available in FRI
    a. Master joints [Not Planned]
5. Tool load data
    a. select different tools