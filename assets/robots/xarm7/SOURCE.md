# xArm7 viewer model

Copied from the local `mujoco_menagerie-main/ufactory_xarm7` distribution.
Upstream: https://github.com/google-deepmind/mujoco_menagerie/tree/main/ufactory_xarm7

The original `xarm7_nohand.xml`, its nine referenced STL files, and
BSD-3-Clause LICENSE are included unchanged. This is a simplified xArm7 model
derived from UFACTORY's public robot description.

The viewer removes the MJCF's 0.12 m scene placement of `link_base` at runtime
so the displayed base frame is the robot's physical base coordinate frame.
The model's flange pose is compared against the live SDK pose after removing
the configured TCP offset. MuJoCo is used for forward kinematics only; no
physics integration or robot commands are performed.
