# G1 HG DDS message headers

This directory contains the complete HG message header dependency set used by
the G1 controller's DDS wrapper:

- `LowCmd_.hpp` and its dependency `MotorCmd_.hpp`.
- `LowState_.hpp` and its dependencies `MotorState_.hpp` and `IMUState_.hpp`.

The files are copied without changes from `include/unitree/idl/hg/` in
Unitree SDK2, commit `63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36`, from the local
`unitree_sdk2` checkout. The upstream BSD 3-Clause license is included in
`LICENSE`. The generated headers identify their generator as Cyclone DDS
v0.10.2; retain their message layouts and serialization code unchanged.

These are architecture-independent C++ headers, usable on both x86_64 and
AArch64. The G1 CMake target adds this directory's `include/` before system
headers and propagates it to the controller and controller-linked tests.
The standalone depth capture tool only links the ROS 2 depth backend and does
not require HG message headers.
Copy this whole directory with the project when using rsync.

This subset supplies message definitions only. The target machine must still
provide Unitree SDK2 channel headers and libraries, plus compatible Cyclone DDS
C/C++ headers and libraries. No development-machine binaries are bundled here.
