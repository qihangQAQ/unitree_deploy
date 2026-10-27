# G1 DDS wrapper headers

This directory carries the header-only DDS wrapper subset used by the G1 29DoF
controller, so copying the repository also copies this build dependency.

The files were taken from the development machine's installed
`/usr/local/include/unitree/dds_wrapper` headers. Their original Unitree copyright
notices are retained. The subset contains the publisher/subscriber helpers, CRC,
joystick decoding, and G1 LowCmd/LowState wrappers. Unused robot, arm, and hand
wrappers and their IDL includes are omitted; direct standard-library includes
were added. The retained communication and joystick behavior is unchanged.
Publishing uses `ChannelPublisher::Write(message)` to support older SDK2
versions that only accept one argument. Newer SDK2 versions default their
optional `waitMicrosec` argument to zero, matching the original call.

These headers are independent of CPU architecture. They do **not** replace
Unitree SDK2 or Cyclone DDS: install those libraries for the target architecture.
G1's CMake build locates the SDK2 library in standard search locations or
`/opt/unitree_robotics/lib{,64}` and links its full path. Set
`UNITREE_SDK2_LIBRARY` to override the library path on the target machine.
Following the layout exported by the board's `unitree_sdk2Targets.cmake`,
SDK channel and DDS C headers come from `include/` under the selected SDK
prefix, DDS C++ headers come from `include/ddscxx`, and `libddscxx.so` and
`libddsc.so` are linked by full paths beside the SDK2 library. These paths
are recomputed on each configure. Missing SDK/DDS headers or libraries are
errors rather than reasons to fall back to another system or ROS installation.
No hardcoded `/usr/local` DDS C++ or iceoryx include directory is added.
The build does not require `unitree_sdk2Config.cmake`.
This directory's `include/` is placed ahead of the selected SDK headers. SDK2's HG
LowCmd/LowState IDL headers and their message dependencies are bundled in
`../unitree_sdk2_hg`, whose `include/` also precedes the selected SDK headers. Copy
both directories when deploying. Go2 and hand IDL headers are not needed.
