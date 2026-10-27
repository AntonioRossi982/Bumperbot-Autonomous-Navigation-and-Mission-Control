# Integrating MissionStatus.msg into bumperbot_msgs

Here are the three steps to integrate the new message into your existing package.

1. Copy `msg/MissionStatus.msg` into `bumperbot_msgs/msg/` (alongside the other `.msg` files you have already defined).

2. In the `bumperbot_msgs` `CMakeLists.txt`, locate the `rosidl_generate_interfaces(${PROJECT_NAME} ...)` call and add a line for the new file, leaving the existing entries unchanged. For example:

   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/ExistingMsgName1.msg"
     "msg/ExistingMsgName2.msg"
     "msg/MissionStatus.msg"
   )
   ```

   If the package does not already contain a `rosidl_generate_interfaces` call 
   (i.e., if `bumperbot_msgs` has so far contained only services/actions), 
   you must also ensure that `find_package(rosidl_default_generators REQUIRED)` 
   is present in `CMakeLists.txt` and that the `rosidl_default_generators` / `rosidl_default_runtime` 
   dependencies are declared in `package.xml`. In most cases, these are already 
   present because the package is specifically intended to generate interfaces.

3. Rebuild only that package to regenerate the bindings:

   ```bash
   colcon build --packages-select bumperbot_msgs
   source install/setup.bash
   ```

From this point onward, `MissionStatus` can be imported as follows:

- Python: `from bumperbot_msgs.msg import MissionStatus`
- C++: `#include "bumperbot_msgs/msg/mission_status.hpp"`