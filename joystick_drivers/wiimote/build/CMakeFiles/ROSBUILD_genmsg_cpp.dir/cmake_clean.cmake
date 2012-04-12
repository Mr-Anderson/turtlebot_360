FILE(REMOVE_RECURSE
  "../src/wiimote/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/wiimote/LEDControl.h"
  "../msg_gen/cpp/include/wiimote/TimedSwitch.h"
  "../msg_gen/cpp/include/wiimote/State.h"
  "../msg_gen/cpp/include/wiimote/RumbleControl.h"
  "../msg_gen/cpp/include/wiimote/IrSourceInfo.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
