FILE(REMOVE_RECURSE
  "../src/wiimote/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/wiimote/msg/__init__.py"
  "../src/wiimote/msg/_LEDControl.py"
  "../src/wiimote/msg/_TimedSwitch.py"
  "../src/wiimote/msg/_State.py"
  "../src/wiimote/msg/_RumbleControl.py"
  "../src/wiimote/msg/_IrSourceInfo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
