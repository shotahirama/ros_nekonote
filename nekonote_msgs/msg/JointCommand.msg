uint8 POSITION_CONTROL = 1
uint8 TORQUE_CONTROL = 2

uint8 mode  # command mode
string[]  name  # joint name
float64[] command  # desired joint position(rad) or joint torque(Nm), same order as joint name
int64 time  # time(msec) to execute command when mode is POSITION_CONTROL. If mode is TORQUE_CONTROL, this value is ignored.
