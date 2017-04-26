# This is a message to hold data from an Mikrokopter IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational angle should be in rad
#
# by Inkyu

Header header

float64 anglePitch
float64 angleRoll
float64 angleYaw
float64 gyroPitch
float64 gyroRoll
float64 gyroYaw
float64 accelX
float64 accelY
float64 accelZ
int32 motor1
int32 motor2
int32 motor3
int32 motor4
int32 motor5
int32 motor6
float64 baromHeight
int32 batt
int32 extern_on

geometry_msgs/Vector3 linear_acceleration
