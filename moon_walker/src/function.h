#include "wrapper.h"

void gps_callback(float x, float y, float theta, double timestamp);

void odometry_callback(float x, float y, float theta, double timestamp);

inline void setSpeed(float v, float omega){ _set_speed(v,omega);}
