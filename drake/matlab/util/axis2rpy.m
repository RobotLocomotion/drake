function rpy = axis2rpy(axis_angle)

rpy = quat2rpy(axis2quat(axis_angle));
