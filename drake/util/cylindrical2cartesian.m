function [x_cartesian,v_cartesian,J, Jdotv] = cylindrical2cartesian(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder,v_cylinder)
% Given the axis, the x direction and the origin of the cylinder, compute
% the point x_cartesian in the cylindrical coordinate. The cylindrical
% coordinate is illustrated in drake/doc/doc_cartesian2cylindrical.pdf
% @param cylinder_axis   A 3 x 1 vector. The axis of the cylinder in the
% cartesian frame
% @param cylinder_x_dir  A 3 x 1 vector. the cylinder x direction, the
% direction that theta = 0 is measured.
% @param cylinder_origin A 3 x 1 vector. The origin (height = 0) of the
% cylinder
% @param x_cylinder     A 6 x 1 vector. [radius;theta;height;roll;pitch;yaw] in the
% cylindrical coordinate
% @param v_cylinder     A 6 x 1 vector. [radius_dot;theta_dot;height_dot;angular_vel_x;angular_vel_y;angular_vel_z]
% in the cylindrical coordinate. The angular velocity is measured in the
% cylinder tangential frame
% @retval x_cartesian     A 6 x 1 vector. The [x;y;z;roll;pitch;yaw] in the
% cartesian frame
% @retval v_cartesian     A 6 x 1 vector. The [xdot;ydot;zdot;angular_vel_x;angular_vel_y;angular_vel_z] in the
% cartesian frame
% @retval J      A 6 x 6 matrix, the geometric Jacobian, J*v_cylinder =
% v_cartesian
% @retval Jdotv  A 3 x 1 vector, the Jacobian_dot *v_cylinder
sizecheck(cylinder_axis,[3,1]);
sizecheck(cylinder_x_dir,[3,1]);
sizecheck(cylinder_origin,[3,1]);
axis_norm = norm(cylinder_axis);
if(axis_norm<eps)
error('cylinder_axis should be a non-zero vector');
end
x_dir_norm = norm(cylinder_x_dir);
if(x_dir_norm<eps)
error('cylinder_x_dir should be a non-zero vector');
end
cylinder_axis = cylinder_axis/axis_norm;
cylinder_x_dir = cylinder_x_dir/x_dir_norm;
if(abs(cylinder_axis'*cylinder_x_dir)>1e-10)
  error('cylinder_axis and cylinder_x_dir should be perpendicular to each other');
end
R_cylinder2cartesian = [cylinder_x_dir cross(cylinder_axis,cylinder_x_dir) cylinder_axis];

radius = x_cylinder(1);
theta = x_cylinder(2);
height = x_cylinder(3);
radius_dot = v_cylinder(1);
theta_dot = v_cylinder(2);
height_dot = v_cylinder(3);

c_theta = cos(theta);
s_theta = sin(theta);
x_pos_cartesian = R_cylinder2cartesian*[radius*c_theta;radius*s_theta;height]+cylinder_origin;
v_pos_cartesian = R_cylinder2cartesian*[radius*-s_theta*theta_dot + radius_dot*c_theta;...
                              radius*c_theta*theta_dot + radius_dot*s_theta;... 
                              height_dot];
R_tangent = rpy2rotmat(x_cylinder(4:6));
[R_tangent2cylinder,dR_tangent2cylinder_dtheta] = rotz(theta-pi/2);
R_cylinder = R_tangent2cylinder*R_tangent;
R_cartesian = R_cylinder2cartesian*R_cylinder;
x_rpy_cartesian = rotmat2rpy(R_cartesian);
x_cartesian = zeros(6,1);
x_cartesian(1:3) = x_pos_cartesian;
x_cartesian(4:6) = x_rpy_cartesian;
v_cartesian = zeros(6,1);
v_cartesian(1:3) = v_pos_cartesian;
% The angular velocity of the point in the Cartesian frame aligned with the
% cylinder, is omega_theta+R_tangent*omega_cylinder*R_tangent'
v_cartesian(4:6) = R_cylinder2cartesian*([0;0;theta_dot] + R_tangent2cylinder*v_cylinder(4:6));
J = zeros(6,6);
J(1:3,1:3) = R_cylinder2cartesian*[c_theta radius*-s_theta 0;...
                s_theta radius*c_theta 0;...
                0  0  1];
J(4:6,2) = R_cylinder2cartesian(:,3);
J(4:6,4:6) = R_cylinder2cartesian*R_tangent2cylinder;
% Jdotv1 is Jdotv(1:3)
Jdotv1 = R_cylinder2cartesian*([0 -s_theta 0;0 c_theta 0;0 0 0]*radius_dot+[-s_theta -radius*c_theta 0;c_theta -radius*s_theta 0;zeros(1,3)]*theta_dot)*[radius_dot;theta_dot;height_dot];
% Jdotv2 is Jdotv(4:6)
Jdotv2 = R_cylinder2cartesian*dR_tangent2cylinder_dtheta*theta_dot*v_cylinder(4:6);
Jdotv = [Jdotv1;Jdotv2];
end
