function [x_cylinder,v_cylinder,J, Jdotv] = cartesian2cylindrical(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian,v_cartesian)
% Given the axis, the x direction and the origin of the cylinder, compute
% the point x_cartesian in the cylindrical coordinate. The cylindrical
% coordinate is illustrated in drake/doc/doc_cartesian2cylindrical.pdf
% @param cylinder_axis   A 3 x 1 vector. The axis of the cylinder in the
% cartesian frame
% @param cylinder_x_dir  A 3 x 1 vector. the cylinder x direction, the
% direction that theta = 0 is measured.
% @param cylinder_origin A 3 x 1 vector. The origin (height = 0) of the
% cylinder
% @param x_cartesian     A 6 x 1 vector. The [x;y;z;roll;pitch;yaw] in the
% cartesian frame
% @param v_cartesian     A 6 x 1 vector. The [xdot;ydot;zdot;angular_vel_x;angular_vel_y;angular_vel_z] in the
% cartesian frame. The angular velocity is measured in the Cartesian frame
% @retval x_cylinder     A 6 x 1 vector. [radius;theta;height;roll;pitch;yaw] in the
% cylindrical coordinate
% @retval v_cylinder     A 6 x 1 vector. [radius_dot;theta_dot;height_dot;angular_vel_x;angular_vel_y;angular_vel_z]
% in the cylindrical coordinate. The angular velocity is measured in the
% tangential frame
% @retval J      A 6 x 6 matrix, the geometric Jacobian, v_cartesian =
% J*v_cylinder
% @retval Jdotv  A 3 x 1 vector, the Jacobian_dot *v_cartesian
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
x_pos_cylinder = R_cylinder2cartesian\(x_cartesian(1:3)-cylinder_origin);
v_pos_cylinder = R_cylinder2cartesian\v_cartesian(1:3);
radius = sqrt(x_pos_cylinder(1)^2+x_pos_cylinder(2)^2);
radius_dot = (x_pos_cylinder(1)*v_pos_cylinder(1)+x_pos_cylinder(2)*v_pos_cylinder(2))/radius;
% dradius_dx_pos_cylinder = [x_pos_cylinder(1:2)' 0]/radius;
theta = atan2(x_pos_cylinder(2),x_pos_cylinder(1));
dtheta_dx_pos_cylinder = [datan2(x_pos_cylinder(1),x_pos_cylinder(2)) 0];
theta_dot = dtheta_dx_pos_cylinder*v_pos_cylinder;
height = x_pos_cylinder(3);
% dheight_dx_pos_cylinder = [0 0 1];
height_dot = v_pos_cylinder(3);
x_cylinder = zeros(6,1);
x_cylinder(1:3) = [radius;theta;height];
[R_tangent2cylinder,dR_tangent2cylinder_dtheta] = rotz(theta-pi/2);
x_cylinder(4:6) = rotmat2rpy(R_tangent2cylinder'*R_cylinder2cartesian'*rpy2rotmat(x_cartesian(4:6)));
J = zeros(6,6);
Jdot = zeros(6,6);
J(1:3,1:3) = [x_pos_cylinder(1)/radius x_pos_cylinder(2)/radius 0;...
             -x_pos_cylinder(2)/radius^2 x_pos_cylinder(1)/radius^2 0;...
             0 0 1]*R_cylinder2cartesian';
Jdot(1:3,1:3) = ([x_pos_cylinder(2)^2/radius^3 -x_pos_cylinder(1)*x_pos_cylinder(2)/radius^3  0;...
                 2*x_pos_cylinder(1)*x_pos_cylinder(2)/radius^4 (x_pos_cylinder(2)^2-x_pos_cylinder(1)^2)/radius^4 0;...
                 0 0 0]*R_cylinder2cartesian'*v_pos_cylinder(1)+...
                [-x_pos_cylinder(1)*x_pos_cylinder(2)/radius^3 x_pos_cylinder(1)^2/radius^3 0;...
                 (x_pos_cylinder(2)^2-x_pos_cylinder(1)^2)/radius^4 -2*x_pos_cylinder(1)*x_pos_cylinder(2)/radius^4 0;...
                 0 0 0]*R_cylinder2cartesian'* v_pos_cylinder(2));
% J(1:3,1:3) = [dradius_dx_pos_cylinder;dtheta_dx_pos_cylinder;dheight_dx_pos_cylinder]*R_cylinder2cartesian';
v_cylinder = zeros(6,1);
v_cylinder(1:3) = [radius_dot;theta_dot;height_dot];

v_cylinder(4:6) = R_tangent2cylinder'*(R_cylinder2cartesian'*v_cartesian(4:6)-[0;0;theta_dot]);
J(4:6,1:3) = R_tangent2cylinder(3,:)'*-J(2,1:3);
J(4:6,4:6) = R_tangent2cylinder'*R_cylinder2cartesian';
Jdot(4:6,1:3) = dR_tangent2cylinder_dtheta(3,:)'*-J(2,1:3)*theta_dot+R_tangent2cylinder(3,:)'*-Jdot(2,1:3);
Jdot(4:6,4:6) = dR_tangent2cylinder_dtheta'*theta_dot*R_cylinder2cartesian';
Jdotv = Jdot*v_cartesian;
end
