function testCylindrical2cartesian()
cylinder_axis = [0;0;1];
cylinder_x_dir = [1;0;0];
cylinder_origin  = [0;0;0];
x_cylinder = [rand();2*pi*rand();randn();uniformlyRandomRPY];
v_cylinder = randn(6,1);
[x_cartesian,v_cartesian,J,Jdotv] = test_userfun(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder,v_cylinder);
valuecheck(x_cartesian(1)^2+x_cartesian(2)^2,x_cylinder(1)^2);
valuecheck(atan2(x_cartesian(2),x_cartesian(1)),x_cylinder(2));
valuecheck(x_cartesian(3),x_cylinder(3));
valuecheck(rotz(pi/2-x_cylinder(2))*rpy2rotmat(x_cylinder(4:6)),rpy2rotmat(x_cartesian(4:6)));
end

function [x_cartesian,v_cartesian,J,Jdotv] = test_userfun(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder,v_cylinder)
[x_cartesian,v_cartesian,J,Jdotv] = cylindrical2cartesian(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder,v_cylinder);
valuecheck(v_cartesian,J*v_cylinder);
x_rpy_cylinder = x_cylinder(4:6);
x_rpydot_cylinder = angularvel2rpydotMatrix(x_rpy_cylinder)*v_cartesian(4:6);
dt = 1e-4;
x_cartesian_dt = cylindrical2cartesian(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder+[v_cylinder(1:3);x_rpydot_cylinder]*dt,v_cylinder);
valuecheck((x_cartesian_dt(1:3)-x_cartesian(1:3))/dt,v_cartesian(1:3),1e-3);
valuecheck((x_cartesian_dt(4:6)-x_cartesian(4:6))/dt,angularvel2rpydot(x_cartesian(4:6),v_cartesian(4:6)),1e-3);

end