function testCylindrical2cartesian
cylinder_axis = [0;0;1];
cylinder_x_dir = [1;0;0];
cylinder_origin = [0;0;0];
x_cartesian = [[0;1;0];zeros(3,1)];
v_cartesian = [zeros(3,1);randn(3,1)];
[x_cylinder,v_cylinder,J,Jdotv] = test_fun(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian,v_cartesian);
valuecheck(v_cylinder,[zeros(3,1);v_cartesian(4:6)]);

cylinder_axis = randn(3,1);
cylinder_x_dir = cross(cylinder_axis,randn(3,1));
cylinder_origin = randn(3,1);
x_cartesian = [randn(3,1);uniformlyRandomRPY()];
v_cartesian = randn(6,1);
test_fun(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian,v_cartesian);
end

function [x_cylinder,v_cylinder,J,Jdotv] = test_fun(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian,v_cartesian)
[x_cylinder,v_cylinder,J,Jdotv] = cartesian2cylindrical(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian,v_cartesian);
valuecheck(J*v_cartesian,v_cylinder);
[x_cylinder_mex,v_cylinder_mex,J_mex,Jdotv_mex] = cartesian2cylindricalmex(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian,v_cartesian);
valuecheck(x_cylinder_mex,x_cylinder);
valuecheck(v_cylinder_mex,v_cylinder);
valuecheck(J_mex,J);
valuecheck(Jdotv_mex,Jdotv);
[x_cartesian2,v_cartesian2,J2,Jdotv2] = cylindrical2cartesian(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder,v_cylinder);
valuecheck(J2*v_cylinder,v_cartesian2);
[x_cartesian2_mex,v_cartesian2_mex,J2_mex,Jdotv2_mex] = cylindrical2cartesianmex(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder,v_cylinder);
valuecheck(x_cartesian2_mex,x_cartesian2);
valuecheck(v_cartesian2_mex,v_cartesian2);
valuecheck(J2_mex,J2);
valuecheck(Jdotv2_mex,Jdotv2);
valuecheck(x_cartesian,x_cartesian2);
valuecheck(v_cartesian2,v_cartesian);
valuecheck(J*J2,eye(6));
end