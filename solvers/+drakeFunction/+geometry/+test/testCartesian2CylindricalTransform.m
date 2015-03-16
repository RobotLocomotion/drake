function testCartesian2CylindricalTransform
import drakeFunction.*
import drakeFunction.frame.*
import drakeFunction.geometry.*

% First test the identity transform, the cylinder is coaxial with the world
% z axis
cylinder_axis = [0;0;1];
cylinder_x_dir = [1;0;0];
cylinder_origin = [0;0;0];
x_cartesian = [1;1;1;0;0;0];
[x_cylinder,dd] = testCartesian2Cylindrical(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian);
valuecheck(x_cylinder(1:3),[sqrt(2);pi/4;1]);
valuecheck(x_cylinder(4:6),[0;0;-pi/4]);
[x_cartesian2,dd2] = testCylindrical2Cartesian(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder);
valuecheck(x_cartesian,x_cartesian2,1e-5);
valuecheck(dd2*dd,eye(6),1e-3);

% Now test random transform for the cylinder
cylinder_axis = randn(3,1);
cylinder_x_dir = cross(cylinder_axis,randn(3,1));
cylinder_origin = randn(3,1);
x_cartesian = zeros(6,1);
x_cartesian(1:3) = randn(3,1);
x_cartesian(4:6) = uniformlyRandomRPY();
[x_cylinder,dd] = testCartesian2Cylindrical(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian);
[x_cartesian2,dd2] = testCylindrical2Cartesian(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder);
valuecheck(x_cartesian,x_cartesian2,1e-5);
valuecheck(dd2*dd,eye(6),1e-3);
end

function [x_cylinder,dd] = testCartesian2Cylindrical(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian)
import drakeFunction.*
import drakeFunction.frame.*
import drakeFunction.geometry.*
fcn = Cartesian2CylindricalTransform(cylinder_axis,cylinder_x_dir,cylinder_origin);
[x_cylinder,dd] = fcn.eval(x_cartesian);
[~,df] = geval(@(x) fcn.eval(x),x_cartesian,struct('grad_method','numerical'));
valuecheck(dd,df,1e-3);
[x_cylinder_mex,dd_mex] = cartesian2cylindricalmex(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cartesian);
valuecheck(x_cylinder,x_cylinder_mex);
valuecheck(dd,dd_mex);
end

function [x_cartesian,dd] = testCylindrical2Cartesian(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder)
import drakeFunction.*
import drakeFunction.frame.*
import drakeFunction.geometry.*
fcn = Cylindrical2CartesianTransform(cylinder_axis,cylinder_x_dir,cylinder_origin);
[x_cartesian,dd] = fcn.eval(x_cylinder);
[~,df] = geval(@(x) fcn.eval(x),x_cylinder,struct('grad_method','numerical'));
valuecheck(dd,df,1e-3);
[x_cartesian_mex,dd_mex] = cylindrical2cartesianmex(cylinder_axis,cylinder_x_dir,cylinder_origin,x_cylinder);
valuecheck(x_cartesian,x_cartesian_mex);
valuecheck(dd,dd_mex);
end