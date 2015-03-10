function testCartesian2CylindricalTransform
import drakeFunction.*
import drakeFunction.frame.*
import drakeFunction.geometry.*

% First test the identity transform, the cylinder is coaxial with the world
% z axis
T1 = eye(4);
fcn = Cartesian2CylindricalTransform(T1);
pos = [1;1;1;0;0;0];
[d,dd] = fcn.eval(pos);
[~,df] = geval(@(x) fcn.eval(x),pos,struct('grad_method','numerical'));
valuecheck(df,dd,1e-3);
valuecheck(d(1:3),[sqrt(2);pi/4;1]);
valuecheck(d(4:6),[0;0;-pi/4]);

% Now test random transform for the cylinder
T2 = [rpy2rotmat(randn(3,1)) randn(3,1);0 0 0 1];
fcn = Cartesian2CylindricalTransform(T2);
pos = randn(6,1);
[d,dd] = fcn.eval(pos);
[~,df] = geval(@(x) fcn.eval(x),pos,struct('grad_method','numerical'));
valuecheck(dd,df,1e-3);
end