function testDesiredQuatDiff
import drakeFunction.*
import drakeFunction.geometry.*

quat_des = randn(4,1);
quat_des = quat_des/norm(quat_des);
des_quat_diff_fcn = DesiredQuatDiff(quat_des);
quat = randn(4,1);
quat = quat./norm(quat);
[d,dd] = des_quat_diff_fcn.eval(quat);
[f,df] = geval(@(quat) eval(des_quat_diff_fcn,quat),quat,struct('grad_method',{{'user','taylorvar'}},'tol',1e-6));

d = des_quat_diff_fcn.eval(quat_des);
valuecheck(d,1,1e-5);

d = des_quat_diff_fcn.eval([1;0;0;0]);
axis_des = quat2axis(quat_des);
valuecheck(cos(axis_des(4)),d,1e-5);
end
