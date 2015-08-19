function testOrientation
% test rpy to rotmat
for i = 1:1000
    rpy = 2*pi*(rand(3,1)-0.5);
    rpy1 = rotmat2rpy(rpy2rotmat(rpy));
    mat = rpy2rotmat(rpy);
    mat1 = rpy2rotmat(rpy1);
    valuecheck(mat,mat1,1e-6);
    mat2 = rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1));  % used in RigidBodyManipulator/addJoint
    valuecheck(mat,mat2,1e-6);
end
disp('rpy to rotmat and rotmat to rpy is consistent');
% test quat to rotmat
for i = 1:1000
    quat = randn(4,1);
    quat = quat./norm(quat);
    mat = quat2rotmat(quat);
    quat1 = rotmat2quat(mat);
    mat1 = quat2rotmat(quat1);
    valuecheck(mat,mat1,1e-6);
end
disp('quat to rotmat and rotmat to quat is consistent');

% Check the corner case of rotmat2quat
quat = [1;0;0;0];
mat = quat2rotmat(quat);
quat1 = rotmat2quat(mat);
valuecheck((quat'*quat1)^2,1);
quat = [0;1;0;0];
mat = quat2rotmat(quat);
quat1 = rotmat2quat(mat);
valuecheck((quat'*quat1)^2,1);
quat = [0;0;1;0];
mat = quat2rotmat(quat);
quat1 = rotmat2quat(mat);
valuecheck((quat'*quat1)^2,1);
quat = [0;0;0;1];
mat = quat2rotmat(quat);
quat1 = rotmat2quat(mat);
valuecheck((quat'*quat1)^2,1);
quat = [0;randn(3,1)];
quat = quat./norm(quat);
mat = quat2rotmat(quat);
quat1 = rotmat2quat(mat);
valuecheck((quat'*quat1)^2,1);
disp('quat to rotmat and rotmat to quat is consistent for the corner case');
for i = 1:1000
    rpy = 2*pi*(rand(3,1)-0.5);
    quat = rpy2quat(rpy);
    quat1 = rotmat2quat(rpy2rotmat(rpy));
    valuecheck(1-abs(quat1'*quat),0,1e-6);
    rpy1 = quat2rpy(quat);
    mat = rpy2rotmat(rpy);
    mat1 = rpy2rotmat(rpy1);
    valuecheck(mat,mat1,1e-6);
end
disp('rpy to quat and quat to rpy is consistent, rpy to quat is correct');

for i = 1:1000
    quat = randn(4,1);
    quat = quat./norm(quat);
    axis = quat2axis(quat);
    quat1 = axis2quat(axis);
    valuecheck(acos(abs(quat'*quat1)),0,1e-6);
end
disp('axis to quat and quat to axis is consistent');

for i = 1:1000
    quat = randn(4,1);
    quat = quat./norm(quat);
    mat = quat2rotmat(quat);
    axis = rotmat2axis(mat);
    quat1 = axis2quat(axis);
    valuecheck(acos(abs(quat'*quat1)),0,1e-6);
end
disp('rotmat2axis is correct');

for i = 1:1000
    axis = randn(4,1);
    axis(1:3) = axis(1:3)/norm(axis(1:3));
    R1 = quat2rotmat(axis2quat(axis));
    R2 = axis2rotmat(axis);
    valuecheck(R1,R2,1e-6);
end
disp('axis to rotmat is correct');

