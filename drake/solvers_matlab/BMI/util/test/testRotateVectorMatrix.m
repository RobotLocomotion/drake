function testRotateVectorMatrix
% test to find a rotation matrix R that rotate vector a to align with b
% through rank minimization
checkDependency('spotless');
[p,quat,Quat] = rotateMatrixProg();

display('Check vectors that are in the same direction');
a = randn(3,1);
b = rand()*a;
R = testRotateMatrix_userfun(p,quat,Quat,a,b);

a = [0;0;1];
b = [0;0;10];
R = testRotateMatrix_userfun(p,quat,Quat,a,b);

display('Check vectors that are in the opposite direction');
a = randn(3,1);
b = -rand()*a;
R = testRotateMatrix_userfun(p,quat,Quat,a,b);

a = [0;0;1];
b = [0;0;-10];
R = testRotateMatrix_userfun(p,quat,Quat,a,b);

display('Check vectors in the arbitrary direction');
a = randn(3,1);
b = randn(3,1);
R = testRotateMatrix_userfun(p,quat,Quat,a,b);

end

function [p,quat,Quat] = rotateMatrixProg()
p = BMIspotless();
[p,quat] = p.newFree(4,1);
[p,Quat] = p.newSym(4);
p = p.withEqs(Quat(1,1)+Quat(2,2)+Quat(3,3)+Quat(4,4)-1);
p = p.addBilinearVariable(quat,Quat);
end

function R = testRotateMatrix_userfun(p,quat,Quat,a,b)
a1 = a/norm(a);
b1 = b/norm(b);
p1 = p.withEqs(rotatePtByQuatBilinear(Quat,a1)-b1);
[solver_sol,info] = p1.optimize();
if(info ~= 1)
  error('faild to solve the problem');
end
quat_sol = double(solver_sol.eval(quat));
R = quat2rotmat(quat_sol);
end