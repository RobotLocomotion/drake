function testRotateVectorToAlign()
display('Check when a,b are colinear and in the same direction');
a = [0;0;10];
b = [0;0;0.1];
R = testRotationMatrix(a,b);

display('Check when a,b are not colinear and are in the opposite direction');
a = [0;0;10];
b = [0;0;-0.1];
R = testRotationMatrix(a,b);

display('Check random vectors that are colinear');
a = randn(3,1);
b = a;
R = testRotationMatrix(a,b);

b = -a;
R = testRotationMatrix(a,b);

display('Check random vectors');
a = randn(3,1);
b = randn(3,1);
R = testRotationMatrix(a,b);
end

function flag = isRotationMatrix(R)
if(norm(reshape(R'*R-eye(3),[],1))>1e-4 || abs(det(R)-1)>1e-4)
  flag = false;
else
  flag = true;
end
end

function R = testRotationMatrix(a,b)
R = rotateVectorToAlign(a,b);
a1 = a/norm(a);
b1 = b/norm(b);
if(norm(R*a1-b1)>1e-4)
  error('Rotation matrix is incorrect');
end
if(~isRotationMatrix(R))
  error('R is not a rotation matrix');
end
end