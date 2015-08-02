function matHessianMultMatTest
for i = 1:100
  q = randn(2,1);
  [AB,dAB,ddAB] = geval(@matTest,q,struct('grad_method','taylorvar'));
  [AB2,dAB2,ddAB2] = matTest(q);
  valuecheck(dAB,dAB2);
  valuecheck(ddAB,ddAB2);
end

for i = 1:100
  x = randn(2,1);
  [AB,dAB,ddAB] = geval(@matTest2,x,struct('grad_method','taylorvar'));
  [AB2,dAB2,ddAB2] = matTest2(x);
  valuecheck(dAB,dAB2);
  valuecheck(ddAB,ddAB2);
end
end

function [AB,dAB,ddAB] = matTest(q)
A = rotz(q(1));
B = rotz(q(2));

AB = A*B;
if(nargout>1)
  [A,dA,ddA] = rotz(q(1));
  [B,dB,ddB] = rotz(q(2));
  
  dA = [dA(:) zeros(9,1)];
  ddA = [ddA(:) zeros(9,3)];
  dB = [zeros(9,1) dB(:)];
  ddB = [zeros(9,3) ddB(:)];
  dAB = matGradMultMat(A,B,dA,dB);
  ddAB = matHessianMultMat(A,B,dA,dB,ddA,ddB);
end
end

function [AB,dAB,ddAB] = matTest2(x)
A = test_fun_A(x);
B = test_fun_B(x);
AB = A*B;
if(nargout>1)
  [A,dA,ddA] = geval(@test_fun_A,x,struct('grad_method','taylorvar'));
  [B,dB,ddB] = geval(@test_fun_B,x,struct('grad_method','taylorvar'));
  dAB = matGradMultMat(A,B,dA,dB);
  ddAB = matHessianMultMat(A,B,dA,dB,ddA,ddB);
end
end
function [A,dA,ddA] = test_fun_A(x)
A = [x(1)^3+x(2) x(1)*x(2)^2;x(1)-x(2) -x(2)^3];
end

function [B,dB,ddB] = test_fun_B(x)
B = [cos(x(1))*x(2) -sin(x(1)+x(2)); x(1)^2*sin(x(2)) cos(x(1)+2*x(2))];
end