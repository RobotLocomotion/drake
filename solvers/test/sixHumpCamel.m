function sixHumpCamel

% example 1 from gloptipoly documentation
% the function has six local minima

x = msspoly('x',2);
f = x(1)^2*(4-2.1*x(1)^2 + x(1)^4/3) + x(1)*x(2) + x(2)^2*(-4+4*x(2)^2);

% plot the surface
[X1,X2] = meshgrid(-2:.1:2,-1:.1:1);
fval = reshape(msubs(f,x,[X1(:)';X2(:)']),size(X1));
mesh(X1,X2,fval);
xlabel('x1'); ylabel('x2');


[xstar,fval] = polynomialOptimization(x,f)