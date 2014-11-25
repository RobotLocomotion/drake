function sixHumpCamel

% example 1 from gloptipoly documentation
% the function has six local minima

checkDependency('spotless');

x = msspoly('x',2);
f = x(1)^2*(4-2.1*x(1)^2 + x(1)^4/3) + x(1)*x(2) + x(2)^2*(-4+4*x(2)^2);

% plot the surface
[X1,X2] = meshgrid(-2:.1:2,-1:.1:1);
fval = reshape(msubs(f,x,[X1(:)';X2(:)']),size(X1));
mesh(X1,X2,fval);
xlabel('x1'); ylabel('x2');

prog = PolynomialProgram(x,f);
[xstar,fval] = solve(prog,randn(2,1))

hold on;
plot3(xstar(1),xstar(2),fval,'r*','MarkerSize',10,'LineWidth',3);
hold off;

solvers = {};
if(checkDependency('gloptipoly3'))
  solvers = [solvers,{'gloptipoly'}];
end
if(checkDependency('bertini'))
  solvers = [solvers,{'bertini'}];
end
if(checkDependency('snopt'))
  solvers = [solvers,{'snopt'}];
end
if(checkDependency('fmincon'))
  solvers = [solvers,{'fmincon'}];
end
compareSolvers(prog,randn(2,1),solvers)