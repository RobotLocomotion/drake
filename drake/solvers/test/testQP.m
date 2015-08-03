function testQP

w = warning('off','optim:quadprog:SwitchToMedScale');

fprintf('****************************************\n');
fprintf(' min_x (.5*x''*A*x + x)\n');
fprintf('****************************************\n');
a = randn(2); 
prog = QuadraticProgram(a'*a+eye(2),randn(2,1));
testSolvers(prog)

fprintf('****************************************\n');
fprintf(' min_x (.5*x^2 + x), subj to -10<=x<=10\n');
fprintf('****************************************\n');
prog = QuadraticProgram(1,1,1,50,[],[],-10,10);
testSolvers(prog);

fprintf('****************************************\n');
fprintf(' min_x (.5*x^2 + x), subj to 5<=x<=10\n');
fprintf('****************************************\n');
prog = QuadraticProgram(1,1,[],[],[],[],5,10);
testSolvers(prog);


fprintf('****************************************\n');
fprintf(' min_x (.5*x(1)^2+x(2)^2+x(1)*x(2) + x(1)+2*x(2)), subj to 5<=x(1)<=10;-20<=x(1)+4*x(2)<10\n');
fprintf('****************************************\n');
prog = QuadraticProgram(eye(2),[1;0],[],[],[],[],[],[]);
prog = prog.addCost(QuadraticConstraint(-inf,inf,[0 1;1 0],zeros(2,1)));
prog = prog.addBoundingBoxConstraint(BoundingBoxConstraint(5,10),1);
prog = prog.addLinearConstraint(LinearConstraint(-20,10,[1 4]));
solvers = {};
if(checkDependency('quadprog'))
  solvers = [solvers,{'quadprog'}];
end
if(checkDependency('gurobi'))
  solvers = [solvers,{'gurobi'}];
end
if(checkDependency('gurobi_mex'))
  solvers = [solvers,{'gurobi_mex'}];
end
if(checkDependency('snopt'))
  solvers = [solvers,{'snopt'}];
end
testSolvers(prog,solvers,1e-3);
warning(w);

end



function testSolvers(prog,solvers,tol)
if(nargin<2)
  solvers = {};
  if(checkDependency('quadprog'))
    solvers = [solvers,{'quadprog'}];
  end
  if(checkDependency('gurobi'))
    solvers = [solvers,{'gurobi'}];
  end
  if(checkDependency('gurobi_mex'))
    solvers = [solvers,{'gurobi_mex'}];
  end
  if(checkDependency('fastqp'))
    solvers = [solvers,{'fastqp'}];
  end
  if(checkDependency('snopt'))
    solvers = [solvers,{'snopt'}];
  end
end
[x,objval,exitflag,execution_time]=compareSolvers(prog,randn(prog.num_vars,1),solvers);
if(nargin<3)
  tol = 1e-4;
end
  fprintf('\n\n');
  
  for i=2:length(x)
    if isempty(x{i}), continue; end
    valuecheck(x{1},x{i},tol);
    valuecheck(objval{1},objval{i},tol);
  end
end
