function testRPYdot2Angularvel()
% test both rpydot2angularvel and rpydot2angularvelMatrix
options.grad_method = {'numerical','user'};
options.tol = 1e-4;
for i = 1:100
  rpy = uniformlyRandomRPY();
  rpydot = randn(3,1);
  [~,~] = geval(@rpydot2angularvelMatrix,rpy,options);
  [~,~] = geval(2,@rpydot2angularvel,rpy,rpydot,options);
end
end