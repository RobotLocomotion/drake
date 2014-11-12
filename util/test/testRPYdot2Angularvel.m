function testRPYdot2Angularvel()
% test both rpydot2angularvel and rpydot2angularvelMatrix
options.grad_method = {'taylorvar','user'};
for i = 1:100
  rpy = uniformlyRandomRPY();
  rpydot = randn(3,1);
  [~,~] = geval(@rpydot2angularvelMatrix,rpy,options);
  [~,~] = geval(2,@rpydot2angularvel,rpy,rpydot,options);
end
end