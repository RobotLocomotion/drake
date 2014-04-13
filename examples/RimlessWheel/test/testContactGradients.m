function testContactGradients

options.floating = true;
options.twoD = true;
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('../RimlessWheel.urdf',.01,options);
warning(w);
x0 = p.resolveConstraints([0;1+rand;randn;5*rand;randn;5*rand]);

options.grad_method = {'user','taylorvar'};
[n,D,dn,dD] = geval(2,@contactConstraintsWrapper,p,x0(1:p.getNumDOF()),options);

for i=1:100
  q = randn(p.getNumDOF,1); 
  [n,D,dn,dD] = geval(2,@contactConstraintsWrapper,p,q,options);
end



function [n,D,dn,dD] = contactConstraintsWrapper(manip,q)
  if (nargout>2)
    [phi,~,~,~,~,~,~,mu,n,D,dn,dD] = contactConstraints(manip,q);
    dD = cellfun(@(A)reshape(full(A),size(D{1},1),size(D{1},2),size(dD{1},2)),dD,'UniformOutput',false);
    dD = reshape(vertcat(dD{:}),length(dD)*size(dn,1),size(dn,2));
  else
    [phi,~,~,~,~,~,~,mu,n,D] = contactConstraints(manip,q);
  end
  D = vertcat(D{:});
end

end
