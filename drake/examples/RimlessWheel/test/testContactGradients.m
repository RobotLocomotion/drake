function testContactGradients

% Setting a fixed seed to avoid stochastic failures
rng(3);

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('../RimlessWheel.urdf',.01,options);
warning(w);
x0 = p.resolveConstraints([0;1+rand;randn;5*rand;randn;5*rand]);

w = warning('off','Drake:RigidBodyManipulator:collisionDetect:doKinematicsMex');
warning('off','Drake:TaylorVar:DoubleConversion');
options.grad_method = {'user','numerical'};
options.diff_type = 'central';
options.tol = 1e-4;
[n,D,dn,dD] = geval(2,@contactConstraintsWrapper,p,x0(1:p.getNumPositions()),options);

for i=1:100
  q = randn(p.getNumPositions,1); 
  [n,D,dn,dD] = geval(2,@contactConstraintsWrapper,p,q,options);
end

warning(w);



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
