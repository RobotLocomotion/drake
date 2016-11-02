function testBrickKinematics
warning('off','RigidBodyManipulator:collisionDetect:doKinematicsMex');
warning('off','Drake:TaylorVar:DoubleConversion');
for fb = {'rpy','RPY'};%,'quat'};
  options.floating = fb{1};
  m = RigidBodyManipulator('FallingBrick.urdf',options);
  
  nq=getNumPositions(m);
  for i=1:25
    q = getRandomConfiguration(m); qd = randn(getNumVelocities(m),1);
    options.grad_method = {'user','taylorvar'};
    [x,J] = geval(1,@terrainContactPositions,m,q,options);
    
    options.grad_method = 'taylorvar';
    [x, J] = geval(1,@terrainContactPositions,m,q,options);
    [~, dJ] = geval(1, @(q) terrainContactJacobian(m, q), q, options);
    kinsol = doKinematics(m,q,false,false,qd);
    Jdot_times_v = terrainContactJacobianDotTimesV(m,kinsol);
    valuecheck(Jdot_times_v,matGradMult(reshape(dJ,numel(x)*nq,nq),qd) * qd);
  end
end

end

function J = terrainContactJacobian(m, q)
[~, J] = terrainContactPositions(m, q);
end
