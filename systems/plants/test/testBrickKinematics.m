function testBrickKinematics
warning('off','RigidBodyManipulator:collisionDetect:doKinematicsMex');
warning('off','Drake:TaylorVar:DoubleConversion');
for fb = {'rpy','RPY'};%,'quat'};
  options.floating = fb{1};
  m = RigidBodyManipulator('FallingBrick.urdf',options);
  
  nq=getNumDOF(m);
  for i=1:25
    q = randn(nq,1); qd = randn(nq,1);
    options.grad_method = {'user','taylorvar'};
     [x,J] = geval(1,@contactPositions,m,q,options);
    
    options.grad_method = 'taylorvar';
    [x,J,dJ] = geval(1,@contactPositions,m,q,options);
    kinsol = doKinematics(m,q,false,false,qd);
    [~,~,Jdot] = contactPositionsJdot(m,kinsol);
    valuecheck(Jdot,matGradMult(reshape(dJ,6*getNumContactPairs(m)*nq,nq),qd));
  end
end


end