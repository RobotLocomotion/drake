function testBrickKinematics

options.floating = true;
m = RigidBodyManipulator('FallingBrick.urdf',options);

nq=getNumDOF(m);
for i=1:25
  q = randn(nq,1); qd = randn(nq,1);
  options.grad_method = {'user','taylorvar'};
%  [x,J] = geval(1,@contactPositions,m,q,options);

  options.grad_method = 'taylorvar';
  [x,J,dJ] = geval(1,@contactPositions,m,q,options);
  kinsol = doKinematics(m,q,false,false,qd);
  [~,~,Jdot] = contactPositionsJdot(m,kinsol);
  valuecheck(Jdot,matGradMult(reshape(dJ,3*getNumContacts(m)*nq,nq),qd));
end



end