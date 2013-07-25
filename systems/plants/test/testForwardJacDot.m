function testForwardJacDot

p = RigidBodyManipulator('../../../examples/FurutaPendulum/FurutaPendulum.urdf');
nq = p.getNumStates()/2;

options.grad_method = {'user','taylorvar'};

for i=1:100
  q = randn(nq,1);
  qd = randn(nq,1);
  body_ind = randi(3);
  pt = randn(3,1);

  % non-mex
  kinsol = doKinematics(p,q,true,false,qd);
  [x,J,dJ] = forwardKin(p,kinsol,body_ind,pt);
  Jdot = forwardJacDot(p,kinsol,body_ind,pt);
  valuecheck(Jdot,matGradMult(reshape(dJ,3*nq,nq),qd));
  
  [c,Jc,dJc] = getCOM(p,kinsol); %geval(@myfun,q,options);
  Jcdot = forwardJacDot(p,kinsol,0);
  valuecheck(Jcdot,matGradMult(reshape(dJc,3*nq,nq),qd));

  % mex version
  kinsol = doKinematics(p,q,true,true,qd);
  [xmex,Jmex,dJmex] = forwardKin(p,kinsol,body_ind,pt);
  Jdotmex = forwardJacDot(p,kinsol,body_ind,pt);
  valuecheck(Jdotmex,Jdot);
  valuecheck(matGradMult(reshape(dJmex,3*nq,nq),qd),Jdotmex);

  [cmex,Jcmex,dJcmex] = getCOM(p,kinsol);
  valuecheck(dJcmex,dJc);
  Jcdotmex = forwardJacDot(p,kinsol,0);
  valuecheck(Jcdotmex,matGradMult(reshape(dJcmex,3*nq,nq),qd));
end

end
