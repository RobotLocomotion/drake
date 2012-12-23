function testMex

p_mex = PlanarRigidBodyManipulator('../../../examples/Acrobot/Acrobot.urdf');
p_mat = PlanarRigidBodyManipulator('../../../examples/Acrobot/Acrobot.urdf');
p_mat.model = p_mat.model.deleteMexPointer();

nq = p_mex.getNumStates()/2;

for i=1:100
  q = randn(2,1);
  qd = randn(2,1);

  [Hm,Cm,Bm] = p_mex.manipulatorDynamics(q,qd);
  [H,C,B] = p_mat.manipulatorDynamics(q,qd);

  valuecheck(H,Hm,1e-8);
  valuecheck(C,Cm,1e-8);
  valuecheck(B,Bm,1e-8);
  
  [Hm,Cm,Bm,dHm,dCm,dBm] = p_mex.manipulatorDynamics(q,qd);
  [H,C,B,dH,dC,dB] = p_mat.manipulatorDynamics(q,qd);

  valuecheck(H,Hm,1e-8);
  valuecheck(C,Cm,1e-8);
  valuecheck(B,Bm,1e-8);
  
  valuecheck(dH,dHm,1e-8);
  valuecheck(dC,dCm,1e-8);
  valuecheck(dB,dBm,1e-8);

  % test mex kinematics
  rq = rand(nq,1);
  p_mex.model.doKinematics(rq,true);
  rb = randi(3);
  rp = rand(2,1);
  [xm,Jm,dJm] = forwardKin(p_mex.model,rb,rp);
  
  p_mat.model.doKinematics(rq,true);
  [x,J,dJ] = forwardKin(p_mat.model,rb,rp);

  valuecheck(x,xm,1e-8);
  valuecheck(J,Jm,1e-8);
  valuecheck(dJ,dJm,1e-8);

end

p_mex = RigidBodyManipulator('../../../examples/FurutaPendulum/FurutaPendulum.urdf');
p_mat = RigidBodyManipulator('../../../examples/FurutaPendulum/FurutaPendulum.urdf');
p_mat.model = p_mat.model.deleteMexPointer();

nq = p_mex.getNumStates()/2;

for i=1:100
  q = randn(nq,1);
  qd = randn(nq,1);
  
  [Hm,Cm,Bm] = p_mex.manipulatorDynamics(q,qd);
  [H,C,B] = p_mat.manipulatorDynamics(q,qd);

  valuecheck(H,Hm,1e-8);
  valuecheck(C,Cm,1e-8);
  valuecheck(B,Bm,1e-8);

  [Hm,Cm,Bm,dHm,dCm,dBm] = p_mex.manipulatorDynamics(q,qd);
  [H,C,B,dH,dC,dB] = p_mat.manipulatorDynamics(q,qd);

  valuecheck(H,Hm,1e-8);
  valuecheck(C,Cm,1e-8);
  valuecheck(B,Bm,1e-8);
  
  valuecheck(dH,dHm,1e-8);
  valuecheck(dC,dCm,1e-8);
  valuecheck(dB,dBm,1e-8);

  % test mex kinematics
  rq = rand(nq,1);
  p_mex.model.doKinematics(rq,true);
  rb = randi(3);
  rp = rand(3,1);
  [xm,Jm,dJm] = forwardKin(p_mex.model,rb,rp);
  
  p_mat.model.doKinematics(rq,true);
  [x,J,dJ] = forwardKin(p_mat.model,rb,rp);

  valuecheck(x,xm,1e-8);
  valuecheck(J,Jm,1e-8);
  valuecheck(dJ,dJm,1e-8);


end

