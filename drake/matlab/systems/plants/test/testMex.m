function testMex

p = PlanarRigidBodyManipulator('../../../../examples/Acrobot/Acrobot.urdf');

nq = p.getNumStates()/2;

for i=1:100
  q = randn(2,1);
  qd = randn(2,1);

  [Hm,Cm,Bm] = p.manipulatorDynamics(q,qd,true);
  [H,C,B] = p.manipulatorDynamics(q,qd,false);

  valuecheck(H,Hm,1e-8);
  valuecheck(C,Cm,1e-8);
  valuecheck(B,Bm,1e-8);
  
  [Hm,Cm,Bm,dHm,dCm,dBm] = p.manipulatorDynamics(q,qd,true);
  [H,C,B,dH,dC,dB] = p.manipulatorDynamics(q,qd,false);

  valuecheck(H,Hm,1e-8);
  valuecheck(C,Cm,1e-8);
  valuecheck(B,Bm,1e-8);
  
  valuecheck(dH,dHm,1e-8);
  valuecheck(dC,dCm,1e-8);
  valuecheck(dB,dBm,1e-8);

  % test mex kinematics
  rq = rand(nq,1);
  kinsol = p.doKinematics(rq,true,true);
  rb = randi(3);
  rp = rand(2,1);
  [xm,Jm,dJm] = forwardKin(p,kinsol,rb,rp);
  
  kinsol = p.doKinematics(rq,true,false);
  [x,J,dJ] = forwardKin(p,kinsol,rb,rp);

  valuecheck(xm,x,1e-8);
  valuecheck(Jm,J,1e-8);
  valuecheck(dJm,dJ,1e-8);
end

p = RigidBodyManipulator('../../../../examples/FurutaPendulum/FurutaPendulum.urdf');

nq = p.getNumStates()/2;

for i=1:100
  q = randn(nq,1);
  qd = randn(nq,1);
  
  [Hm,Cm,Bm] = p.manipulatorDynamics(q,qd,true);
  [H,C,B] = p.manipulatorDynamics(q,qd,false);

  valuecheck(H,Hm,1e-8);
  valuecheck(C,Cm,1e-8);
  valuecheck(B,Bm,1e-8);

  [Hm,Cm,Bm,dHm,dCm,dBm] = p.manipulatorDynamics(q,qd,true);
  [H,C,B,dH,dC,dB] = p.manipulatorDynamics(q,qd,false);

  valuecheck(C,Cm,1e-8);
  valuecheck(H,Hm,1e-8);
  valuecheck(B,Bm,1e-8);
  
  valuecheck(dC,dCm,1e-8);
  valuecheck(dH,dHm,1e-8);
  valuecheck(dB,dBm,1e-8);

  % test mex kinematics
  rq = rand(nq,1);
  kinsol=p.doKinematics(rq,true,true);
  rb = randi(3);
  rp = rand(3,1);
  [xm,Jm,dJm] = forwardKin(p,kinsol,rb,rp);
  [commex,Jcommex,dJcommex]=getCOM(p,kinsol);
  xbm = bodyKin(p,kinsol,rb,rp);
  
  kinsol=p.doKinematics(rq,true,false);
  [x,J,dJ] = forwardKin(p,kinsol,rb,rp);
  [com,Jcom,dJcom]=getCOM(p,kinsol);
  xb = bodyKin(p,kinsol,rb,rp);

  valuecheck(x,xm,1e-8);
  valuecheck(J,Jm,1e-8);
  valuecheck(dJ,dJm,1e-8);

  % test mex COM
  valuecheck(com,commex,1e-8);
  valuecheck(Jcom,Jcommex,1e-8);
  valuecheck(dJcom,dJcommex,1e-8);
  
  valuecheck(xbm,xb);
end

w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
p = RigidBodyManipulator('../../../../examples/Pendulum/test/PendulumWithFriction.urdf');
warning(w);

nq = p.getNumStates()/2;

for i=1:100
  q = randn(nq,1);
  qd = randn(nq,1);
  
  [Hm,Cm,Bm] = p.manipulatorDynamics(q,qd,true);
  [H,C,B] = p.manipulatorDynamics(q,qd,false);

  valuecheck(H,Hm,1e-8);
  valuecheck(C,Cm,1e-8);
  valuecheck(B,Bm,1e-8);

  [Hm,Cm,Bm,dHm,dCm,dBm] = p.manipulatorDynamics(q,qd,true);
  [H,C,B,dH,dC,dB] = p.manipulatorDynamics(q,qd,false);

  valuecheck(C,Cm,1e-8);
  valuecheck(H,Hm,1e-8);
  valuecheck(B,Bm,1e-8);
  
  valuecheck(dC,dCm,1e-8);
  valuecheck(dH,dHm,1e-8);
  valuecheck(dB,dBm,1e-8);

  % test mex kinematics
  rq = rand(nq,1);
  rqd = rand(nq,1);
  kinsol=p.doKinematics(rq,true,true,rqd);
  rp = rand(3,1);
  [xm,Jm,dJm] = forwardKin(p,kinsol,1,rp);
  [commex,Jcommex,dJcommex]=getCOM(p,kinsol);
  xbm = bodyKin(p,kinsol,1,rp);

  kinsol=p.doKinematics(rq,true,false,rqd);
  [x,J,dJ] = forwardKin(p,kinsol,1,rp);
  [com,Jcom,dJcom]=getCOM(p,kinsol);
  xb = bodyKin(p,kinsol,1,rp);

  valuecheck(x,xm,1e-8);
  valuecheck(J,Jm,1e-8);
  valuecheck(dJ,dJm,1e-8);

  % test mex COM
  valuecheck(com,commex,1e-8);
  valuecheck(Jcom,Jcommex,1e-8);
  valuecheck(dJcom,dJcommex,1e-8);

  valuecheck(xbm,xb);
end

