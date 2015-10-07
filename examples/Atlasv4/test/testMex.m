function testMex

options.floating=true;
p = PlanarRigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);
nq = getNumPositions(p);

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

  valuecheck(x,xm,1e-8);
  valuecheck(J,Jm,1e-8);
  valuecheck(dJ,dJm,1e-8);

end

options.floating=true;
p = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);
nq = getNumPositions(p);

for i=1:25
  q = randn(nq,1);
  qd = randn(nq,1);
  
  [Hm,Cm,Bm] = p.manipulatorDynamics(q,qd,true);
  [H,C,B] = p.manipulatorDynamics(q,qd,false);

  valuecheck(Hm,H,1e-8);
  valuecheck(Cm,C,1e-8);
  valuecheck(Bm,B,1e-8);

  [Hm,Cm,Bm,dHm,dCm,dBm] = p.manipulatorDynamics(q,qd,true);
  [H,C,B,dH,dC,dB] = p.manipulatorDynamics(q,qd,false);

  valuecheck(Hm,H,1e-8);
  valuecheck(Cm,C,1e-8);
  valuecheck(Bm,B,1e-8);
  
  valuecheck(dCm,dC,1e-8);
  valuecheck(dHm,dH,1e-8);
  valuecheck(dBm,dB,1e-8);

  % test mex kinematics
  rq = rand(nq,1);
  kinsol=p.doKinematics(rq,true,true);
  rb = randi(3);
  rp = rand(3,1);
  [xm,Jm,dJm] = forwardKin(p,kinsol,rb,rp);
  
  kinsol=p.doKinematics(rq,true,false);
  [x,J,dJ] = forwardKin(p,kinsol,rb,rp);

  valuecheck(xm,x,1e-8);
  valuecheck(Jm,J,1e-8);
  valuecheck(dJm,dJ,1e-8);

  % test mex COM
  kinsol = p.doKinematics(rq,true,false);
  [com,Jcom,dJcom]=getCOM(p,kinsol);
  kinsol = p.doKinematics(rq,true,true);
  [commex,Jcommex,dJcommex]=getCOM(p,kinsol);
  
  valuecheck(commex,com,1e-8);
  valuecheck(Jcommex,Jcom,1e-8);
  valuecheck(dJcommex,dJcom,1e-8);
end

