function testLQRmex()
  r = RigidBodyManipulator([getDrakePath, '/examples/Acrobot/Acrobot.urdf']);
  [A,B] = r.linearize(0, [pi;0;0;0], 0);
  Q = diag([10,10,1,1]);
  R = 0.5;
  
  compareValues(A,B,Q,R);
  
  
  tmp = addpathTemporary([getDrakePath, '/examples/Quadrotor']);
  r = QuadPlantPenn();
  [A,B] = linearize(r,0,[0;0;1;zeros(9,1)],double(nominalThrust(r)));
  Q = diag([10*ones(6,1); ones(6,1)]);
  R = .1*eye(4);
  
  compareValues(A,B,Q,R);
  
end

function compareValues(A,B,Q,R)
  A = full(A);
  B = full(B);
  Q = full(Q);
  R = full(R);
  [K, S] = lqr(A,B,Q,R);
  [K_mex, S_mex] = lqrmex(A,B,Q,R);

  valuecheck(K_mex, K, 1e-6);
  valuecheck(S_mex, S, 1e-6);
end

