function testLQRmex()
  r = RigidBodyManipulator([getDrakePath, '/examples/Acrobot/Acrobot.urdf']);
  [A,B] = r.linearize(0, [pi;0;0;0], 0);
  Q = diag([10,10,1,1]);
  R = 0.1;
  
  [K, S] = lqr(A,B,Q,R);
  [K_mex, S_mex] = lqrmex(A,B,Q,R);

  valuecheck(K, K_mex, 1e-5);
  valuecheck(S, S_mex, 1e-5);
  
end

