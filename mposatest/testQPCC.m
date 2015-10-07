megaclear
if 0
  Q{1} = zeros(4);
  f_Q{1} = zeros(4,1);
  R{1} = 1;
  f_R{1} = 0;
  H{1} = eye(2);
  
  g = 9.81;
  umax = 20;
  C{1} = [0;g - umax];
  B{1} = [0; 1];
  J{1} = [0 1];
  Jf{1} = [1 0];
  phi0{1} = 0;
  Dphi{1} = [0 1];
  psi0{1} = 0;
  Dpsi{1} = [0 0 1 0];
  mu = 1;
  u_lb = -2*umax;
  u_ub = 0;
  q0 = [0;0];
  qd0 = [0;0];
  dt = .1;
  
  [x_opt, f_opt] = solveQPCC(Q,R,f_Q,f_R,H,C,B,J,Jf,phi0,Dphi,psi0,Dpsi,mu,u_lb,u_ub,q0,qd0,dt)
  
else
  Q{1} = diag([0;1;0;0]);
  f_Q{1} = [0;-20;0;0];
  R{1} = .001;
  f_R{1} = 0;
  H{1} = eye(2);
  
  g = 9.81;
  umax = 20;
  C{1} = [0;g];
  B{1} = [0; 1];
  J{1} = [0 1];
  Jf{1} = [1 0];
  phi0{1} = 0;
  Dphi{1} = [0 1];
  psi0{1} = 0;
  Dpsi{1} = [0 0 1 0];
  mu = 1;
  u_lb = -umax;
  u_ub = umax;
  q0 = [0;0];
  qd0 = [0;0];
  dt = .1;
  
  [x_opt, f_opt] = solveQPCC(Q,R,f_Q,f_R,H,C,B,J,Jf,phi0,Dphi,psi0,Dpsi,mu,u_lb,u_ub,q0,qd0,dt)
end