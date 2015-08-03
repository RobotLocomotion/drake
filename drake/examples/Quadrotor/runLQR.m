function runLQR

r = Quadrotor();
v = r.constructVisualizer();

x0 = [0;0;1;zeros(9,1)];
u0 = double(nominalThrust(r));

%c = tilqr(r,x0,u0,diag([10*ones(5,1);0;ones(5,1);0]),eye(4));

% the linearized system 
[A,B] = linearize(r,0,x0,double(u0));

Q = diag([10*ones(6,1); ones(6,1)]);
R = .1*eye(4);
K = lqr(full(A),full(B),Q,R);

% u = u0 - K*(x-x0)
c = AffineSystem([],[],[],[],[],[],[],-K,u0 + K*x0);
c = setInputFrame(c,getStateFrame(r));
c = setOutputFrame(c,getInputFrame(r));

sys = feedback(r,c);

for i=1:5
  xtraj = simulate(sys,[0 4],double(x0)+[.5*randn(6,1);zeros(6,1)]);
  v.playback(xtraj);
end