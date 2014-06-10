function runLQR

r = Quadrotor();
v = r.constructVisualizer();

x0 = [0;0;1;zeros(9,1)];
u0 = double(nominalThrust(r));

%c = tilqr(r,x0,u0,diag([10*ones(5,1);0;ones(5,1);0]),eye(4));

% the linearized system is not controllable in yaw, so ignore that state
% when generating the controller
[A,B] = linearize(r,0,x0,double(u0));
A = A([1:5,7:11],[1:5,7:11]);  B = B([1:5,7:11],:); 

Q = diag([10*ones(5,1); ones(5,1)]);
R = .1*eye(4);
K = lqr(full(A),full(B),Q,R);
K = [K(:,1:5),zeros(4,1),K(:,6:10),zeros(4,1)];

% u = u0 - K*(x-x0)
c = AffineSystem([],[],[],[],[],[],[],-K,u0 + K*x0);
c = setInputFrame(c,getStateFrame(r));
c = setOutputFrame(c,getInputFrame(r));

sys = feedback(r,c);

for i=1:5
  xtraj = simulate(sys,[0 4],double(x0)+.5*randn(12,1));
  v.playback(xtraj);
end