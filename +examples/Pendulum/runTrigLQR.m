function runTrigLQR()

p = PendulumPlant;

Q = diag([10 1]); R=1;
[c,V] = tilqr(p,p.xG,p.uG,Q,R);

options.replace_output_w_new_state = true;
tp = extractTrigPolySystem(p,options);

%% quick sanity test on the linearization
[A,B] = linearize(p,0,double(p.xG),double(p.uG));
[Ap,Bp] = linearize(tp,0,double(p.xG.inFrame(tp.getStateFrame)),double(p.uG));
valuecheck(A(2,1),-Ap(3,1));  % the linearized dynamics should have sin(q) approx -q
valuecheck(A(2,2),Ap(3,3));  

%%

Q = diag([10 10 1]); R = 1;
[cp,Vp] = tilqr(tp,p.xG,p.uG,Q,R);

%%  verify that the state constrainted lqr derives the -Kx => -Ksin(x) controller

fprintf(1,'     lqr: -Kx = %f*q + %f*qdot\n',c.D(1),c.D(2));
fprintf(1,'trig lqr: -Kx = %f*sin(q) + %f*cos(q) + %f*qdot\n',cp.D(1),cp.D(2),cp.D(3));

K=c.D;Kp=cp.D;
valuecheck(K(1),-Kp(1));
valuecheck(K(2),Kp(3));
valuecheck(Kp(2),0);


%S=V.S; Sp=Vp.S;

% plot the one-level sets
if (0)
  figure(1); clf; hold on;
  options.x0 = double(p.xG);
  plotFunnel(V.inFrame(p.getStateFrame),options);
  plotFunnel(Vp.inFrame(p.getStateFrame),options);  % need a much faster way to publish this
end
