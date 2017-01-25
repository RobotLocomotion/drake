function testExternalForce()

r = KukaArm();

nx = r.getNumStates;
nu = r.getNumInputs;

% test derivatives
for i=1:10
  x=randn(nx,1);
  u=randn(nu,1);
  w=randn(3,1);

  [f,df] = r.dynamics_w(0,x,u,w);
  [f2,df2] = geval(@r.dynamics_w,0,x,u,w,struct('grad_method','numerical'));
  
  valuecheck(df2,df,1e-2); %SK: not sure why the tolerance is so bad---true for vanilla dynamics too
end



% test visually
w = [110;0;0];

h = 0.001;
v=r.constructVisualizer;

T = 2.0;
N = ceil(T/h);
xn = zeros(nx,1);
for i=1:N
  
  xn = xn + h*r.dynamics_w(i*h,xn,zeros(nu,1),w);
  v.draw(i*h,xn);
  pause(0.0025)
  
end

end