%Closed-loop simulation

close all

Nq = 7;
Nx = 14;
Nu = 7;

%Cost weighting matrices
Q = diag([100*ones(Nq,1);10*ones(Nq,1)]);
R = 0.01*eye(Nu);
Qf = 5*Q;

N = 2000;
tsamp1 = linspace(0,xtraj1.tspan(2),N);
tsamp2 = linspace(0,xtraj2.tspan(2),N);
dt1 = tsamp1(2)-tsamp1(1);
dt2 = tsamp2(2)-tsamp2(1);

% xsamp1 = xtraj1.eval(tsamp1);
% usamp1 = utraj1.eval(tsamp1);
% xsamp2 = xtraj2.eval(tsamp2);
% usamp2 = utraj2.eval(tsamp2);

A1 = zeros(14,14,N);
B1 = zeros(14,7,N);
A2 = A1;
B2 = B1;
for k = 1:N
    [~,df1] = p.dynamics_w(0, xsamp1(:,k), usamp1(:,k), zeros(3,1));
    [~,df2] = p.dynamics_w(0, xsamp2(:,k), usamp2(:,k), zeros(3,1));
    A1(:,:,k) = expm(dt1*df1(:,1+(1:Nx)));
    B1(:,:,k) = iexpm(df1(:,1+(1:Nx)),dt1)*df1(:,1+Nx+(1:Nu));
    A2(:,:,k) = expm(dt2*df2(:,1+(1:Nx)));
    B2(:,:,k) = iexpm(df2(:,1+(1:Nx)),dt2)*df2(:,1+Nx+(1:Nu));
end

P1 = zeros(14,14,N);
P1(:,:,N) = Qf;
K1 = zeros(7,14,N-1);
for k = (N-1):-1:1
    K1(:,:,k) = (R+B1(:,:,k)'*P1(:,:,k+1)*B1(:,:,k))\(B1(:,:,k)'*P1(:,:,k+1)*A1(:,:,k));
    P1(:,:,k) = Q + K1(:,:,k)'*R*K1(:,:,k) + (A1(:,:,k)-B1(:,:,k)*K1(:,:,k))'*P1(:,:,k+1)*(A1(:,:,k)-B1(:,:,k)*K1(:,:,k));
end

P2 = zeros(14,14,N);
K2 = zeros(7,14,N-1);
P2(:,:,N) = Qf;
for k = (N-1):-1:1
    K2(:,:,k) = (R+B2(:,:,k)'*P2(:,:,k+1)*B2(:,:,k))\(B2(:,:,k)'*P2(:,:,k+1)*A2(:,:,k));
    P2(:,:,k) = Q + K2(:,:,k)'*R*K2(:,:,k) + (A2(:,:,k)-B2(:,:,k)*K2(:,:,k))'*P2(:,:,k+1)*(A2(:,:,k)-B2(:,:,k)*K2(:,:,k));
end

figure();
subplot(7,1,1);
plot(tsamp1, usamp1(1,:));
hold on;
plot(tsamp2, usamp2(1,:));
subplot(7,1,2);
plot(tsamp1, usamp1(2,:));
hold on;
plot(tsamp2, usamp2(2,:));
subplot(7,1,3);
plot(tsamp1, usamp1(3,:));
hold on;
plot(tsamp2, usamp2(3,:));
subplot(7,1,4);
plot(tsamp1, usamp1(4,:));
hold on;
plot(tsamp2, usamp2(4,:));
subplot(7,1,5);
plot(tsamp1, usamp1(5,:));
hold on;
plot(tsamp2, usamp2(5,:));
subplot(7,1,6);
plot(tsamp1, usamp1(6,:));
hold on;
plot(tsamp2, usamp2(6,:));
subplot(7,1,7);
plot(tsamp1, usamp1(7,:));
hold on;
plot(tsamp2, usamp2(7,:));

% design FIR filter to filter noise to 5% of Nyquist rate
b = fir1(1024, 0.01);

for j = 1:1

% generate Gaussian (normally-distributed) white noise
nx = randn(N,1);
ny = randn(N,1);
nz = randn(N,1);
% apply to filter to yield bandlimited noise
wx = filter(b,1,nx);
wy = filter(b,1,ny);
wz = filter(b,1,nz);
w = [(5/max(wx))*wx (5/max(abs(wy)))*wy (5/max(abs(wz)))*wz]';
%w = zeros(3,N);

figure();
plot(tsamp1, w(1,:));
hold on
plot(tsamp1, w(2,:));
plot(tsamp1, w(3,:));

%Simulate with random ee force
xcl1 = zeros(Nx,N);
xcl1(:,1) = xsamp1(:,1);
for k = 1:(N-1)
    [~,xk] = ode3(@(t,x)p.dynamics_w(t,x,usamp1(:,k)-K1(:,:,k)*(xcl1(:,k)-xsamp1(:,k)), w(:,k)), [0 dt1], xcl1(:,k), dt1);
    xcl1(:,k+1) = xk(:,2);
end

%Simulate with random ee force
xcl2 = zeros(Nx,N);
xcl2(:,1) = xsamp2(:,1);
for k = 1:(N-1)
    [~,xk] = ode3(@(t,x)p.dynamics_w(t,x,usamp2(:,k)-K2(:,:,k)*(xcl2(:,k)-xsamp2(:,k)), w(:,k)), [0 dt2], xcl2(:,k), dt2);
    xcl2(:,k+1) = xk(:,2);
end

xcltraj1 = PPTrajectory(foh(tsamp1,xcl1));
xcltraj1 = xcltraj1.setOutputFrame(p.getStateFrame());
xcltraj2 = PPTrajectory(foh(tsamp2,xcl2));
xcltraj2 = xcltraj2.setOutputFrame(p.getStateFrame());
v = p.constructVisualizer();
v.playback(xcltraj1,struct('slider',true));
v.playback(xcltraj2,struct('slider',true));
end




