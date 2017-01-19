%Closed-loop simulation

close all

%Cost weighting matrices
Q = blkdiag(10*eye(6), 1*eye(6));
R = .1*eye(4);
Qf = Q;

N = 301;
tsamp = linspace(0,3,N);
dt = tsamp(2)-tsamp(1);

xsamp1 = xtraj1.eval(tsamp);
usamp1 = utraj1.eval(tsamp);
xsamp2 = xtraj2.eval(tsamp);
usamp2 = utraj2.eval(tsamp);

A1 = zeros(12,12,N);
B1 = zeros(12,4,N);
A2 = A1;
B2 = B1;
for k = 1:N
    [~,df1] = p.dynamics_w(0, xsamp1(:,k), usamp1(:,k), zeros(3,1));
    [~,df2] = p.dynamics_w(0, xsamp2(:,k), usamp2(:,k), zeros(3,1));
    A1(:,:,k) = expm(dt*df1(:,2:13));
    B1(:,:,k) = iexpm(df1(:,2:13),dt)*df1(:,14:17);
    A2(:,:,k) = expm(dt*df2(:,2:13));
    B2(:,:,k) = iexpm(df2(:,2:13),dt)*df2(:,14:17);
end

P1 = zeros(12,12,N);
P1(:,:,N) = Qf;
K1 = zeros(4,12,N-1);
for k = (N-1):-1:1
    K1(:,:,k) = (R+B1(:,:,k)'*P1(:,:,k+1)*B1(:,:,k))\(B1(:,:,k)'*P1(:,:,k+1)*A1(:,:,k));
    P1(:,:,k) = Q + K1(:,:,k)'*R*K1(:,:,k) + (A1(:,:,k)-B1(:,:,k)*K1(:,:,k))'*P1(:,:,k+1)*(A1(:,:,k)-B1(:,:,k)*K1(:,:,k));
end

P2 = zeros(12,12,N);
K2 = zeros(4,12,N-1);
P2(:,:,N) = Qf;
for k = (N-1):-1:1
    K2(:,:,k) = (R+B2(:,:,k)'*P2(:,:,k+1)*B2(:,:,k))\(B2(:,:,k)'*P2(:,:,k+1)*A2(:,:,k));
    P2(:,:,k) = Q + K2(:,:,k)'*R*K2(:,:,k) + (A2(:,:,k)-B2(:,:,k)*K2(:,:,k))'*P2(:,:,k+1)*(A2(:,:,k)-B2(:,:,k)*K2(:,:,k));
end

figure();
subplot(4,1,1);
plot(tsamp, usamp1(1,:));
hold on;
plot(tsamp, usamp2(1,:));
subplot(4,1,2);
plot(tsamp, usamp1(2,:));
hold on;
plot(tsamp, usamp2(2,:));
subplot(4,1,3);
plot(tsamp, usamp1(3,:));
hold on;
plot(tsamp, usamp2(3,:));
subplot(4,1,4);
plot(tsamp, usamp1(4,:));
hold on;
plot(tsamp, usamp2(4,:));

% design FIR filter to filter noise to 1/10 of Nyquist rate
b = fir1(48, 0.05);
% generate Gaussian (normally-distributed) white noise
nx = randn(N,1);
ny = randn(N,1);
nz = randn(N,1);
% apply to filter to yield bandlimited noise
wx = filter(b,1,nx);
wy = filter(b,1,ny);
wz = .1*filter(b,1,nz);
w = 2*[wx wy wz]';
%w = zeros(3,N);

figure()
subplot(2,1,1);
plot(nx);
hold on;
plot(wx);
subplot(2,1,2);
plot(ny);
hold on;
plot(wy);

%Simulate with random wind input
xcl1 = zeros(12,N);
xcl1(:,1) = xsamp1(:,1);
for k = 1:(N-1)
    [~,xk] = ode3(@(t,x)p.dynamics_w(t,x,usamp1(:,k)-K1(:,:,k)*(xcl1(:,k)-xsamp1(:,k)), w(:,k)), [0 dt], xcl1(:,k), dt);
    xcl1(:,k+1) = xk(:,2);
end

figure();
subplot(3,1,1);
plot(tsamp, xsamp1(1,:));
hold on
plot(tsamp, xcl1(1,:));
subplot(3,1,2);
plot(tsamp, xsamp1(2,:));
hold on;
plot(tsamp, xcl1(2,:));
subplot(3,1,3);
plot(tsamp, xsamp1(3,:));
hold on
plot(tsamp, xcl1(3,:));

%Simulate with random wind input
xcl2 = zeros(12,N);
xcl2(:,1) = xsamp2(:,1);
for k = 1:(N-1)
    [~,xk] = ode3(@(t,x)p.dynamics_w(t,x,usamp2(:,k)-K2(:,:,k)*(xcl2(:,k)-xsamp2(:,k)), w(:,k)), [0 dt], xcl2(:,k), dt);
    xcl2(:,k+1) = xk(:,2);
end

figure();
subplot(3,1,1);
plot(tsamp, xsamp2(1,:));
hold on
plot(tsamp, xcl2(1,:));
subplot(3,1,2);
plot(tsamp, xsamp2(2,:));
hold on;
plot(tsamp, xcl2(2,:));
subplot(3,1,3);
plot(tsamp, xsamp2(3,:));
hold on
plot(tsamp, xcl2(3,:));

%Check for collisions
tree_width = ((.1+.4*[.45 .9 .95 .5 .65 .85]')+.5).^2;
for j = 1:size(xcl1,2)
    check1(:,j) = (treeDistance(xcl1(1:6,j)) < tree_width);
end
any(check1(:))
%v.playback(PPTrajectory(foh(tsamp,xcl1)), struct('slider',true));

for j = 1:size(xcl2,2)
    check2(:,j) = (treeDistance(xcl2(1:6,j)) < tree_width);
end
any(check2(:))
%v.playback(PPTrajectory(foh(tsamp2, xcl2)) ,struct('slider',true));

%Write movie files
%v.playbackAVI(xcl1, 'quad1.avi');
%v.playbackAVI(xcl2, 'quad2.avi');
% setenv('PATH', [getenv('PATH') ':/usr/local/bin']);
% setenv('PATH', [getenv('PATH') ':/Library/TeX/texbin']);
% v.playbackSWF(xcl1, 'swing1.swf');
% v.playbackSWF(xcl2, 'swing2.swf');

