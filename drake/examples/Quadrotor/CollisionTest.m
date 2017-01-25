%Closed-loop simulation

close all

%Cost weighting matrices
Q = blkdiag(10*eye(6), 1*eye(6));
R = .1*eye(4);
Qf = Q;

N = 301;
tsamp = linspace(0,3,N);
dt = tsamp(2)-tsamp(1);

% xsamp1 = xtraj1.eval(tsamp);
% usamp1 = utraj1.eval(tsamp);
% xsamp2 = xtraj2.eval(tsamp);
% usamp2 = utraj2.eval(tsamp);
% xsamp3 = xtraj3.eval(tsamp);
% usamp3 = utraj3.eval(tsamp);
% xsamp4 = xtraj4.eval(tsamp);
% usamp4 = utraj4.eval(tsamp);

A1 = zeros(12,12,N);
B1 = zeros(12,4,N);
A2 = A1;
B2 = B1;
A3 = A1;
B3 = B1;
A4 = A1;
B4 = B1;
for k = 1:N
    [~,df1] = p.dynamics_w(0, xsamp1(:,k), usamp1(:,k), zeros(3,1));
    [~,df2] = p.dynamics_w(0, xsamp2(:,k), usamp2(:,k), zeros(3,1));
    [~,df3] = p.dynamics_w(0, xsamp3(:,k), usamp3(:,k), zeros(3,1));
    [~,df4] = p.dynamics_w(0, xsamp4(:,k), usamp4(:,k), zeros(3,1));
    A1(:,:,k) = expm(dt*df1(:,2:13));
    B1(:,:,k) = iexpm(df1(:,2:13),dt)*df1(:,14:17);
    A2(:,:,k) = expm(dt*df2(:,2:13));
    B2(:,:,k) = iexpm(df2(:,2:13),dt)*df2(:,14:17);
    A3(:,:,k) = expm(dt*df3(:,2:13));
    B3(:,:,k) = iexpm(df3(:,2:13),dt)*df3(:,14:17);
    A4(:,:,k) = expm(dt*df4(:,2:13));
    B4(:,:,k) = iexpm(df4(:,2:13),dt)*df4(:,14:17);
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

P3 = zeros(12,12,N);
K3 = zeros(4,12,N-1);
P3(:,:,N) = Qf;
for k = (N-1):-1:1
    K3(:,:,k) = (R+B3(:,:,k)'*P3(:,:,k+1)*B3(:,:,k))\(B3(:,:,k)'*P3(:,:,k+1)*A3(:,:,k));
    P3(:,:,k) = Q + K3(:,:,k)'*R*K3(:,:,k) + (A3(:,:,k)-B3(:,:,k)*K3(:,:,k))'*P3(:,:,k+1)*(A3(:,:,k)-B3(:,:,k)*K3(:,:,k));
end

P4 = zeros(12,12,N);
K4 = zeros(4,12,N-1);
P4(:,:,N) = Qf;
for k = (N-1):-1:1
    K4(:,:,k) = (R+B4(:,:,k)'*P4(:,:,k+1)*B4(:,:,k))\(B4(:,:,k)'*P4(:,:,k+1)*A4(:,:,k));
    P4(:,:,k) = Q + K4(:,:,k)'*R*K4(:,:,k) + (A4(:,:,k)-B4(:,:,k)*K4(:,:,k))'*P4(:,:,k+1)*(A4(:,:,k)-B4(:,:,k)*K4(:,:,k));
end

figure();
subplot(4,1,1);
plot(tsamp, usamp1(1,:));
hold on;
plot(tsamp, usamp2(1,:));
plot(tsamp, usamp3(1,:));
plot(tsamp, usamp4(1,:));
subplot(4,1,2);
plot(tsamp, usamp1(2,:));
hold on;
plot(tsamp, usamp2(2,:));
plot(tsamp, usamp3(2,:));
plot(tsamp, usamp4(2,:));
subplot(4,1,3);
plot(tsamp, usamp1(3,:));
hold on;
plot(tsamp, usamp2(3,:));
plot(tsamp, usamp3(3,:));
plot(tsamp, usamp4(3,:));
subplot(4,1,4);
plot(tsamp, usamp1(4,:));
hold on;
plot(tsamp, usamp2(4,:));
plot(tsamp, usamp3(4,:));
plot(tsamp, usamp4(4,:));

%Calculate minimum distances
tree_width = (.1+.4*[.45 .9 .95 .5 .65 .85]')+.55;
minDist1 = 1000;
minDist2 = minDist1;
minDist3 = minDist1;
minDist4 = minDist1;
for k = 1:(N-1)
    dist1 = sqrt(treeDistance(xsamp1(1:6,k))) - tree_width;
    if min(dist1) < minDist1
        minDist1 = min(dist1);
    end
    dist2 = sqrt(treeDistance(xsamp2(1:6,k))) - tree_width;
    if min(dist2) < minDist2
        minDist2 = min(dist2);
    end
    dist3 = sqrt(treeDistance(xsamp3(1:6,k))) - tree_width;
    if min(dist3) < minDist3
        minDist3 = min(dist3);
    end
    dist4 = sqrt(treeDistance(xsamp4(1:6,k))) - tree_width;
    if min(dist4) < minDist4
        minDist4 = min(dist4);
    end
end

minDist1
minDist2
minDist3
minDist4

% design FIR filter to filter noise to 5% of Nyquist rate
b = fir1(48, 0.05);

hit1 = zeros(100,1);
hit2 = zeros(100,1);
hit3 = zeros(100,1);
hit4 = zeros(100,1);
for j = 1:100

% generate Gaussian (normally-distributed) white noise
nx = randn(N,1);
ny = randn(N,1);
nz = randn(N,1);
% apply to filter to yield bandlimited noise
wx = filter(b,1,nx);
wy = filter(b,1,ny);
wz = filter(b,1,nz);
w = [(.5/max(wx))*wx (.5/max(abs(wy)))*wy (.05/max(abs(wz)))*wz]';
%w = zeros(3,N);

%Simulate with random wind input
xcl1 = zeros(12,N);
xcl1(:,1) = xsamp1(:,1);
for k = 1:(N-1)
    [~,xk] = ode3(@(t,x)p.dynamics_w(t,x,usamp1(:,k)-K1(:,:,k)*(xcl1(:,k)-xsamp1(:,k)), w(:,k)), [0 dt], xcl1(:,k), dt);
    xcl1(:,k+1) = xk(:,2);
end

%Simulate with random wind input
xcl2 = zeros(12,N);
xcl2(:,1) = xsamp2(:,1);
for k = 1:(N-1)
    [~,xk] = ode3(@(t,x)p.dynamics_w(t,x,usamp2(:,k)-K2(:,:,k)*(xcl2(:,k)-xsamp2(:,k)), w(:,k)), [0 dt], xcl2(:,k), dt);
    xcl2(:,k+1) = xk(:,2);
end

%Simulate with random wind input
xcl3 = zeros(12,N);
xcl3(:,1) = xsamp3(:,1);
for k = 1:(N-1)
    [~,xk] = ode3(@(t,x)p.dynamics_w(t,x,usamp3(:,k)-K3(:,:,k)*(xcl3(:,k)-xsamp3(:,k)), w(:,k)), [0 dt], xcl3(:,k), dt);
    xcl3(:,k+1) = xk(:,2);
end

%Simulate with random wind input
xcl4 = zeros(12,N);
xcl4(:,1) = xsamp4(:,1);
for k = 1:(N-1)
    [~,xk] = ode3(@(t,x)p.dynamics_w(t,x,usamp4(:,k)-K4(:,:,k)*(xcl4(:,k)-xsamp4(:,k)), w(:,k)), [0 dt], xcl4(:,k), dt);
    xcl4(:,k+1) = xk(:,2);
end

%Check for collisions
tree_width = ((.1+.4*[.45 .9 .95 .5 .65 .85]')+.54).^2;
for k = 1:size(xcl1,2)
    check1(:,k) = (treeDistance(xcl1(1:6,k)) < tree_width);
end
hit1(j) = any(check1(:));
%v.playback(PPTrajectory(foh(tsamp,xcl1)), struct('slider',true));

for k = 1:size(xcl2,2)
    check2(:,k) = (treeDistance(xcl2(1:6,k)) < tree_width);
end
hit2(j) = any(check2(:));

for k = 1:size(xcl3,2)
    check3(:,k) = (treeDistance(xcl3(1:6,k)) < tree_width);
end
hit3(j) = any(check3(:));

for k = 1:size(xcl3,2)
    check4(:,k) = (treeDistance(xcl4(1:6,k)) < tree_width);
end
hit4(j) = any(check4(:));

end

sum(hit1)
sum(hit2)
sum(hit3)
sum(hit4)

% figure();
% subplot(3,1,1);
% plot(tsamp, xsamp1(1,:));
% hold on
% plot(tsamp, xcl1(1,:));
% subplot(3,1,2);
% plot(tsamp, xsamp1(2,:));
% hold on;
% plot(tsamp, xcl1(2,:));
% subplot(3,1,3);
% plot(tsamp, xsamp1(3,:));
% hold on
% plot(tsamp, xcl1(3,:));
% 
% figure();
% subplot(3,1,1);
% plot(tsamp, xsamp2(1,:));
% hold on
% plot(tsamp, xcl2(1,:));
% subplot(3,1,2);
% plot(tsamp, xsamp2(2,:));
% hold on;
% plot(tsamp, xcl2(2,:));
% subplot(3,1,3);
% plot(tsamp, xsamp2(3,:));
% hold on
% plot(tsamp, xcl2(3,:));

%v.playback(PPTrajectory(foh(tsamp2, xcl2)) ,struct('slider',true));

%Write movie files
%v.playbackAVI(xcl1, 'quad1.avi');
%v.playbackAVI(xcl2, 'quad2.avi');
% setenv('PATH', [getenv('PATH') ':/usr/local/bin']);
% setenv('PATH', [getenv('PATH') ':/Library/TeX/texbin']);
% v.playbackSWF(xcl1, 'swing1.swf');
% v.playbackSWF(xcl2, 'swing2.swf');

