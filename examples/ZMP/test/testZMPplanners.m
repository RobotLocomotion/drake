function testZMPplanners

%oldpath = path;
oldpath = addpath(fullfile(pwd,'..'));

tf = 10;
limp = LinearInvertedPendulum2D(1.0);
ts = linspace(0,tf,100); 
x0 = randn;
zmptraj = setOutputFrame(PPTrajectory(spline(ts,x0 + 0.5*cos(ts*pi))),desiredZMP1D);
tic 
c = ZMPtracker(limp,zmptraj);
toc
tic
comtraj = ZMPplanFromTracker(limp,x0,0,zmptraj,c);
toc
tic
comtraj2 = ZMPplan(limp,x0,eval(comtraj,tf),zmptraj);
toc

figure(1); clf; hold on;
h = fnplt(zmptraj); set(h,'Color','k');
h = fnplt(comtraj2); set(h,'Color','b');
h = fnplt(comtraj); set(h,'Color','r');
legend('zmp','com from closed form','com from Harada analytic');

ht = 1.0;
g = 9.81;
limp = LinearInvertedPendulum(ht);
x0 = randn(2,1);
zmptraj = setOutputFrame(PPTrajectory(spline(ts,[x0(1) + 0.5*cos(ts*pi); x0(2) + sin(ts*pi)])),desiredZMP);
tic 
[c,V,comtraj] = ZMPtracker(limp,zmptraj,struct('use_tvlqr',false,'ignore_frames',true,'com0',x0));
toc
tic 
comtraj_harada = COMplan(limp,x0,eval(comtraj,tf),zmptraj);
toc
tic
c_tvlqr = ZMPtracker(limp,zmptraj,struct('use_tvlqr',true,'ignore_frames',true,'com0',x0));
toc
tic
comtraj_sim = COMplanFromTracker(limp,x0, [0;0], ts([1,end]), c_tvlqr);
toc

figure(2); clf;
subplot(2,1,1); hold on;
h = fnplt(zmptraj(1));  set(h,'Color','k');
h = fnplt(comtraj(1));  set(h,'Color','b');
h = fnplt(comtraj_harada(1)); set(h,'Color','r');
h = fnplt(comtraj_sim(1)); set(h,'Color','g');
subplot(2,1,2); hold on;
h = fnplt(zmptraj(2)); set(h,'Color','k');
h = fnplt(comtraj(2)); set(h,'Color','b');
h = fnplt(comtraj_harada(2)); set(h,'Color','r');
h = fnplt(comtraj_sim(2)); set(h,'Color','g','Marker','o');
legend('zmp', 'com from closed form', 'com from Harada analytic', 'com from sim')

coms = comtraj.eval(ts);
ddcoms = eval(fnder(comtraj, 2), ts);
com2s = comtraj_harada.eval(ts);
ddcom2s = [[0;0], (diff(com2s, 2, 2)./ (ts(2)-ts(1))^2), [0;0]];
zmps = zmptraj.eval(ts);

% Plot acceleration of CoM divided by distance from CoM to ZmP, which
% should always be equal to 1 from the LIPM dynamics. It's not always 1,
% however, because the actual and desired ZMP do not always match.
figure(3)
clf
subplot(2,1,1);
hold on
plot(ts, ddcoms(1,:) ./ ((g/ht)*(coms(1,:) - zmps(1,:))), 'b.-');
plot(ts, ddcom2s(1,:) / ((g/ht)*(com2s(1,:) - zmps(1,:))), 'r.-');

subplot(2,1,2);
hold on
plot(ts, ddcoms(2,:) ./ ((g/ht)*(coms(2,:) - zmps(2,:))), 'b.-');
plot(ts, ddcom2s(2,:) / ((g/ht)*(com2s(2,:) - zmps(2,:))), 'r.-');
legend('closed form', 'tracker');

