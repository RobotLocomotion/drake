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
legend('zmp','com from closed form','com from tracker');

limp = LinearInvertedPendulum(1.0);
x0 = randn(2,1);
zmptraj = setOutputFrame(PPTrajectory(spline(ts,[x0(1) + 0.5*cos(ts*pi); x0(2) + sin(ts*pi)])),desiredZMP);
tic 
[c,V,comtraj] = ZMPtracker(limp,zmptraj,struct('use_tvlqr',false,'ignore_frames',true,'com0',x0));
toc
tic 
comtraj2 = COMplan(limp,x0,eval(comtraj,tf),zmptraj);
toc

figure(2); clf;
subplot(2,1,1); hold on;
h = fnplt(zmptraj(1));  set(h,'Color','k');
h = fnplt(comtraj(1));  set(h,'Color','b');
h = fnplt(comtraj2(1)); set(h,'Color','r');
subplot(2,1,2); hold on;
h = fnplt(zmptraj(2)); set(h,'Color','k');
h = fnplt(comtraj(2)); set(h,'Color','b');
h = fnplt(comtraj2(2)); set(h,'Color','r');



