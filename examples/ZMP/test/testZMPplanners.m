function testZMPplanners

oldpath = addpath(fullfile(pwd,'..'));

limp = LinearInvertedPendulum2D(1.0);
ts = linspace(0,10,100); 
x0 = randn;
zmptraj = setOutputFrame(PPTrajectory(spline(ts,x0 + 0.5*cos(ts*pi))),desiredZMP1D);
%comtraj = ZMPplan(limp,x0,0,zmptraj);
c = ZMPtracker(limp,zmptraj);
comtraj2 = ZMPplanFromTracker(limp,x0,0,zmptraj,c);

figure(1); clf; hold on;
h = fnplt(zmptraj); set(h,'Color','k');
%h = fnplt(comtraj); set(h,'Color','b');
h = fnplt(comtraj2); set(h,'Color','r');
%legend('com','zmp from closed form','zmp from tracker');
legend('com','zmp from tracker');


limp = LinearInvertedPendulum(1.0);
x0 = randn(2,1);
zmptraj = setOutputFrame(PPTrajectory(spline(ts,[x0(1) + 0.5*cos(ts*pi); x0(2) + sin(ts*pi)])),desiredZMP);
%comtraj = ZMPplan(limp,x0,zeros(2,1),zmptraj);
%comtraj2 = ZMPplanFromTracker(limp,x0,zeros(2,1),zmptraj);
c = ZMPtracker(limp,zmptraj);
comtraj2 = ZMPplanFromTracker(limp,x0,zeros(2,1),zmptraj,c);

figure(2); clf;
subplot(2,1,1); hold on;
h = fnplt(zmptraj(1));  set(h,'Color','k');
%h = fnplt(comtraj(1));  set(h,'Color','b');
h = fnplt(comtraj2(1)); set(h,'Color','r');
subplot(2,1,2); hold on;
h = fnplt(zmptraj(2)); set(h,'Color','k');
%h = fnplt(comtraj(2)); set(h,'Color','b');
h = fnplt(comtraj2(2)); set(h,'Color','r');



