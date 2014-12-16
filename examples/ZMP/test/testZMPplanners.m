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
Q_scale = rand()*10 + 1;
limp = LinearInvertedPendulum(ht);
x0 = randn(2,1);
zmptraj = setOutputFrame(PPTrajectory(spline(ts,[x0(1) + 0.5*cos(ts*pi); x0(2) + sin(ts*pi)])),desiredZMP);
tic 
[c_closed_form,V,comtraj] = ZMPtracker(limp,zmptraj,struct('use_tvlqr',false,'ignore_frames',true,'com0',x0,'Qy',Q_scale*diag([0,0,0,0,1,1])));
toc
tic 
comtraj_harada = COMplan(limp,x0,comtraj.eval(tf),zmptraj);
toc
tic
c_tvlqr = ZMPtracker(limp,zmptraj,struct('use_tvlqr',true,'ignore_frames',true,'com0',x0,'Qy',Q_scale*diag([0,0,0,0,1,1])));
toc
comtraj_sim_closed = COMplanFromTracker(limp,x0, [0;0], ts([1,end]), c_closed_form);
ddcomtraj_sim = fnder(comtraj_sim_closed, 2);
comtraj_sim_tvlqr = COMplanFromTracker(limp,x0, [0;0], ts([1,end]), c_tvlqr);

figure(2); clf;
subplot(2,1,1); hold on;
h = fnplt(zmptraj(1));  set(h,'Color','k');
h = fnplt(comtraj(1));  set(h,'Color','b');
h = fnplt(comtraj_harada(1)); set(h,'Color','r','LineStyle','none','Marker','^');
h = fnplt(comtraj_sim_closed(1)); set(h,'Color','g','LineStyle','none','Marker','+');
h = fnplt(comtraj_sim_closed(1) - ht/g * ddcomtraj_sim(1)); set(h,'Color','b','LineStyle','none','Marker','o');
subplot(2,1,2); hold on;
h = fnplt(zmptraj(2)); set(h,'Color','k');
h = fnplt(comtraj(2)); set(h,'Color','b');
h = fnplt(comtraj_harada(2)); set(h,'Color','r','LineStyle','none','Marker','^');
h = fnplt(comtraj_sim_closed(2)); set(h,'Color','g','LineStyle','none','Marker','+');
h = fnplt(comtraj_sim_closed(2) - ht/g * ddcomtraj_sim(2)); set(h,'Color','b','LineStyle','none','Marker','o');
legend('zmp', 'com from closed form', 'com from Harada analytic', 'com from sim', 'zmp from sim');

tsample = linspace(ts(1), ts(end), 100);
valuecheck(comtraj_sim_closed.eval(tsample), comtraj_sim_tvlqr.eval(tsample), 1e-3);
