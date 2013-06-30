function foh_zmp_traj_test

load zmp_plan_debug;

p = addpath(fullfile(pwd,'..'));
% resample as a spline
%breaks = getBreaks(zmptraj);
%zmptraj = PPTrajectory(spline(breaks,eval(zmptraj,breaks)));

planar = false;
if (planar) % debug planar case 
  comtraj = comtraj(2);
  zmptraj = zmptraj(2); zmp_pp = zmptraj.pp;
end

com0 = comtraj.eval(comtraj.tspan(1));
comf = comtraj.eval(comtraj.tspan(end));

if (planar) % debug planar case 
  figure(2); clf; hold on;
  fnplt(comtraj.pp,'r');
  ct = LinearInvertedPendulum2D.COMtrajFromZMP(h,com0,comf,zmp_pp);
  fnplt(ct);
  ct2 = LinearInvertedPendulum2D.COMsplineFromZMP(h,com0,comf,zmp_pp);
  fnplt(ct2,'g');
  fnplt(zmp_pp,'k');
  
  figure(3); clf; hold on;
  fnplt(fnder(comtraj.pp),'r');
  fnplt(fnder(ct));
  fnplt(fnder(ct2),'g');
else
  zmptraj = setOutputFrame(zmptraj,desiredZMP);

  limp = LinearInvertedPendulum(h);

  options.use_tvlqr = false;
  options.com0 = com0;
  options.comdot0 = zeros(2,1);
  options.ignore_frames = true;
%  options.compute_lyapunov = false;
  tic
  [ct,~,comtraj] = ZMPtracker(limp,zmptraj,options);
  toc
  
  for i=1:2
    figure(i); clf; hold on;
    h = fnplt(comtraj(i)); set(h,'Color','r');
    h = fnplt(zmptraj(i)); set(h,'Color','k');
    legend('from tracker','from analytic','desired zmp');
  end
end

path(p);