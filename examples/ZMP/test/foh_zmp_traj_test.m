function debug_zmp

load zmp_plan_debug;

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

  ct = ZMPplan(limp,com0,comf,zmptraj);

  for i=1:2
    figure(i); clf; hold on;
    h = fnplt(comtraj(i)); set(h,'Color','r');
    h = fnplt(ct(i));
    h = fnplt(zmptraj(i)); set(h,'Color','k');
    legend('from tracker','from analytic','desired zmp');
  end
end