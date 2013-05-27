function testZMPplan_coordinates

% zmp plan should be coordinate independent (shifting the origin should
% just shift the resulting plan)

oldpath = path;
%oldpath = addpath(fullfile(pwd,'..'));

x0 = 0; y0 = 0;
[zmptraj,comtraj] = plan(x0,y0);


x0 = 1000;%*randn(); 
y0 = 1000;%*randn();
tol = 1e-3;  % note: low tolerance seems potentially reasonable since we're integrating the result of multiplying big numbers
[zmptraj_offset,comtraj_offset] = plan(x0,y0);

path(oldpath);

valuecheck(zmptraj,zmptraj_offset-[x0;y0]);
valuecheck(comtraj,comtraj_offset-[x0;y0],tol);


return;

figure(1);
clf; 
subplot(2,1,1); hold on;
fnplt(zmptraj(1));
fnplt(comtraj(1));
subplot(2,1,2); hold on;
fnplt(zmptraj(2));
fnplt(comtraj(2));


end

function [zmptraj,comtraj] = plan(x0,y0,options)

limp = LinearInvertedPendulum(1.0);
ts = linspace(0,10,100); 
zmptraj = setOutputFrame(PPTrajectory(spline(ts,[x0 + 0.5*cos(ts*pi); y0 + sin(ts*pi)])),desiredZMP);
options.use_mex = true;
[c_mex,V_mex] = ZMPtracker(limp,zmptraj,options);
c_mex = c_mex.inInputFrame(limp.getStateFrame);
c_mex = c_mex.inOutputFrame(limp.getInputFrame);
V_mex = V_mex.inFrame(limp.getStateFrame);
options.use_mex = false;
[c,V] = ZMPtracker(limp,zmptraj,options);
c = c.inInputFrame(limp.getStateFrame);
c = c.inOutputFrame(limp.getInputFrame);
V = V.inFrame(limp.getStateFrame);

tol = .1;
valuecheck(V_mex.S,V.S,tol,true);
valuecheck(V_mex.s1,V.s1,tol,true);
valuecheck(V_mex.s2,V.s2,tol,true);
valuecheck(c_mex.y0,c.y0,tol,true);
valuecheck(c_mex.D,c.D,tol,true);

comtraj = COMplanFromTracker(limp,[x0;y0],zeros(2,1),zmptraj.tspan,c_mex);
%comtraj = ZMPplanner(limp,[x0;y0],zeros(2,1),zmptraj);

end

