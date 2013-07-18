function testZMPplan_coordinates

% zmp plan should be coordinate independent (shifting the origin should
% just shift the resulting plan)

oldpath = addpath(fullfile(pwd,'..'));

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
options.use_tvlqr = true;
[c_tvlqr,V_tvlqr] = ZMPtracker(limp,zmptraj,options);
%c_tvlqr = c_tvlqr.inInputFrame(limp.getStateFrame);
%c_tvlqr = c_tvlqr.inOutputFrame(limp.getInputFrame);
V_tvlqr = V_tvlqr.inFrame(limp.getStateFrame);
options.use_tvlqr = false;
options.com0 = [x0;y0];
options.comdot0 = zeros(2,1);
[c,V,comtraj] = ZMPtracker(limp,zmptraj,options);
%c = c.inInputFrame(limp.getStateFrame);
%c = c.inOutputFrame(limp.getInputFrame);
V = V.inFrame(limp.getStateFrame);

tol = 1e-3;
valuecheck(V.S,V_tvlqr.S,tol,true);
valuecheck(V.s1,V_tvlqr.s1,tol,true);
%valuecheck(V.s2,V_tvlqr.s2,tol,true);  
valuecheck(c.y0,c_tvlqr.y0,100*tol,true);
valuecheck(c.D,c_tvlqr.D,tol,true);

comtraj_ode = COMplanFromTracker(limp,options.com0,options.comdot0,zmptraj.tspan,c_tvlqr);
valuecheck(comtraj,comtraj_ode,tol,true);
%comtraj = ZMPplanner(limp,[x0;y0],zeros(2,1),zmptraj);

end

