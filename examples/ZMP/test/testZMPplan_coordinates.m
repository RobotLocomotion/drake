function testZMPplan_coordinates

% zmp plan should be coordinate independent (shifting the origin should
% just shift the resulting plan)

oldpath = path;
%oldpath = addpath(fullfile(pwd,'..'));

x0 = 0; y0 = 0;
[zmptraj,comtraj] = plan(x0,y0);


x0 = 100;%*randn(); 
y0 = 100;%*randn();
tol = 1e-3;  % note: low tolerance seems potentially reasonable since we're integrating the result of multiplying big numbers
[zmptraj_offset,comtraj_offset] = plan(x0,y0);

figure(1); clf; 
for i=1:2;
  subplot(2,1,i); hold on;
  fnplt(comtraj,i);
  h = fnplt(comtraj_offset-[x0;y0],i); set(h,'Color','r');
end
drawnow;

path(oldpath);

for ts = getBreaks(comtraj);
  valuecheck(eval(zmptraj,ts),eval(zmptraj_offset,ts)-[x0;y0]);
  valuecheck(eval(comtraj,ts),eval(comtraj_offset,ts)-[x0;y0],tol);
end



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

