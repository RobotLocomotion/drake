function testZMPplan_coordinates

% zmp plan should be coordinate independent (shifting the origin should
% just shift the resulting plan)

x0 = 0; y0 = 0;
[zmptraj,comtraj] = plan(x0,y0);

%x0 = 10000;%*randn(); 
%y0 = 10000;%*randn();
x0 = 10; 
y0 = 10;
tol = 1e-2;  % note: low tolerance seems potentially reasonable since we're integrating the result of multiplying big numbers
[zmptraj_offset,comtraj_offset] = plan(x0,y0);

figure(1); clf; 
for i=1:2;
  subplot(2,1,i); hold on;
  fnplt(comtraj,i);
  h = fnplt(comtraj_offset-[x0;y0],i); set(h,'Color','r');
end

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

function [zmptraj,comtraj] = plan(x0,y0)

limp = LinearInvertedPendulum(1.0);
ts = linspace(0,10,100); 
zmptraj = setOutputFrame(PPTrajectory(spline(ts,[x0 + 0.5*cos(ts*pi); y0 + sin(ts*pi)])),desiredZMP);
comtraj = ZMPplanner(limp,[x0;y0],zeros(2,1),zmptraj);

end

