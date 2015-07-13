function xtrajSim = simulateEuler(sysCl,tend,x0,dt,sys_type,xp,up)

if nargin < 5
    sys_type = 'double';
    xp = [];
    up = [];
end

ts = 0:dt:tend;
xs = zeros(length(x0),length(ts));
xs(:,1) = x0;


for k = 1:(length(ts)-1)
    xnow = xs(:,k);
    if strcmp(sys_type, 'msspoly')
       xdot = subs(sysCl.p_dynamics_traj.eval(ts(k)),[xp;up],[xnow;zeros(4,1)]);
    else
       xdot = sysCl.dynamics(ts(k),xnow,zeros(4,1));
    end
    
    xnext = xnow + dt*xdot;
    
    xs(:,k+1) = xnext;
    
end


xtrajSim = PPTrajectory(spline(ts,xs));

xtrajSim = xtrajSim.setOutputFrame(sysCl.getStateFrame);