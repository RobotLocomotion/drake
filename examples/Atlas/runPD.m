function runPD()

options.floating = true;
s = 'urdf/atlas_minimal_contact.urdf';

dt = 0.001;
r = TimeSteppingRigidBodyManipulator(s,dt,options);
v = r.constructVisualizer;
v.display_dt = 0.01;

[kp,kd] = getPDGains(r.manip); 
pd = pdcontrol(r,kp,kd);

x0 = Point(r.getStateFrame);
x0 = r.manip.resolveConstraints(double(x0));

%theta_des = Point(pd.getInputFrame);
%c = ConstOrPassthroughSystem(theta_des); % command a constant desired theta
%sys = cascade(c,pd); 

sys = pd;

T = 5.0; % sec
if (0)
    tic;
    traj = simulate(sys,[0 T],x0); 
    toc;
    playback(v,traj,struct('slider',true));
else
    s = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  % we are knowingly breaking out to a simulink model with the cascade on the following line.
    sys = cascade(sys,v);
    warning(s);
    simulate(sys,[0 T],x0);
end

end




