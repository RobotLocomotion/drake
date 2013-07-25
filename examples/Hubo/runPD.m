function runPD

[sys,v] = HuboSaggitalPlant();

theta_d = Point(sys.getInputFrame);
theta_d.RHP = -.32;
theta_d.LHP = -.32;
theta_d.RKP = .5;
theta_d.LKP = .5;
theta_d.RAP = -.25;
theta_d.LAP = -.25;
sys = cascade(ConstOrPassthroughSystem(theta_d),sys);  % command a constant desired theta

x0 = Point(r.getStateFrame);
%x0.base_link_z = 1;
x0 = r.resolveConstraints(double(x0));
v.draw(0,double(x0));

simulate(cascade(sys,v),[0 inf],x0);

return;
xtraj= simulate(sys,[0 4],x0);
v.playback_speed = 0.2;
%v.playback(xtraj,struct('slider',true));

end