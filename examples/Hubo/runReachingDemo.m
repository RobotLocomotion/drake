function runReachingDemo

[sys,v,r] = HuboSaggitalPlant();
control = HuboReachingControl();
sys = feedback(sys,control);

x0 = Point(r.getStateFrame);
x0.RHP = -.32;
x0.LHP = -.32;
x0.RKP = .5;
x0.LKP = .5;
x0.RAP = -.25;
x0.LAP = -.25;
x0 = r.resolveConstraints(double(x0));
v.draw(0,double(x0));
v.debug = true;

simulate(cascade(sys,v),[0 inf]);

return;
xtraj= simulate(sys,[0 4],x0);
v.playback_speed = 0.2;
%v.playback(xtraj,struct('slider',true));
