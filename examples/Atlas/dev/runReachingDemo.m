function runReachingDemo

options.ground = false;
r = RigidBodyManipulator('urdf/atlas.urdf');
for i=1:length(r.body), r.body(i).contact_pts=[]; end
r = compile(r);
v = r.constructVisualizer(options);
v.display_dt = .05;

[Kp,Kd] = getPDGains(r);
sys = pdcontrol(r,Kp,Kd);

c = ReachingControl(sys,r);
sys = feedback(sys,c);

simulate(cascade(sys,v),[0 10]);