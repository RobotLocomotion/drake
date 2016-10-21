function testMassSpringDamperThrust

x0 = [1e-6;0];%needed to avoid a singularity in the solution
run_time = 5;

utraj = PPTrajectory(foh([0 10],[15 15]));

p = PlanarRigidBodyManipulator('../../../../systems/plants/test/MassSpringDamperThrust.urdf');
utraj = utraj.setOutputFrame(p.getInputFrame());
c = cascade(utraj,p);
xtrajp = simulate(c,[0 run_time],x0);
vp = p.constructVisualizer();
vp.axis = [-5 5 -5 5];
vp.playback(xtrajp);

r = RigidBodyManipulator('../../../../systems/plants/test/MassSpringDamperThrust.urdf');
utraj = utraj.setOutputFrame(r.getInputFrame());
cr = cascade(utraj,r);
xtrajr = simulate(cr,[0 run_time],x0);
fnplt(xtrajr,1);
vr = r.constructVisualizer;
vr.playback(xtrajr);

for t=0.01:.5:run_time
    expected = [exp(-t/2)*(-3*sqrt(39)*sin(0.5*sqrt(39)*t)/(39*2)-1.5*cos(0.5*sqrt(39)*t))+3/2;
                4.80384*exp(-t/2)*sin(3.1225*t)];
    result_r = xtrajr.eval(t);
    result_p = xtrajp.eval(t);
    valuecheck(result_r, expected, 0.003);
    valuecheck(result_p, expected, 0.003);
end

end

% TIMEOUT 1500
