function testMassSpringDamper 

r = RigidBodyManipulator('MassSpringDamper.urdf');
x0 = [5;0];%randn(2,1);
xtraj = simulate(r,[0 5],x0);
%fnplt(xtraj,1);
%v = r.constructVisualizer();
%v.playback(xtraj);

r = PlanarRigidBodyManipulator('MassSpringDamper.urdf');
x0 = [5;0];%randn(2,1);
xtraj = simulate(r,[0 5],x0);
fnplt(xtraj,1);

r = RigidBodyManipulator('MassSpringDamper.urdf');
x0 = [10;0];
run_time = 10;
xtraj = simulate(r,[0,run_time],x0);
for t=0:run_time
    expected = [(10/39)*exp(-t/2)*(sqrt(39)*sin(0.5*sqrt(39)*t)+39*cos(0.5*sqrt(39)*t));
                (-200*exp(-t/2)*sin(0.5*sqrt(39)*t))/sqrt(39)];
    result = xtraj.eval(t);
    valuecheck(result, expected, 0.01);
end

end

