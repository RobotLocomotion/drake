function contactImplicitBrick
% tests that the contact implicit trajectory optimization can reproduce a
% simulation of the falling brick

options.terrain = RigidBodyFlatTerrain();
options.floating = true;
plant = RigidBodyManipulator(fullfile(getDrakePath,'systems','plants','test','FallingBrickBetterCollisionGeometry.urdf'),options);
x0 = [0;0;.6;0.1*randn(3,1);zeros(6,1)];

plant_ts = TimeSteppingRigidBodyManipulator(plant,.1);
xtraj_ts = simulate(plant_ts,[0 1],x0);
v = constructVisualizer(plant_ts);
v.playback(xtraj_ts);

N=11; tf=1;
prog = ContactImplicitTrajectoryOptimization(plant,N,tf);

% initial conditions constraint
prog = addStateConstraint(prog,ConstantConstraint(x0),1);

traj_init.x = PPTrajectory(foh([0,tf],[x0,x0]));
[xtraj,utraj,z,F,info] = solveTraj(prog,tf,traj_init);
v.playback(xtraj);

% check if the two simulations did the same thing:
ts = getBreaks(xtraj_ts);
valuecheck(ts,getBreaks(xtraj));
valuecheck(xtraj.eval(ts),xtraj_ts.eval(ts),1e-4); % is there a correct tolerance here?


