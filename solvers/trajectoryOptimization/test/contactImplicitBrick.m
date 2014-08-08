function contactImplicitBrick
% tests that the contact implicit trajectory optimization can reproduce a
% simulation of the falling brick

options.terrain = RigidBodyFlatTerrain();
options.floating = true;
plant = RigidBodyManipulator(fullfile(getDrakePath,'systems','plants','test','FallingBrickContactPoints.urdf'),options);
x0 = [0;0;.8;0.1*randn(3,1);zeros(6,1)];

N=11; tf=1;

plant_ts = TimeSteppingRigidBodyManipulator(plant,tf/(N-1));
xtraj_ts = simulate(plant_ts,[0 1],x0);
v = constructVisualizer(plant_ts);
v.playback(xtraj_ts);

to_options = struct();
to_options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;
to_options.active_collision_options.terrain_only = true;

scale_sequence = [1;.001;0];

for i=1:length(scale_sequence)
  scale = scale_sequence(i);

  to_options.compl_slack = scale*.01;
  to_options.lincompl_slack = scale*.001;
  to_options.jlcompl_slack = scale*.01;
  
  prog = ContactImplicitTrajectoryOptimization(plant,N,tf,to_options);
  prog = prog.setSolverOptions('snopt','MajorIterationsLimit',200);
  prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
  prog = prog.setSolverOptions('snopt','IterationsLimit',200000);
  % prog = prog.setCheckGrad(true);
  
%   snprint('snopt.out');
  
  % initial conditions constraint
  prog = addStateConstraint(prog,ConstantConstraint(x0),1);
  
  if i == 1,
    traj_init.x = PPTrajectory(foh([0,tf],[x0,x0]));
  else
    traj_init.x = xtraj;
    traj_init.l = ltraj;
  end
  [xtraj,utraj,ltraj,~,z,F,info] = solveTraj(prog,tf,traj_init);
end

v.playback(xtraj);

% check if the two simulations did the same thing:
ts = getBreaks(xtraj_ts);
valuecheck(ts,getBreaks(xtraj));
valuecheck(xtraj.eval(ts),xtraj_ts.eval(ts),1e-2); % is there a correct tolerance here?


