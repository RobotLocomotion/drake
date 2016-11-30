function contactImplicitBrick(visualize,position_tol,velocity_tol)
% tests that the contact implicit trajectory optimization can reproduce a
% simulation of the falling brick
rng(0)
if nargin < 1, visualize = false; end
if nargin < 2, position_tol = 1.5e-2; end
if nargin < 3, velocity_tol = 1e-1; end

options.terrain = RigidBodyFlatTerrain();
options.floating = true;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
plant = RigidBodyManipulator(fullfile(getDrakePath,'matlab','systems','plants','test','FallingBrickContactPoints.urdf'),options);
warning(w);
x0 = [0;0;.8;0.05*randn(3,1);zeros(6,1)];

N=5; tf=.5;

plant_ts = TimeSteppingRigidBodyManipulator(plant,tf/(N-1));
w = warning('off','Drake:TimeSteppingRigidBodyManipulator:ResolvingLCP');
xtraj_ts = simulate(plant_ts,[0 tf],x0);
x0 = xtraj_ts.eval(0);
warning(w);
if visualize
  v = constructVisualizer(plant_ts);
  v.playback(xtraj_ts);
end

options = struct();
options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;

scale_sequence = [1;.001;0];

for i=1:length(scale_sequence)
  scale = scale_sequence(i);

  options.compl_slack = scale*.01;
  options.lincompl_slack = scale*.001;
  options.jlcompl_slack = scale*.01;
  
  prog = ContactImplicitTrajectoryOptimization(plant,N,tf,options);
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

if visualize
  v.playback(xtraj);
end

% check if the two simulations did the same thing:
ts = getBreaks(xtraj_ts);
valuecheck(ts,getBreaks(xtraj));
xtraj_data = xtraj.eval(ts); 
xtraj_ts_data = xtraj_ts.eval(ts);
nq = plant.getNumPositions();
nv = plant.getNumVelocities();
valuecheck(xtraj_data(1:nq,:),xtraj_ts_data(1:nq,:),position_tol); % is there a correct tolerance here?
valuecheck(xtraj_data(nq+(1:nv),:),xtraj_ts_data(nq+(1:nv),:),velocity_tol); % is there a correct tolerance here?

% TIMEOUT 750
