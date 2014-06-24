function [xtraj,utraj,info,r,v,planner] = testKinematicPlanner(visualize,nT)
  if nargin < 1
    visualize = false;
  end
  if nargin < 2
    nT = 10;
  end
  options.floating = false;
  urdf = [getDrakePath(),'/examples/SimpleDoublePendulum/SimpleDoublePendulum.urdf'];
  r = RigidBodyManipulator(urdf,options);
  if visualize
    checkDependency('lcmgl');
    v = r.constructVisualizer();
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), ...
      'testSimpleDynamicsFullKinematicsPlanner');
  else
    v = [];
  end
  tspan = [0,2];
  t_seed = linspace(tspan(1),tspan(2),nT);
  nX = r.getNumStates();
  nQ = r.getNumPositions();
  nU = r.getNumInputs();
  x0 = zeros(2*nQ,1);

  target = [1.5;0;1];
  end_effector = [0; 0; -1];

  if visualize
    lcmgl.glDrawAxes();
    % Draw target
    lcmgl.glColor4f(0,1,0,0.5);
    lcmgl.sphere(target,0.2,20,20);
    lcmgl.switchBuffers();
  end

  lb = target; lb(2) = NaN;
  ub = target; ub(2) = NaN;
  grasp = WorldPositionConstraint(r,3,end_effector,lb,ub,tspan([2,2]));

  q_nom_traj = PPTrajectory(foh(t_seed,pi*(2*rand(nQ,nT)-1)));
  x_nom_traj = PPTrajectory(foh(t_seed,[q_nom_traj.eval(t_seed);zeros(nQ,nT)]));
  x_nom_traj = x_nom_traj.setOutputFrame(r.getStateFrame());

  %if visualize
    %v.playback(x_nom_traj);
  %end

  traj_init.x = x_nom_traj;
  traj_init.u = PPTrajectory(foh(t_seed,rand(nU,nT)));
  tf_range = t_seed(end)*[1,2];
  planner = KinematicPlanner(r,nT,tf_range);
  planner = planner.addRunningCost(NonlinearConstraint(0,0,nX+nU+1,@squaredEffort));
  planner = planner.setFixInitialState(true,x0);
  planner = planner.addManagedStateConstraint(ConstraintManager([],NonlinearConstraint(0,0,nX,@velocityPenalty)),{1,nT});
  planner = planner.addRigidBodyConstraint(grasp,{nT});
  [xtraj,utraj,z,F,info] = solveTraj(planner,t_seed,traj_init);
  if visualize
    v.playback(xtraj);
    t_data = xtraj.getBreaks();
    figure(77);
    subplot(2,1,1);
    plot(t_data,eval(xtraj,t_data));
    xlabel('Time (s)');
    ylabel('State');
    subplot(2,1,2)
    plot(t_data,eval(utraj,t_data));
    xlabel('Time (s)');
    ylabel('Input');
  end
end

function [f,df] = velocityPenalty(x)
  f = sum(x(3:4).^2);
  df = [zeros(1,2), 2*x(3:4)'];
end
