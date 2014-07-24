function walking_plan_data = planWalkingZMP(obj, x0, footstep_plan)
% Construct a dynamic walking plan based on the ZMP formulation.
% @param x0 the initial robot state vector
% @param footstep_plan a FootstepPlan

footstep_plan.sanity_check();
for j = 1:length(footstep_plan.footsteps)
  footstep_plan.footsteps(j).walking_params = applyDefaults(struct(footstep_plan.footsteps(j).walking_params),...
    obj.default_walking_params);
end

nq = getNumDOF(obj);
q0 = x0(1:nq);

[zmptraj,link_constraints, support_times, supports] = planZMPTraj(obj, q0, footstep_plan.footsteps);
zmptraj = setOutputFrame(zmptraj,desiredZMP);

kinsol = doKinematics(obj, q0);
com = getCOM(obj, kinsol);

foot_pos = forwardKin(obj, kinsol, [obj.foot_frame_id.right, obj.foot_frame_id.left], [0;0;0]);
zfeet = mean(foot_pos(3,:));

% get COM traj from desired ZMP traj
options.com0 = com(1:2);
[c,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(com(3)-zfeet,zmptraj,options);

mu = zeros(length(footstep_plan.footsteps), 1);
for j = 1:length(footstep_plan.footsteps)
  mu(j) = footstep_plan.footsteps(j).walking_params.mu;
end

walking_plan_data = WalkingPlanData(x0, support_times, supports, link_constraints, zmptraj, V, c, comtraj, mu);