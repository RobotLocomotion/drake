function walking_plan_data = planWalking(obj, x0, footstep_plan)

footstep_plan.sanity_check();
for j = 1:length(footstep_plan.footsteps)
  footstep_plan.footsteps(j).walking_params = applyDefaults(footstep_plan.footsteps(j).walking_params,...
    obj.default_walking_params);
end

nq = getNumDOF(obj);
q0 = x0(1:nq);

[zmptraj,foottraj, support_times, supports] = planZMPTraj(obj, q0, footstep_plan.footsteps);
zmptraj = setOutputFrame(zmptraj,desiredZMP);

kinsol = doKinematics(obj, q0);
com = getCOM(obj, kinsol);

foot_pos = forwardKin(obj, kinsol, [obj.foot_frame_id.right, obj.foot_frame_id.left], [0;0;0]);
zfeet = mean(foot_pos(3,:));

% get COM traj from desired ZMP traj
options.com0 = com(1:2);
[c,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(com(3)-zfeet,zmptraj,options);

walking_plan_data = WalkingPlanData(support_times, supports, foottraj, zmptraj, V, c, comtraj);