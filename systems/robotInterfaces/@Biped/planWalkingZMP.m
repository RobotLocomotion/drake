function walking_plan_data = planWalkingZMP(obj, x0, footstep_plan)
% Construct a dynamic walking plan based on the ZMP formulation.
% @param x0 the initial robot state vector
% @param footstep_plan a FootstepPlan or DynamicFootstepPlan

path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'ZMP'));

nq = getNumPositions(obj);
q0 = x0(1:nq);

if isa(footstep_plan, 'FootstepPlan')

  footstep_plan.sanity_check();
  for j = 1:length(footstep_plan.footsteps)
    footstep_plan.footsteps(j).walking_params = applyDefaults(struct(footstep_plan.footsteps(j).walking_params),...
      obj.default_walking_params);
  end

  dynamic_footstep_plan = planZMPTraj(obj, q0, footstep_plan.footsteps);

  mu = zeros(length(footstep_plan.footsteps), 1);
  for j = 1:length(footstep_plan.footsteps)
    mu(j) = footstep_plan.footsteps(j).walking_params.mu;
  end
elseif isa(footstep_plan, 'DynamicFootstepPlan')
  dynamic_footstep_plan = footstep_plan;
  mu = 1;
else
  error('invalid plan type: %s\n', class(footstep_plan));
end

link_constraints = dynamic_footstep_plan.buildLinkConstraints();
% link_constraints(1).pelvis_reference_height
% link_constraints(2).pelvis_reference_height
warning('pelvis reference height forced to zero');
link_constraints(1).pelvis_reference_height = zeros(1, length(link_constraints(1).ts));
link_constraints(2).pelvis_reference_height = zeros(1, length(link_constraints(1).ts));

zmptraj = dynamic_footstep_plan.getZMPTraj();

zmptraj = setOutputFrame(zmptraj,desiredZMP);

kinsol = doKinematics(obj, q0);
com = getCOM(obj, kinsol);

foot_pos = forwardKin(obj, kinsol, [obj.foot_frame_id.right, obj.foot_frame_id.left], [0;0;0]);
zfeet = mean(foot_pos(3,:));

% get COM traj from desired ZMP traj
options.com0 = com(1:2);
[c,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(com(3)-zfeet,zmptraj,options);

[supports, support_times] = dynamic_footstep_plan.getSupports();
walking_plan_data = WalkingPlanData(x0, support_times, supports, link_constraints, zmptraj, V, c, comtraj, mu);
