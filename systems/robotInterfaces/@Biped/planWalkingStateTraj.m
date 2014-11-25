function [xtraj, htraj, ts] = planWalkingStateTraj(obj, walking_plan_data, xstar)
% Given the results of the ZMP tracker, find a state trajectory for the robot to execute
% its walking plan.
% @param walking_plan_data a WalkingPlanData, such as that returned by biped.planWalkingZMP()
% @param xstar the nominal robot state vector
% @retval xtraj a PPTrajectory of robot states
% @retval htraj a PPTrajectory of CoM heights
% @retval ts the time points at which the trajectory constraints were applied

if nargin < 3
  xstar = obj.loadFixedPoint();
end

nq = obj.getNumPositions();
q0 = walking_plan_data.x0(1:nq);
qstar = xstar(1:nq);

% time spacing of samples for IK
ts = 0:0.1:walking_plan_data.comtraj.tspan(end);
if length(ts)>300 % limit number of IK samples to something reasonable
  ts = linspace(0,walking_plan_data.comtraj.tspan(end),300);
end

% We no longer compute a trajectory for the feet, just a sequence of poses,
% so we need to build that trajectory now.
for j = 1:length(walking_plan_data.link_constraints)
  walking_plan_data.link_constraints(j).traj = PPTrajectory(pchip(walking_plan_data.link_constraints(j).ts, walking_plan_data.link_constraints(j).poses));
end

%% create desired joint trajectory
cost = Point(obj.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_bkz = 10;
cost.back_bky = 100;
cost.back_bkx = 100;
cost = double(cost);
ikoptions = IKoptions(obj);
ikoptions = ikoptions.setQ(diag(cost(1:obj.getNumPositions)));

htraj = [];
full_IK_calls = 0;
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    ik_args = {};
    for j = 1:length(walking_plan_data.link_constraints)
      body_ind = walking_plan_data.link_constraints(j).link_ndx;
      if ~isempty(walking_plan_data.link_constraints(j).traj)
        min_pos = walking_plan_data.link_constraints(j).traj.eval(t);
        max_pos = min_pos;
      else
        min_pos = walking_plan_data.link_constraints(j).min_traj.eval(t);
        max_pos = walking_plan_data.link_constraints(j).max_traj.eval(t);
      end
      ik_args = [ik_args,{constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,...
          obj,body_ind, [0;0;0],min_pos(1:3),max_pos(1:3)),...
          constructRigidBodyConstraint(RigidBodyConstraint.WorldEulerConstraintType,true,obj,body_ind,min_pos(4:6),max_pos(4:6))}];
    end
    kc_com = constructRigidBodyConstraint(RigidBodyConstraint.WorldCoMConstraintType,true,obj.getMexModelPtr,[walking_plan_data.comtraj.eval(t);nan],[walking_plan_data.comtraj.eval(t);nan]);
    [q(:,i),info] = approximateIKmex(obj.getMexModelPtr,q(:,i-1),qstar,kc_com,ik_args{:},ikoptions.mex_ptr);
    if info
      full_IK_calls = full_IK_calls + 1;
      q(:,i) = inverseKin(obj,q(:,i-1),qstar,kc_com,ik_args{:},ikoptions);
    end

  else
    q = q0;
  end
  com = getCOM(obj,q(:,i));
  htraj = [htraj com(3)];
end

if full_IK_calls > 0
  fprintf(1, 'Called inverseKin due to failure of approximateIK %d times.\n', full_IK_calls);
end
% qtraj = PPTrajectory(spline(ts,q));
htraj = PPTrajectory(spline(ts,htraj));
x = zeros(getNumStates(obj),length(ts));
x(1:getNumPositions(obj),:) = q;
xtraj = PPTrajectory(spline(ts, x));
xtraj = xtraj.setOutputFrame(obj.getStateFrame());

end


