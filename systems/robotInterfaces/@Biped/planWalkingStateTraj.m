function [xtraj, htraj, ts] = planWalkingStateTraj(obj, walking_plan_data, xstar)

if nargin < 3
  xstar = obj.loadFixedPoint();
end

nq = obj.getNumDOF();
q0 = walking_plan_data.x0(1:nq);
qstar = xstar(1:nq);

% time spacing of samples for IK
ts = 0:0.1:walking_plan_data.comtraj.tspan(end);
if length(ts)>300 % limit number of IK samples to something reasonable
  ts = linspace(0,walking_plan_data.comtraj.tspan(end),300);
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
ikoptions = ikoptions.setQ(diag(cost(1:obj.getNumDOF)));

% Mex constraints don't support Drake Frames attached to URDFs, so we need to look up each frame's parent body and associated transform. We'll do this once now for efficiency's sake.
parent_bodies = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
parent_T = containers.Map('KeyType', 'int32', 'ValueType', 'any');
body_ids = walking_plan_data.bodytraj.keys();
for j = 1:length(body_ids)
  if body_ids{j} < 0
    % Then this is a frame ID, so we should find its parent body and transform
    body_ind = obj.getFrame(body_ids{j}).body_ind;
    T = inv(obj.getFrame(body_ids{j}).T);
    parent_bodies(body_ids{j}) = body_ind;
    parent_T(body_ids{j}) = T;
  else
    parent_bodies(body_ids{j}) = body_ids{j};
    parent_T(body_ids{j}) = eye(4);
  end
end

htraj = [];
full_IK_calls = 0;
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    ik_args = {};
    body_ids = walking_plan_data.bodytraj.keys();
    for j = 1:length(body_ids)
      body_ind = parent_bodies(body_ids{j});
      xyzrpy = walking_plan_data.bodytraj(body_ids{j}).eval(t);
      T = parent_T(body_ids{j});
      xyz1 = T * [xyzrpy(1:3); 1];
      rpy = rotmat2rpy(rpy2rotmat(xyzrpy(4:6)) * T(1:3,1:3));
      ik_args = [ik_args,{constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,...
          obj,body_ind, [0;0;0],xyz1(1:3),xyz1(1:3)),...
          constructRigidBodyConstraint(RigidBodyConstraint.WorldEulerConstraintType,true,obj,body_ind,rpy,rpy)}];
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
x(1:getNumDOF(obj),:) = q;
xtraj = PPTrajectory(spline(ts, x));
xtraj = xtraj.setOutputFrame(obj.getStateFrame());

