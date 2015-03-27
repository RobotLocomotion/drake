function [y, qdd, info_fqp, supp,...
          alpha, Hqp, fqp, Aeq, beq, Ain, bin,lb,ub] = setupAndSolveQP(obj, t, q, qd, qp_input, contact_sensor)
% Construct the quadratic control problem for Atlas and solve it, returning
% torques and accelerations. This file currently represents most of the inner
% loop of atlasControllers.AtlasQPController.output(), but extracted out into
% its own function. It also mirrors the functionality of instantaneousQPControllermex.cpp
% @param t current time (s)
% @param q, qd current robot pose and velocity
% @param qp_input QPInputConstantHeight describing the current plan state
% @param contact_sensor num_bodies vector indicating whether the given body has detected contact force

params = obj.param_sets.(qp_input.param_set_name);
r = obj.robot;
ctrl_data = obj.controller_data;

% Compute desired body accelerations and costs
all_bodies_vdot = struct('body_id', cell(1,length(qp_input.body_motion_data)),...
                         'body_vdot', cell(1,length(qp_input.body_motion_data)),...
                         'params', cell(1,length(qp_input.body_motion_data)));

for j = 1:length(qp_input.body_motion_data)
  body_id = qp_input.body_motion_data(j).body_id;
  t_spline = max(qp_input.body_motion_data(j).ts(1), min(qp_input.body_motion_data(j).ts(2), t));
  [body_des, body_v_des, body_vdot_des] = evalCubicSplineSegment(t_spline - qp_input.body_motion_data(j).ts(1), qp_input.body_motion_data(j).coefs);
  body_vdot = atlasControllers.bodyMotionControl(r, [q; qd], body_id,...
                  body_des, body_v_des, body_vdot_des, params.body_motion(body_id));
  body_vdot_mex = instantaneousBodyMotionControlmex(r.getMexModelPtr(), [q; qd], body_id,...
                  body_des, body_v_des, body_vdot_des, params.body_motion(body_id));
  valuecheck(body_vdot, body_vdot_mex, 1e-8);
  all_bodies_vdot(j).body_id = body_id; 
  all_bodies_vdot(j).body_vdot = body_vdot;
  all_bodies_vdot(j).params = params.body_motion(body_id);
end

% Find the active set of supports
mask = getActiveSupportsmex(obj.data_mex_ptr, [q; qd], qp_input.support_data, contact_sensor, params.contact_threshold, obj.default_terrain_height);
supp = qp_input.support_data(logical(mask));

% Run PID on the desired configuration
qddot_des = obj.wholeBodyPID(t, q, qd, qp_input.whole_body_data.q_des, params.whole_body);

% Find the constrained joints
condof = qp_input.whole_body_data.constrained_dofs;

% Extract the coefficient of friction
if isempty(qp_input.support_data)
  mu = 1;
else
  mu = qp_input.support_data(1).mu;
  for j = 2:length(qp_input.support_data)
    if qp_input.support_data(j).mu ~= mu
      warning('Drake:MultipleMuNotSupported', 'We do not currently support different mu values for each support');
    end
  end
end
if (mu ~= 1)
  r.warning_manager.warnOnce('Drake:MuAssumedToBe1', 'mu is assumed to be one in constactConstraints.m');
end

% Construct the linear system
zmp_data = qp_input.zmp_data;
B_ls = zmp_data.B;
C_ls = zmp_data.C;
D_ls = zmp_data.D;
Qy = zmp_data.Qy;
R_ls = zmp_data.R;
S = zmp_data.S;
s1 = zmp_data.s1;
x0 = zmp_data.x0;
y0 = zmp_data.y0;
u0 = zmp_data.u0;

% get the whole-body weights
w_qdd = params.whole_body.w_qdd;

% Apply PD overrides for particular joints
for j = 1:length(qp_input.joint_pd_override)
  override = qp_input.joint_pd_override(j);
  ind = override.position_ind;
  err_q = override.qi_des - q(ind);
  err_qd = override.qdi_des - qd(ind);
  qddot_des(ind) = override.kp * err_q + override.kd * err_qd;
  w_qdd(ind) = override.weight;
end

nq = r.getNumPositions();                                           
kinsol = doKinematics(r,q,false,true,qd);

R_DQyD_ls = R_ls + D_ls'*Qy*D_ls;

for j = 1:length(supp)
  supp(j).num_contact_pts = size(supp(j).contact_pts, 2);
end

dim = 3; % 3D
nd = 4; % for friction cone approx, hard coded for now
float_idx = 1:6; % indices for floating base dofs
act_idx = 7:nq; % indices for actuated dofs

[H,C,B] = manipulatorDynamics(r,q,qd);

H_float = H(float_idx,:);
C_float = C(float_idx);

H_act = H(act_idx,:);
C_act = C(act_idx);
B_act = B(act_idx,:);

[xcom,Jcom] = getCOM(r,kinsol);
% lcmgl = LCMGLClient('actual com');
% lcmgl.glColor3f(0.8, 1, 0);
% lcmgl.sphere([xcom(1:2); 0], 0.02, 20, 20);
% lcmgl.switchBuffers();


include_angular_momentum = any(any(params.W_kdot));

if include_angular_momentum
  [A,Adot] = getCMM(r,kinsol,qd);
end

Jcomdot = forwardJacDot(r,kinsol,0);
if length(x0)==4
 Jcom = Jcom(1:2,:); % only need COM x-y
 Jcomdot = Jcomdot(1:2,:);
end

if ~isempty(supp)
  nc = sum([supp.num_contact_pts]);
  Dbar = [];
  for j=1:length(supp)

    % The new plan and control tools all work with contact points directly,
    % and the c++ functions support this, but contactConstraintsBV.m expects
    % only collision group names, so we have to back out the names of the
    % groups corresponding to the points we're using. This is obviously rather
    % slow, but it's just for the Matlab-only controller. -rdeits
    body = obj.robot.getBody(supp(j).body_id);
    collision_groups = body.collision_geometry_group_names;
    group_mask = false(size(collision_groups));
    for k = 1:length(collision_groups)
      group_pts = body.getTerrainContactPoints(collision_groups{k});
      any_in_group = false;
      all_in_group = true;
      for l = 1:size(group_pts, 2)
        if any(all(abs(bsxfun(@minus, group_pts(:,l), supp(j).contact_pts)) < 1e-3, 1))
          any_in_group = true;
        else
          all_in_group = false;
        end
      end
      if any_in_group && all_in_group
        group_mask(k) = true;
      elseif any_in_group && ~all_in_group
        error('I got some contact points belonging to collision group: %s, but not all of the points in that group. That will work in the c++ controller, but not in the matlab-only controller');
      end
    end
    active_collision_groups = collision_groups(group_mask);

    [~,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',~obj.use_bullet,...
      'body_idx',[1,supp(j).body_id], 'collision_groups', {active_collision_groups}));
    Dbar = [Dbar, vertcat(JB{:})']; % because contact constraints seems to ignore the collision_groups option
  end

  Dbar_float = Dbar(float_idx,:);
  Dbar_act = Dbar(act_idx,:);

  terrain_pts = struct('idx', {supp.body_id},...
                       'pts', {supp.contact_pts});
  [~,Jp,Jpdot] = terrainContactPositions(r,kinsol,terrain_pts,true);
  Jp = sparse(Jp);
  Jpdot = sparse(Jpdot);

  if length(x0)==4
    xlimp = [xcom(1:2); Jcom*qd]; % state of LIP model
  else
    xlimp = [xcom;Jcom*qd];
  end
  x_bar = xlimp - x0;
else
  nc = 0;
end
neps = nc*dim;


%----------------------------------------------------------------------
% Build handy index matrices ------------------------------------------

nf = nc*nd; % number of contact force variables
nparams = nq+nf+neps;
Iqdd = zeros(nq,nparams); Iqdd(:,1:nq) = eye(nq);
Ibeta = zeros(nf,nparams); Ibeta(:,nq+(1:nf)) = eye(nf);
Ieps = zeros(neps,nparams);
Ieps(:,nq+nf+(1:neps)) = eye(neps);


%----------------------------------------------------------------------
% Set up problem constraints ------------------------------------------

qdd_lb = -inf(1, nq);
qdd_ub = inf(1, nq);
lb = [qdd_lb zeros(1,nf)   -params.slack_limit*ones(1,neps)]'; % qddot/contact forces/slack vars
ub = [qdd_ub 1e3*ones(1,nf) params.slack_limit*ones(1,neps)]';

Aeq_ = cell(1,length(all_bodies_vdot)+3+1);
beq_ = cell(1,5);
Ain_ = cell(1,2+length(all_bodies_vdot)*2);
bin_ = cell(1,2+length(all_bodies_vdot)*2);

% constrained dynamics
if nc>0
  Aeq_{1} = H_float*Iqdd - Dbar_float*Ibeta;
else
  Aeq_{1} = H_float*Iqdd;
end
beq_{1} = -C_float;

% input saturation constraints
% u=B_act'*(H_act*qdd + C_act - Jz_act'*z - Dbar_act*beta)

if nc>0
  Ain_{1} = B_act'*(H_act*Iqdd - Dbar_act*Ibeta);
else
  Ain_{1} = B_act'*H_act*Iqdd;
end
bin_{1} = -B_act'*C_act + r.umax;
Ain_{2} = -Ain_{1};
bin_{2} = B_act'*C_act - r.umin;

constraint_index = 3;
for ii=1:length(all_bodies_vdot)
  body_id = all_bodies_vdot(ii).body_id;
  [~,Jb] = forwardKin(r,kinsol,body_id,[0;0;0],1);
  Jbdot = forwardJacDot(r,kinsol,body_id,[0;0;0],1);
  Ain_{constraint_index} = Jb*Iqdd;
  bin_{constraint_index} = -Jbdot*qd + params.body_motion(body_id).accel_bounds.max;
  constraint_index = constraint_index + 1;
  Ain_{constraint_index} = -Jb*Iqdd;
  bin_{constraint_index} = Jbdot*qd - params.body_motion(body_id).accel_bounds.min;
  constraint_index = constraint_index + 1;
end

if nc > 0
  % relative acceleration constraint
  Aeq_{2} = Jp*Iqdd + Ieps;
  beq_{2} = -Jpdot*qd - params.Kp_accel*Jp*qd;
end

eq_count=3;

for ii=1:length(all_bodies_vdot)
  body_id = all_bodies_vdot(ii).body_id;
  if all_bodies_vdot(ii).params.weight < 0
    body_vdot = all_bodies_vdot(ii).body_vdot;
    if ~any([supp.body_id]==body_id)
      [~,J] = forwardKin(r,kinsol,body_id,[0;0;0],1);
      Jdot = forwardJacDot(r,kinsol,body_id,[0;0;0],1);
      cidx = ~isnan(body_vdot);
      Aeq_{eq_count} = J(cidx,:)*Iqdd;
      beq_{eq_count} = -Jdot(cidx,:)*qd + body_vdot(cidx);
      eq_count = eq_count+1;
    end
  end
end

if ~isempty(condof)
  % add joint acceleration constraints
  conmap = zeros(length(condof),nq);
  conmap(:,condof) = eye(length(condof));
  Aeq_{eq_count} = conmap*Iqdd;
  beq_{eq_count} = qddot_des(condof);
end

% linear equality constraints: Aeq*alpha = beq
Aeq = sparse(vertcat(Aeq_{:}));
beq = vertcat(beq_{:});

% linear inequality constraints: Ain*alpha <= bin
Ain = sparse(vertcat(Ain_{:}));
bin = vertcat(bin_{:});
Ain = Ain(bin~=inf,:);
bin = bin(bin~=inf);

if include_angular_momentum
  Ak = A(1:3,:);
  Akdot = Adot(1:3,:);
  k=Ak*qd;
  kdot_des = -params.Kp_ang*k;
end

%----------------------------------------------------------------------
% QP cost function ----------------------------------------------------
%
% min: ybar*Qy*ybar + ubar*R*ubar + (2*S*xbar + s1)*(A*x + B*u) +
% w_qdd*quad(qddot_ref - qdd) + w_eps*quad(epsilon) +
% w_grf*quad(beta) + quad(kdot_des - (A*qdd + Adot*qd))
if nc > 0
  Hqp = Iqdd'*Jcom'*R_DQyD_ls*Jcom*Iqdd;
  Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + diag(w_qdd);
  if include_angular_momentum
    Hqp = Hqp + Iqdd'*Ak'*params.W_kdot*Ak*Iqdd;
  end

  fqp = xlimp'*C_ls'*Qy*D_ls*Jcom*Iqdd;
  fqp = fqp + qd'*Jcomdot'*R_DQyD_ls*Jcom*Iqdd;
  fqp = fqp + (x_bar'*S + 0.5*s1')*B_ls*Jcom*Iqdd;
  fqp = fqp - u0'*R_ls*Jcom*Iqdd;
  fqp = fqp - y0'*Qy*D_ls*Jcom*Iqdd;
  fqp = fqp - (w_qdd.*qddot_des)'*Iqdd;
  if include_angular_momentum
    fqp = fqp + qd'*Akdot'*params.W_kdot*Ak*Iqdd;
    fqp = fqp - kdot_des'*params.W_kdot*Ak*Iqdd;
  end

  Hqp(nq+(1:nf),nq+(1:nf)) = params.w_grf*eye(nf);
  Hqp(nparams-neps+1:end,nparams-neps+1:end) = params.w_slack*eye(neps);
else
  Hqp = Iqdd'*Iqdd;
  fqp = -qddot_des'*Iqdd;
end

for ii=1:length(all_bodies_vdot)
  body_id = all_bodies_vdot(ii).body_id;
  w = all_bodies_vdot(ii).params.weight;
  if w>0
    body_vdot = all_bodies_vdot(ii).body_vdot;
    if ~any([supp.body_id]==body_id)
      [~,J] = forwardKin(r,kinsol,body_id,[0;0;0],1);
      Jdot = forwardJacDot(r,kinsol,body_id,[0;0;0],1);
      cidx = ~isnan(body_vdot);
      Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + w*J(cidx,:)'*J(cidx,:);
      fqp = fqp + w*(qd'*Jdot(cidx,:)'- body_vdot(cidx)')*J(cidx,:)*Iqdd;
    end
  end
end

%----------------------------------------------------------------------
% Solve QP ------------------------------------------------------------

REG = 1e-8;

IR = eye(nparams);
lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
Ain_fqp = full([Ain; -IR(lbind,:); IR(ubind,:)]);
bin_fqp = [bin; -lb(lbind); ub(ubind)];

% call fastQPmex first
QblkDiag = {Hqp(1:nq,1:nq) + REG*eye(nq), ...
            params.w_grf*ones(nf,1) + REG*ones(nf,1), ...
            params.w_slack*ones(neps,1) + REG*ones(neps,1)};
Aeq_fqp = full(Aeq);

if (nc ~= ctrl_data.num_active_contact_pts) 
  % Contact state has changed, so our active set is invalid
  ctrl_data.qp_active_set = [];
end
ctrl_data.num_active_contact_pts = nc;

% NOTE: model.obj is 2* f for fastQP!!!
[alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq,ctrl_data.qp_active_set);

if info_fqp<0
  % then call gurobi
  % disp('QPController: failed over to gurobi');
  model.Q = sparse(Hqp + REG*eye(nparams));
  model.A = [Aeq; Ain];
  model.rhs = [beq; bin];
  eq_array = repmat('=',length(beq),1); % so we can avoid using repmat in the loop
  ineq_array = repmat('<',length(bin),1); % so we can avoid using repmat in the loop
  model.sense = [eq_array; ineq_array];
  model.lb = lb;
  model.ub = ub;

  model.obj = fqp;
  if obj.gurobi_options.method==2
    % see drake/algorithms/QuadraticProgram.m solveWGUROBI
    model.Q = .5*model.Q;
  end

  if (any(any(isnan(model.Q))) || any(isnan(model.obj)) || any(any(isnan(model.A))) || any(isnan(model.rhs)) || any(isnan(model.lb)) || any(isnan(model.ub)))
    keyboard;
  end

%         qp_tic = tic;
  result = gurobi(model,obj.gurobi_options);
%         qp_toc = toc(qp_tic);
%         fprintf('QP solve: %2.4f\n',qp_toc);

  alpha = result.x;
end

qp_active_set = find(abs(Ain_fqp*alpha - bin_fqp)<1e-6);
ctrl_data.qp_active_set = qp_active_set;

%----------------------------------------------------------------------
% Solve for inputs ----------------------------------------------------

qdd = alpha(1:nq);
if nc>0
  beta = alpha(nq+(1:nf));
  u = B_act'*(H_act*qdd + C_act - Dbar_act*beta);
else
  u = B_act'*(H_act*qdd + C_act);
end
y = u;
