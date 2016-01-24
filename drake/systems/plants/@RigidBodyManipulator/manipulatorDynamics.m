function [H,C,B,dH,dC,dB] = manipulatorDynamics(obj, q, v, use_mex)
% manipulatorDynamics  Calculate coefficients of equation of motion.
% [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,v,use_mex) calculates the
% coefficients of the joint-space equation of motion,
% H(q)*vd+C(q,v,f_ext)=B(q,v)*tau, where q, v and vd are the joint
% position, velocity and acceleration vectors, H is the joint-space inertia
% matrix, C is the vector of gravity, external-force and velocity-product
% terms, and tau is the joint force vector.
%
% Algorithm: recursive Newton-Euler for C, and Composite-Rigid-Body for H.
%
% note that you can also get C(q, v) * v + G(q) separately, because
% C = G when v = 0

checkDirty(obj);

if (nargin<4) use_mex = true; end

compute_gradients = nargout > 3;

% if (nargin<4) use_mex = true; end
if compute_gradients
  [f_ext, B, df_ext, dB] = computeExternalForcesAndInputMatrix(obj, q, v);
else
  [f_ext, B] = computeExternalForcesAndInputMatrix(obj, q, v);
end

options.use_mex = use_mex;
options.compute_gradients = compute_gradients;
options.compute_JdotV = true;
kinsol = doKinematics(obj, q, v, options);

a_grav = [zeros(3, 1); obj.gravity];
if (use_mex && obj.mex_model_ptr~=0 && kinsol.mex)
  if isnumeric(f_ext)
    f_ext = full(f_ext); % makes the mex implementation simpler (for now)
  end
  if compute_gradients
    f_ext = TaylorVar(f_ext, {df_ext});
  end

  H = massMatrixmex(obj.mex_model_ptr, kinsol.mex_ptr);
  C = dynamicsBiasTermmex(obj.mex_model_ptr, kinsol.mex_ptr, f_ext);
  if kinsol.has_gradients
    [H, dH] = eval(H);
    [C, dC] = eval(C);
  end
else
  if compute_gradients
    [inertias_world, dinertias_world] = inertiasInWorldFrame(obj, kinsol);
    [crbs, dcrbs] = compositeRigidBodyInertias(obj, inertias_world, dinertias_world);
    [H, dH] = computeMassMatrix(obj, kinsol, crbs, dcrbs);
    [C, dC] = computeBiasTerm(obj, kinsol, a_grav, inertias_world, f_ext, dinertias_world, df_ext);
  else
    inertias_world = inertiasInWorldFrame(obj, kinsol);
    crbs = compositeRigidBodyInertias(obj, inertias_world);
    H = computeMassMatrix(obj, kinsol, crbs);
    C = computeBiasTerm(obj, kinsol, a_grav, inertias_world, f_ext);
  end
end

end

function [f_ext, B, df_ext, dB] = computeExternalForcesAndInputMatrix(obj, q, v)
compute_gradients = nargout > 2;

nq = length(q);
nv = length(v);

B = obj.B;
if compute_gradients
  dB = zeros(numel(B), nq + nv);
end

if ~isempty(obj.force)
  NB = obj.getNumBodies();
  f_ext = q(1) * zeros(6,NB);
  if compute_gradients
    df_ext = zeros(numel(f_ext), nq + nv);
  end
  for i=1:length(obj.force)
    % compute spatial force should return something that is the same length
    % as the number of bodies in the manipulator
    if (obj.force{i}.direct_feedthrough_flag)
      if compute_gradients
        [force,B_force,dforce,dB_force] = computeSpatialForce(obj.force{i},obj,q,v);
        dB = dB + dB_force;
      else
        [force,B_force] = computeSpatialForce(obj.force{i},obj,q,v);
      end
      B = B+B_force;
    else
      if compute_gradients
        [force,dforce] = computeSpatialForce(obj.force{i},obj,q,v);
        dforce = reshape(dforce,numel(force),[]);
      else
        force = computeSpatialForce(obj.force{i},obj,q,v);
      end
    end
    f_ext = f_ext + force;
    if compute_gradients
      df_ext = df_ext + dforce;
    end
  end
else
  if isempty(q)
    f_ext = double.empty(6, 0);
  else
    f_ext = q(1) * double.empty(6, 0);
  end
  if compute_gradients
    df_ext = [];
  end
end
end

function [H, dH] = computeMassMatrix(manipulator, kinsol, crbs, dcrbs)
compute_gradient = nargout > 1;

% world frame implementation
NB = length(manipulator.body);
nv = manipulator.getNumVelocities();
H = zeros(nv, nv) * kinsol.q(1); % minor adjustment to make TaylorVar work better.

if compute_gradient
  nq = manipulator.getNumPositions();
  dHdq = zeros(numel(H), nq) * kinsol.q(1);
end

for i = 2 : NB
  Ic = crbs{i};
  Si = kinsol.J{i};
  i_indices = manipulator.body(i).velocity_num;
  F = Ic * Si;
  Hii = Si' * F;
  H(i_indices, i_indices) = Hii;
  
  if compute_gradient
    dIc = dcrbs{i};
    dSi = kinsol.dJdq{i};
    dF = matGradMultMat(Ic, Si, dIc, dSi);
    dHii = matGradMultMat(Si', F, transposeGrad(dSi, size(Si)), dF);
    dHdq = setSubMatrixGradient(dHdq, dHii, i_indices, i_indices, size(H));
  end
  
  j = i;
  while manipulator.body(j).parent ~= 1
    j = manipulator.body(j).parent;
    body_j = manipulator.body(j);
    j_indices = body_j.velocity_num;
    Sj = kinsol.J{j};
    Hji = Sj' * F;
    H(j_indices, i_indices) = Hji;
    H(i_indices, j_indices) = Hji';
    
    if compute_gradient
      dSj = kinsol.dJdq{j};
      dHji = matGradMultMat(Sj', F, transposeGrad(dSj, size(Sj)), dF);
      dHdq = setSubMatrixGradient(dHdq, dHji, j_indices, i_indices, size(H));
      dHdq = setSubMatrixGradient(dHdq, transposeGrad(dHji, size(Hji)), i_indices, j_indices, size(H)); % dHdq at this point
    end
  end
end
if compute_gradient
  dHdv = zeros(numel(H), nv);
  dH = [dHdq, dHdv];
end
end

function [C, dC] = computeBiasTerm(manipulator, kinsol, gravitational_accel, inertias_world, f_ext, dinertias_world, df_ext)
compute_gradient = nargout > 1;

nBodies = length(manipulator.body);
world = 1;
twist_size = 6;

nq = manipulator.getNumPositions();
nv = manipulator.getNumVelocities();

% as if we're standing in an elevator that's accelerating upwards:
root_accel = -gravitational_accel;
JdotV = kinsol.JdotV;
net_wrenches = cell(nBodies, 1);
net_wrenches{1} = zeros(twist_size, 1);

if compute_gradient
  dJdotV = kinsol.dJdotVdq;
  dnet_wrenches = cell(nBodies, 1);
  dnet_wrenches{1} = zeros(twist_size, nq);
  
  dnet_wrenchesdv = cell(nBodies, 1);
  dnet_wrenchesdv{1} = zeros(twist_size, nv);
end

has_f_ext = ~isempty(f_ext) && numel(f_ext) > 0; % need both to cover both TaylorVar and TrigPoly

for i = 2 : nBodies
  twist = kinsol.twists{i};
  spatial_accel = root_accel + JdotV{i};
  
  if compute_gradient
    dtwist = kinsol.dtwistsdq{i};
    dspatial_accel = dJdotV{i};
    
    dtwistdv = zeros(twist_size, nv);
    [J, v_indices] = geometricJacobian(manipulator, kinsol, world, i, world);
    dtwistdv(:, v_indices) = J;
    dspatial_acceldv = kinsol.dJdotVidv{i};
  end
  
  I = inertias_world{i};
  I_times_twist = I * twist;
  net_wrenches{i} = I * spatial_accel + crf(twist) * I_times_twist;
  
  if compute_gradient
    dI = dinertias_world{i};
    dI_times_twist = I * dtwist + matGradMult(dI, twist);
    dnet_wrenches{i} = ...
      I * dspatial_accel + matGradMult(dI, spatial_accel) ...
      + dcrf(twist, I_times_twist, dtwist, dI_times_twist);
    
    dI_times_twistdv = I * dtwistdv;
    dnet_wrenchesdv{i} = ...
      I * dspatial_acceldv ...
      + dcrf(twist, I_times_twist, dtwistdv, dI_times_twistdv);
  end
  
  if has_f_ext
    external_wrench = f_ext(:, i);
    
    % external wrenches are expressed in body frame. Transform from body to world:
    AdT_world_to_body = transformAdjoint(homogTransInv(kinsol.T{i}));
    external_wrench = AdT_world_to_body' * external_wrench;
    net_wrenches{i} = net_wrenches{i} - external_wrench;
    
    if compute_gradient
      dexternal_wrench = getSubMatrixGradient(df_ext,1:twist_size,i,size(f_ext),1:nq);
      dexternal_wrench = dTransformSpatialForce(kinsol.T{i}, external_wrench, kinsol.dTdq{i}, dexternal_wrench);
      dnet_wrenches{i} = dnet_wrenches{i} - dexternal_wrench;
      
      dexternal_wrenchdv = getSubMatrixGradient(df_ext,1:twist_size,i,size(f_ext),nq+(1:nv));
      dexternal_wrenchdv = AdT_world_to_body' * dexternal_wrenchdv;
      dnet_wrenchesdv{i} = dnet_wrenchesdv{i} - dexternal_wrenchdv;
    end
  end
end

C = zeros(nv, 1) * kinsol.q(1);

if compute_gradient
  dC = zeros(nv, nq + nv);
end

for i = nBodies : -1 : 2
  body = manipulator.body(i);
  joint_wrench = net_wrenches{i};
  Ji = kinsol.J{i};
  tau = Ji' * joint_wrench;
  C(body.velocity_num) = tau;
  net_wrenches{body.parent} = net_wrenches{body.parent} + joint_wrench;
  
  if compute_gradient
    djoint_wrench = dnet_wrenches{i};
    dJi = kinsol.dJdq{i};
    dtau = Ji' * djoint_wrench + matGradMult(transposeGrad(dJi, size(Ji)), joint_wrench);
    dC = setSubMatrixGradient(dC, dtau, body.velocity_num, 1, size(C), 1:nq);
    dnet_wrenches{body.parent} = dnet_wrenches{body.parent} + djoint_wrench;
    
    djoint_wrenchdv = dnet_wrenchesdv{i};
    dtaudv = Ji' * djoint_wrenchdv;
    dC = setSubMatrixGradient(dC, dtaudv, body.velocity_num, 1, size(C), nq + (1:nv));
    dnet_wrenchesdv{body.parent} = dnet_wrenchesdv{body.parent} + djoint_wrenchdv;
  end
end

if compute_gradient
  [tau_friction, dtau_frictiondv] = computeFrictionForce(manipulator, kinsol.v);
  dC(:, nq + (1:nv)) = dC(:, nq + (1:nv)) + dtau_frictiondv;
else
  tau_friction = computeFrictionForce(manipulator, kinsol.v);
end
C = C + tau_friction;
end
