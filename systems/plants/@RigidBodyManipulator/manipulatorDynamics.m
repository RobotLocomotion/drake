function [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,qd,use_mex)
% note that you can also get C(q,qdot)*qdot + G(q) separately, because
% C = G when qdot=0

checkDirty(obj);

if (nargin<4) use_mex = true; end

if obj.use_new_kinsol
  v = qd; % really want v to be passed in
  compute_gradients = nargout>3;
  if compute_gradients
    [H,C,B,dH,dC,dB] = manipulatorDynamicsNew(obj,q,v,use_mex);
  else
    [H,C,B] = manipulatorDynamicsNew(obj,q,v,use_mex);
  end
else
  
  m = obj.featherstone;
  B = obj.B;
  if (nargout>3)
    dB = zeros(m.NB*obj.num_u,2*m.NB);
  end
  
  if length(obj.force)>0
    f_ext = zeros(6,m.NB);
    if (nargout>3)
      df_ext = zeros(6*m.NB,size(q,1)+size(qd,1));
    end
    for i=1:length(obj.force)
      % compute spatial force should return something that is the same length
      % as the number of bodies in the manipulator
      if (obj.force{i}.direct_feedthrough_flag)
        if (nargout>3)
          [force,B_force,dforce,dB_force] = computeSpatialForce(obj.force{i},obj,q,qd);
          dB = dB + dB_force;
        else
          [force,B_force] = computeSpatialForce(obj.force{i},obj,q,qd);
        end
        B = B+B_force;
      else
        if (nargout>3)
          [force,dforce] = computeSpatialForce(obj.force{i},obj,q,qd);
          dforce = reshape(dforce,numel(force),[]);
        else
          force = computeSpatialForce(obj.force{i},obj,q,qd);
        end
      end
      f_ext(:,m.f_ext_map_to) = f_ext(:,m.f_ext_map_to)+force(:,m.f_ext_map_from);
      if (nargout>3)
        for j=1:size(m.f_ext_map_from,2)
          i_from = m.f_ext_map_from(j);
          i_to = m.f_ext_map_to(j);
          df_ext((i_to-1)*size(f_ext,1)+1:i_to*size(f_ext,1),1:size(q,1)+size(qd,1)) = df_ext((i_to-1)*size(f_ext,1)+1:i_to*size(f_ext,1),1:size(q,1)+size(qd,1)) + dforce((i_from-1)*size(force,1)+1:i_from*size(force,1),1:size(q,1)+size(qd,1));
        end
      end
    end
  else
    f_ext=[];
    if (nargout>3)
      df_ext=[];
    end
  end
  
  if (use_mex && obj.mex_model_ptr~=0 && isnumeric(q) && isnumeric(qd))
    f_ext = full(f_ext);  % makes the mex implementation simpler (for now)
    if (nargout>3)
      df_ext = full(df_ext);
      [H,C,dH,dC] = HandCmex(obj.mex_model_ptr,q,qd,f_ext,df_ext);
      dH = [dH, zeros(m.NB*m.NB,m.NB)];
    else
      [H,C] = HandCmex(obj.mex_model_ptr,q,qd,f_ext);
    end
  else
    if (nargout>3)
      % featherstone's HandC with analytic gradients
      a_grav = [0;0;0;obj.gravity];
      
      S = cell(m.NB,1);
      Xup = cell(m.NB,1);
      
      v = cell(m.NB,1);
      avp = cell(m.NB,1);
      
      %Derivatives
      dXupdq = cell(m.NB,1);
      dvdq = cell(m.NB,1);  %dvdq{i}(:,j) is d/dq(j) v{i}
      dvdqd = cell(m.NB,1);
      davpdq = cell(m.NB,1);
      davpdqd = cell(m.NB,1);
      fvp = cell(m.NB,1);
      dfvpdq = cell(m.NB,1);
      dfvpdqd = cell(m.NB,1);
      
      
      for i = 1:m.NB
        n = m.position_num(i);
        
        dvdq{i} = zeros(6,m.NB)*q(1);
        dvdqd{i} = zeros(6,m.NB)*q(1);
        davpdq{i} = zeros(6,m.NB)*q(1);
        davpdqd{i} = zeros(6,m.NB)*q(1);
        dfvpdq{i} = zeros(6,m.NB)*q(1);
        dfvpdqd{i} = zeros(6,m.NB)*q(1);
        
        [ XJ, S{i} ] = jcalc( m.pitch(i), q(n) );
        dXJdq = djcalc(m.pitch(i), q(n));
        
        vJ = S{i}*qd(n);
        dvJdqd = S{i};
        
        Xup{i} = XJ * m.Xtree{i};
        dXupdq{i} = dXJdq * m.Xtree{i};
        
        if m.parent(i) == 0
          v{i} = vJ;
          dvdqd{i}(:,n) = dvJdqd;
          
          avp{i} = Xup{i} * -a_grav;
          davpdq{i}(:,n) = dXupdq{i} * -a_grav;
        else
          j = m.parent(i);
          
          v{i} = Xup{i}*v{j} + vJ;
          
          dvdq{i} = Xup{i}*dvdq{j};
          dvdq{i}(:,n) = dvdq{i}(:,n) + dXupdq{i}*v{j};
          
          dvdqd{i} = Xup{i}*dvdqd{j};
          dvdqd{i}(:,n) = dvdqd{i}(:,n) + dvJdqd;
          
          avp{i} = Xup{i}*avp{j} + crm(v{i})*vJ;
          
          davpdq{i} = Xup{i}*davpdq{j};
          davpdq{i}(:,n) = davpdq{i}(:,n) + dXupdq{i}*avp{j};
          for k=1:m.NB,
            davpdq{i}(:,k) = davpdq{i}(:,k) + ...
              dcrm(v{i},vJ,dvdq{i}(:,k),zeros(6,1));
          end
          
          dvJdqd_mat = zeros(6,m.NB);
          dvJdqd_mat(:,n) = dvJdqd;
          davpdqd{i} = Xup{i}*davpdqd{j} + dcrm(v{i},vJ,dvdqd{i},dvJdqd_mat);
        end
        fvp{i} = m.I{i}*avp{i} + crf(v{i})*m.I{i}*v{i};
        dfvpdq{i} = m.I{i}*davpdq{i} + dcrf(v{i},m.I{i}*v{i},dvdq{i},m.I{i}*dvdq{i});
        dfvpdqd{i} = m.I{i}*davpdqd{i} + dcrf(v{i},m.I{i}*v{i},dvdqd{i},m.I{i}*dvdqd{i});
        
        if ~isempty(f_ext)
          fvp{i} = fvp{i} - f_ext(:,i);
          dfvpdq{i} = dfvpdq{i} - df_ext((i-1)*size(f_ext,1)+1:i*size(f_ext,1),1:size(q,1));
          dfvpdqd{i} = dfvpdqd{i} - df_ext((i-1)*size(f_ext,1)+1:i*size(f_ext,1),size(q,1)+1:end);
        end
        
      end
      
      C = zeros(m.NB,1)*q(1);
      dC = zeros(m.NB,2*m.NB)*q(1);
      IC = m.I;				% composite inertia calculation
      dIC = cell(m.NB, m.NB);
      dIC = cellfun(@(a) zeros(6), dIC,'UniformOutput',false);
      
      for i = m.NB:-1:1
        n = m.position_num(i);
        C(n,1) = S{i}' * fvp{i};
        dC(n,:) = S{i}'*[dfvpdq{i} dfvpdqd{i}];
        if m.parent(i) ~= 0
          fvp{m.parent(i)} = fvp{m.parent(i)} + Xup{i}'*fvp{i};
          dfvpdq{m.parent(i)} = dfvpdq{m.parent(i)} + Xup{i}'*dfvpdq{i};
          dfvpdq{m.parent(i)}(:,n) = dfvpdq{m.parent(i)}(:,n) + dXupdq{i}'*fvp{i};
          dfvpdqd{m.parent(i)} = dfvpdqd{m.parent(i)} + Xup{i}'*dfvpdqd{i};
          
          IC{m.parent(i)} = IC{m.parent(i)} + Xup{i}'*IC{i}*Xup{i};
          for k=1:m.NB,
            dIC{m.parent(i),k} = dIC{m.parent(i),k} + Xup{i}'*dIC{i,k}*Xup{i};
          end
          dIC{m.parent(i),n} = dIC{m.parent(i),n} + ...
            dXupdq{i}'*IC{i}*Xup{i} + Xup{i}'*IC{i}*dXupdq{i};
        end
      end
      
      % minor adjustment to make TaylorVar work better.
      %H = zeros(m.NB);
      H=zeros(m.NB)*q(1);
      
      %Derivatives wrt q(k)
      dH = zeros(m.NB^2,2*m.NB)*q(1);
      for k = 1:m.NB
        nk = m.position_num(k);
        for i = 1:m.NB
          n = m.position_num(i);
          fh = IC{i} * S{i};
          dfh = dIC{i,nk} * S{i};  %dfh/dqk
          H(n,n) = S{i}' * fh;
          dH(n + (n-1)*m.NB,nk) = S{i}' * dfh;
          j = i;
          while m.parent(j) > 0
            if j==k,
              dfh = Xup{j}' * dfh + dXupdq{j}' * fh;
            else
              dfh = Xup{j}' * dfh;
            end
            fh = Xup{j}' * fh;
            
            j = m.parent(j);
            np = m.position_num(j);
            
            H(n,np) = S{j}' * fh;
            H(np,n) = H(n,np);
            dH(n + (np-1)*m.NB,nk) = S{j}' * dfh;
            dH(np + (n-1)*m.NB,nk) = dH(n + (np-1)*m.NB,nk);
          end
        end
      end
      
      dH = dH(:,1:m.NB)*[eye(m.NB) zeros(m.NB)];
      dC(:,m.NB+1:end) = dC(:,m.NB+1:end) + diag(m.damping);
      
      ind = find(abs(qd)<m.coulomb_window');
      dind = sign(qd(ind))./m.coulomb_window(ind)' .* m.coulomb_friction(ind)';
      fc_drv = zeros(m.NB,1);
      fc_drv(ind) =dind;
      dC(:,m.NB+1:end) = dC(:,m.NB+1:end)+ diag(fc_drv);
    else
      [H,C] = HandC(m,q,qd,f_ext,obj.gravity);
    end
    
    C=C + computeFrictionForce(obj,qd);
  end
end

end

function [H,C,B,dH,dC,dB] = manipulatorDynamicsNew(obj,q,v,use_mex)
% manipulatorDynamics  Calculate coefficients of equation of motion.
% [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,v,use_mex) calculates the
% coefficients of the joint-space equation of motion,
% H(q)*vd+C(q,v,f_ext)=B(q,v)*tau, where q, v and vd are the joint
% position, velocity and acceleration vectors, H is the joint-space inertia
% matrix, C is the vector of gravity, external-force and velocity-product
% terms, and tau is the joint force vector.
%
% Algorithm: recursive Newton-Euler for C, and Composite-Rigid-Body for H.

compute_gradients = nargout > 3;

options.use_mex = use_mex;
options.compute_gradients = compute_gradients;
options.compute_JdotV = true;
kinsol = doKinematics(obj, q, v, options);

% if (nargin<4) use_mex = true; end
if compute_gradients
  [f_ext, B, df_ext, dB] = computeExternalForcesAndInputMatrix(obj, q, v);
else
  [f_ext, B] = computeExternalForcesAndInputMatrix(obj, q, v);
end

a_grav = [zeros(3, 1); obj.gravity];
if (use_mex && obj.mex_model_ptr~=0 && isnumeric(q) && isnumeric(v))
  f_ext = full(f_ext);  % makes the mex implementation simpler (for now)
  if compute_gradients
    df_ext = full(df_ext);
    [H, dH] = massMatrixmex(obj.mex_model_ptr);
    nv = obj.num_velocities;
    dH = [dH, zeros(numel(H), nv)];
    [C, dC] = inverseDynamicsmex(obj.mex_model_ptr, f_ext, [], df_ext);
  else
    H = massMatrixmex(obj.mex_model_ptr);
    C = inverseDynamicsmex(obj.mex_model_ptr, f_ext);
  end
  % TODO: implement in mex
  if compute_gradients
    [tau_friction, dtau_frictiondv] = computeFrictionForce(obj, kinsol.v);
    nq = obj.num_positions; nv = obj.num_velocities;
    dC(:, nq + (1:nv)) = dC(:, nq + (1:nv)) + dtau_frictiondv;
  else
    tau_friction = computeFrictionForce(obj, kinsol.v);
  end
  C = C + tau_friction;
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
  f_ext = zeros(6,NB);
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
  f_ext = [];
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
  
  if ~isempty(f_ext)
    external_wrench = f_ext(:, i);
    dexternal_wrench = getSubMatrixGradient(df_ext,1:twist_size,i,size(f_ext),1:nq);
    
    % external wrenches are expressed in 'joint' frame. Transform from
    % joint to world:
    T_joint_to_body = homogTransInv(manipulator.body(i).T_body_to_joint);
    T_joint_to_world = kinsol.T{i} * T_joint_to_body;
    T_world_to_joint = homogTransInv(T_joint_to_world);
    AdT_world_to_joint = transformAdjoint(T_world_to_joint);
    external_wrench = AdT_world_to_joint' * external_wrench;
    net_wrenches{i} = net_wrenches{i} - external_wrench;
    
    if compute_gradient
      dT_joint_to_world = matGradMult(kinsol.dTdq{i}, T_joint_to_body);
      dexternal_wrench = dTransformSpatialForce(T_joint_to_world, external_wrench, dT_joint_to_world, dexternal_wrench);
      dnet_wrenches{i} = dnet_wrenches{i} - dexternal_wrench;
      
      dexternal_wrenchdv = getSubMatrixGradient(df_ext,1:twist_size,i,size(f_ext),nq+(1:nv));
      dexternal_wrenchdv = AdT_world_to_joint' * dexternal_wrenchdv;
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
