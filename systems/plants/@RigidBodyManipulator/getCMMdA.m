function [A,dA] = getCMMdA(model,kinsol)
% returns the centroidal momentum matrix as described in Orin & Goswami 2008
%
% h = A*qd, where h(4:6) is the total linear momentum and h(1:3) is the
% total angular momentum in the centroid frame (world fram translated to COM).

if model.use_new_kinsol
  
  if ~isstruct(kinsol)
    % treat input as getCMM(model,q)
    q = kinsol;
    kinsol_options.use_mex = false;
    kinsol_options.compute_gradients = nargout > 1;
    kinsol = model.doKinematics(q, [], [], [], kinsol_options);
  end
  
  if nargout > 1
    [A, dA] = centroidalMomentumMatrix(model, kinsol);
  else
    A = centroidalMomentumMatrix(model, kinsol);
  end
else
  if ~isstruct(kinsol)
    % treat input as getCMM(model,q)
    kinsol = doKinematics(model,kinsol,false,false);
  end
  
  if (kinsol.mex)
    error('Drake:RigidBodyManipulator:getCMMdA:noMex', ...
      'getCMMdA is not implemented in mex yet');
  else
    q = kinsol.q;
    nq = getNumPositions(model);
    m = model.featherstone;
    
    A = zeros(6,nq) + 0*q(1);
    Phi = cell(m.NB,1);
    Xup = cell(m.NB,1); % spatial transforms from parent to child
    Xworld = cell(m.NB,1); % spatial transforms from world to each body
    Ic = cell(m.NB,1); % body spatial inertias
    
    compute_first_derivative = (nargout > 1);
    
    if compute_first_derivative
      dA = zeros(6,nq,nq) + 0*q(1);
      dXup_dq = cell(m.NB,1);
      dXworld_dq = cell(m.NB,1);
      [com, Jcom] = getCOM(model,kinsol);
      dXtr = dXtrans(-com);
      % prob not efficient
      dXcom_dq = -dXtr*Jcom;
      dIc = cell(m.NB,1); % derivative of body spatial inertias
      dIc_dq = cell(m.NB,1); % derivative of body spatial inertias
    else
      com = getCOM(model,kinsol);
    end
    Xcom = Xtrans(-com); % spatial transform from centroid to world
    
    for i = 1:m.NB
      Ic{i} = m.I{i};
      dIc{i} = zeros(6,6) + 0*q(1);
      dIc_dq{i} = zeros(36,nq) + 0*q(1);
    end
    
    for i = m.NB:-1:1
      n = m.position_num(i);
      [Xi,phi] = jcalc(m.pitch(i), q(n));
      Xup{i} = Xi * m.Xtree{i} + 0*q(n);
      
      if compute_first_derivative
        dXi_dq = zeros(36,nq) + 0*q(1);
        dXi_dq(:,n) = reshape(djcalc(m.pitch(i), q(n)),36,1);
        %dXup_dq{i} = zeros(36,nq);
        %dXup_dq{i}(:,n) = reshape(dXidq * m.Xtree{i},36,1);
        dXup_dq{i} = matGradMultMat(Xi,m.Xtree{i},dXi_dq,zeros(36,nq));
      end
      
      Phi{i} = phi;
      if m.parent(i) > 0
        Ic{m.parent(i)} = Ic{m.parent(i)} + Xup{i}'*Ic{i}*Xup{i};
        
        if compute_first_derivative
          dIc_dq{m.parent(i)} = dIc_dq{m.parent(i)} + matGradMultMat(Xup{i}',Ic{i}*Xup{i},transposeGrad(dXup_dq{i},[6,6]),matGradMultMat(Ic{i},Xup{i},dIc_dq{i},dXup_dq{i}));
        end
      end
    end
    
    for i = 1:m.NB
      n = m.position_num(i);
      if m.parent(i) > 0
        Xworld{i} = Xup{i} * Xworld{m.parent(i)};
        if compute_first_derivative
          dXworld_dq{i} = matGradMultMat(Xup{i},Xworld{m.parent(i)},dXup_dq{i},dXworld_dq{m.parent(i)});
        end
      else
        Xworld{i} = Xup{i};
        if compute_first_derivative
          dXworld_dq{i} = dXup_dq{i};
        end
      end
      Xg = Xworld{i} * Xcom; % spatial transform from centroid to body
      A(:,n) = Xg'*Ic{i}*Phi{i};
      
      if compute_first_derivative
        dXg_dq = matGradMultMat(Xworld{i},Xcom,dXworld_dq{i},dXcom_dq);
        dA(:,n,:) = matGradMultMat(Xg',Ic{i}*Phi{i},transposeGrad(dXg_dq,size(Xg)),matGradMult(dIc_dq{i},Phi{i}));
      end
    end
    %A = Xcom;
    if compute_first_derivative
      dA = reshape(dA,6*nq,nq);
      %dA = dXcom_dq;
    end
  end
end
end

function [A, dA] = centroidalMomentumMatrix(robot, kinsol)
% Computes the centroidal momentum matrix (CMM), i.e. the matrix that maps
% the joint velocity vector v to the robot's momentum h = [k; l], with
% angular momentum k and linear momentum l, expressed in a centroidal
% frame. See Orin13.
%
% @param kinsol solution structure obtained from doKinematics
%
% @retval A CMM
% @retval dA gradient of A with respect to coordinate vector q

compute_gradients = nargout > 1;

if kinsol.mex
  if compute_gradients
    [A, dA] = centroidalMomentumMatrixmex(robot.mex_model_ptr);
  else
    A = centroidalMomentumMatrixmex(robot.mex_model_ptr);
  end
else
  if compute_gradients
    [inertias_world, dinertias_world] = inertiasInWorldFrame(robot, kinsol);
    [crbs_world, dcrbs_world] = compositeRigidBodyInertias(robot, inertias_world, dinertias_world);
    [com, dcom] = robot.getCOM(kinsol);
  else
    inertias_world = inertiasInWorldFrame(robot, kinsol);
    crbs_world = compositeRigidBodyInertias(robot, inertias_world);
    com = robot.getCOM(kinsol);
  end
  
  ABlocks = cellfun(@mtimes, crbs_world(2:end), kinsol.J(2:end), 'UniformOutput', false);
  A = [ABlocks{:}]; % 'world momentum matrix'
  % A = cell2mat(ABlocks); % doesn't work with TaylorVar
  transform_com_to_world = zeros(4) * kinsol.q(1); % for TaylorVar
  transform_com_to_world(1:3, 1:3) = eye(3);
  transform_com_to_world(4, 4) = 1;
  transform_com_to_world(1:3, 4) = com;
  AdH = transformAdjoint(transform_com_to_world);
  A = AdH' * A;
  
  if compute_gradients
    nq = robot.getNumPositions();
    dABlocks = cellfun(@matGradMultMat, crbs_world(2:end), kinsol.J(2:end), dcrbs_world(2:end), kinsol.dJdq(2:end), 'UniformOutput', false);
    dA = vertcat(dABlocks{:});
    dtransform_com_to_world = zeros(numel(transform_com_to_world), nq);
    dtransform_com_to_world = setSubMatrixGradient(dtransform_com_to_world, dcom, 1:3, 4, size(transform_com_to_world));
    dA = dTransformSpatialForce(inv(transform_com_to_world), A, -dtransform_com_to_world, dA);
  end
end

end
