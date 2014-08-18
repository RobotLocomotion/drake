function kinsol=doKinematics(model,q,b_compute_second_derivatives,use_mex,qd)
% kinsol=doKinematics(model,q,b_compute_second_derivatives,use_mex,qd)
% Computes the (forward) kinematics of the manipulator.
%
% @retval kinsol a certificate containing the solution (or information
% about the solution) that must be passed to model.forwardKin() in order to
% be evaluated.  Note: the contents of kinsol may change with
% implementation details - do not attempt to use kinsol directly (our
% contract is simply that it can always be passed to forwardKin to retrieve
% the answer).
%

checkDirty(model);
if nargin<5, qd=[]; end
if nargin<4, use_mex = true; end
if nargin<3, b_compute_second_derivatives=false; end

kinsol.q = q;
kinsol.qd = qd;

if (use_mex && model.mex_model_ptr~=0 && isnumeric(q))
  doKinematicsmex(model.mex_model_ptr,q,b_compute_second_derivatives,qd);
  kinsol.mex = true;
else
  kinsol.mex = false;
  
  nq = getNumDOF(model);
  nb = length(model.body);
  kinsol.T = cell(1,nb);
  kinsol.dTdq = cell(1,nb);
  kinsol.Tdot = cell(1,nb);
  kinsol.dTdqdot = cell(1,nb);
  kinsol.ddTdqdq = cell(1,nb);
  for i=1:length(model.body)
    body = model.body(i);
    if body.parent<1
      kinsol.T{i} = body.Ttree;
      kinsol.dTdq{i} = sparse(3*nq,4);
      if ~isempty(qd)
        kinsol.Tdot{i} = zeros(4);
        kinsol.dTdqdot{i} = sparse(3*nq,4);
      end
      if (b_compute_second_derivatives)
        kinsol.ddTdqdq{i} = sparse(3*nq*nq,4);
      end
    elseif body.floating==1
      qi = q(body.dofnum); % qi is 6x1
      [rx,drx,ddrx] = rotx(qi(4)); [ry,dry,ddry] = roty(qi(5)); [rz,drz,ddrz] = rotz(qi(6));
      TJ = [rz*ry*rx,qi(1:3);zeros(1,3),1];
      kinsol.T{i}=kinsol.T{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      
      % see notes below
      kinsol.dTdq{i} = kinsol.dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      
      dTJ{1} = sparse(1,4,1,4,4);
      dTJ{2} = sparse(2,4,1,4,4);
      dTJ{3} = sparse(3,4,1,4,4);
      dTJ{4} = [rz*ry*drx,zeros(3,1); zeros(1,4)];
      dTJ{5} = [rz*dry*rx,zeros(3,1); zeros(1,4)];
      dTJ{6} = [drz*ry*rx,zeros(3,1); zeros(1,4)];
      
      for j=1:6
        this_dof_ind = body.dofnum(j)+0:nq:3*nq;
        kinsol.dTdq{i}(this_dof_ind,:) = kinsol.dTdq{i}(this_dof_ind,:) + kinsol.T{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ{j}*body.T_body_to_joint;
      end
      
      if (b_compute_second_derivatives)
        ddTJ = cell(6,6);
        % if j<=3 or k<=3, then ddTJ{j,k} = zeros(4); so I've left them out
        ddTJ{4,4} = [rz*ry*ddrx,zeros(3,1); zeros(1,4)];
        ddTJ{4,5} = [rz*dry*drx,zeros(3,1); zeros(1,4)];
        ddTJ{4,6} = [drz*ry*drx,zeros(3,1); zeros(1,4)];
        ddTJ{5,4} = [rz*dry*drx,zeros(3,1); zeros(1,4)];
        ddTJ{5,5} = [rz*ddry*rx,zeros(3,1); zeros(1,4)];
        ddTJ{5,6} = [drz*dry*rx,zeros(3,1); zeros(1,4)];
        ddTJ{6,4} = [drz*ry*drx,zeros(3,1); zeros(1,4)];
        ddTJ{6,5} = [drz*dry*rx,zeros(3,1); zeros(1,4)];
        ddTJ{6,6} = [ddrz*ry*rx,zeros(3,1); zeros(1,4)];
        
        % ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        kinsol.ddTdqdq{i} = kinsol.ddTdqdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
        for j = 1:6
          ind = 3*nq*(body.dofnum(j)-1) + (1:3*nq); %ddTdqdqj
          kinsol.ddTdqdq{i}(ind,:) = kinsol.ddTdqdq{i}(ind,:) + kinsol.dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*dTJ{j}*body.T_body_to_joint;
          
          ind = reshape(reshape(body.dofnum(j)+0:nq:3*nq*nq,3,[])',[],1); %ddTdqjdq
          kinsol.ddTdqdq{i}(ind,:) = kinsol.ddTdqdq{i}(ind,:) + kinsol.dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*dTJ{j}*body.T_body_to_joint;
          
          if (j>=4)
            for k = 4:6  
              ind = 3*nq*(body.dofnum(k)-1) + (body.dofnum(j)+0:nq:3*nq);  % ddTdqjdqk
              kinsol.ddTdqdq{i}(ind,:) = kinsol.ddTdqdq{i}(ind,:) + kinsol.T{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*ddTJ{j,k}*body.T_body_to_joint;
            end
          end
        end

      else
        kinsol.ddTdqdq{i} = [];
      end
      
      if isempty(qd)
        kinsol.Tdot{i} = [];
        kinsol.dTdqdot{i} = [];
      else
        qdi = qd(body.dofnum);
        TJdot = zeros(4);
        dTJdot{1} = zeros(4);
        dTJdot{2} = zeros(4);
        dTJdot{3} = zeros(4);
        dTJdot{4} = [(drz*qdi(6))*ry*drx + rz*(dry*qdi(5))*drx + rz*ry*(ddrx*qdi(4)),zeros(3,1); zeros(1,4)];
        dTJdot{5} = [(drz*qdi(6))*dry*rx + rz*(ddry*qdi(5))*rx + rz*dry*(drx*qdi(4)),zeros(3,1); zeros(1,4)];
        dTJdot{6} = [(ddrz*qdi(6))*ry*rx + drz*(dry*qdi(5))*rx + drz*ry*(drx*qdi(4)),zeros(3,1); zeros(1,4)];
        for j=1:6
          TJdot = TJdot+dTJ{j}*qdi(j);
        end
        
        kinsol.Tdot{i} = kinsol.Tdot{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + kinsol.T{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        kinsol.dTdqdot{i} = kinsol.dTdqdot{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + kinsol.dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        for j=1:6
          this_dof_ind = body.dofnum(j)+0:nq:3*nq;
          kinsol.dTdqdot{i}(this_dof_ind,:) = kinsol.dTdqdot{i}(this_dof_ind,:) + kinsol.Tdot{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ{j}*body.T_body_to_joint + kinsol.T{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJdot{j}*body.T_body_to_joint;
        end
      end
    elseif body.floating==2
      qi = q(body.dofnum);  % qi is 7x1
      TJ = [quat2rotmat(qi(4:7)),qi(1:3);zeros(1,3),1];
      kinsol.T{i}=kinsol.T{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      
      warning('first derivatives of quaternion floating base not implemented yet');
    else
      qi = q(body.dofnum);
      
      TJ = Tjcalc(body.pitch,qi);
      kinsol.T{i}=kinsol.T{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      
      % note the unusual format of dTdq (chosen for efficiently calculating jacobians from many pts)
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,:),dq1 ...]
      dTJ = dTjcalc(body.pitch,qi);
      kinsol.dTdq{i} = kinsol.dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      this_dof_ind = body.dofnum+0:nq:3*nq;
      kinsol.dTdq{i}(this_dof_ind,:) = kinsol.dTdq{i}(this_dof_ind,:) + kinsol.T{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint;
      
      if (b_compute_second_derivatives)
        % ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
       kinsol.ddTdqdq{i} = kinsol.ddTdqdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
        
       ind = 3*nq*(body.dofnum-1) + (1:3*nq);  %ddTdqdqi
       kinsol.ddTdqdq{i}(ind,:) = kinsol.ddTdqdq{i}(ind,:) + kinsol.dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint;
        
       ind = reshape(reshape(body.dofnum+0:nq:3*nq*nq,3,[])',[],1); % ddTdqidq
       kinsol.ddTdqdq{i}(ind,:) = kinsol.ddTdqdq{i}(ind,:) + kinsol.dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint;
        
       ddTJ = ddTjcalc(body.pitch,qi);
       ind = 3*nq*(body.dofnum-1) + this_dof_ind;  % ddTdqidqi
       kinsol.ddTdqdq{i}(ind,:) = kinsol.ddTdqdq{i}(ind,:) + kinsol.T{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*ddTJ*body.T_body_to_joint;  % body.jsign^2 is there, but unnecessary (since it's always 1)
      end
      
      if ~isempty(qd)
        qdi = qd(body.dofnum);
        TJdot = dTJ*qdi;
        dTJdot = ddTjcalc(body.pitch,qi)*qdi;
        kinsol.Tdot{i} = kinsol.Tdot{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + kinsol.T{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        kinsol.dTdqdot{i} = kinsol.dTdqdot{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + kinsol.dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        kinsol.dTdqdot{i}(this_dof_ind,:) = kinsol.dTdqdot{i}(this_dof_ind,:) + kinsol.Tdot{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint + kinsol.T{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJdot*body.T_body_to_joint;
      end
    end
  end
end
<<<<<<< HEAD
=======
end


function T = computeTransforms(bodies, q)
nb = length(bodies);
T = cell(1, nb);
for i=1:nb
  body = bodies(i);
  if body.parent<1
    T{i} = body.Ttree;
  else
    qi = q(body.position_num); % qi is 6x1
    T_body_to_parent = body.Ttree*jointTransform(body, qi);
    T{i}=T{body.parent}*T_body_to_parent;
  end
end
end

function [S, dSdq] = computeMotionSubspaces(bodies, q)
compute_gradients = nargout > 1;
nb = length(bodies);
S = cell(1, nb);
if compute_gradients
  dSdq = cell(1, nb);
end

for i = 2 : nb
  body = bodies(i);
  qBody = q(body.position_num);
  if compute_gradients
    [S{i}, dSdq{i}] = motionSubspace(body, qBody);
  else
    S{i} = motionSubspace(body, qBody);
  end
end
end

function ret = computeTransformGradients(bodies, T, S, qdotToV)
% computes the gradients of T{i} with respect to q
% makes use of the fact that the gradients of the joint transforms are
% dT/dq = dTdot/dqdot, where Tdot depends on v through the joint motion
% subspace S and qdot depends on v via the qdotToV mapping.

nb = length(bodies);
nq = size(qdotToV, 2);
ret = cell(1, nb);
ret{1} = zeros(16, nq);
for i = 2 : nb
  body = bodies(i);
  T_body_to_parent = T{body.parent} \ T{i};
  qdotToVi = qdotToV(body.velocity_num, body.position_num);
  
  dT_body_to_parentdqi = dHomogTrans(T_body_to_parent, S{i}, qdotToVi);
  dT_body_to_parentdq = zeros(numel(T{i}), nq) * dT_body_to_parentdqi(1); % to make TaylorVar work better
  dT_body_to_parentdq(:, body.position_num) = dT_body_to_parentdqi;
  ret{i} = matGradMultMat(...
    T{body.parent}, T_body_to_parent, ret{body.parent}, dT_body_to_parentdq);
end
end

function J = computeJ(T, S)
% Computes motion subspaces transformed to world frame, i.e.
% transformAdjoint(T{i}) * S{i}

nb = length(T);
J = cell(1, nb);
for i = 2 : nb
  J{i} = transformAdjoint(T{i}) * S{i};
end
end

function ret = computedJdq(bodies, T, S, dTdq, dSdq)
% Computes the gradient of transformAdjoint(T{i}) * S{i} with respect to q
% For uses, see e.g. geometricJacobianDotTimesV, gradients of mass matrix
% and bias vector in manipulatorDynamics

nb = length(T);
ret = cell(1, nb);
for i = 2 : nb
  body = bodies(i);
  nq = size(dTdq{i}, 2);
  Si = S{i};
  dSdqi = zeros(numel(Si), nq) * dTdq{i}(1); % to make TaylorVar work better
  dSdqi(:, body.position_num) = dSdq{i};
  ret{i} = dTransformAdjoint(T{i}, Si, dTdq{i}, dSdqi);
end
end

function [twists, dtwistsdq] = computeTwistsInBaseFrame(bodies, J, v, dJdq)
compute_gradient = nargout > 1;
if compute_gradient
  if nargin < 4
    error('must provide dJdq to compute gradient');
  end
  nq = size(dJdq{end}, 2);
end

nb = length(bodies);
twistSize = 6;

twists = cell(1, nb);
twists{1} = zeros(twistSize, 1);

if compute_gradient
  dtwistsdq = cell(1, nb);
  dtwistsdq{1} = zeros(numel(twists{1}), nq);
end

for i = 2 : nb
  body = bodies(i);
  vBody = v(body.velocity_num);

  parentTwist = twists{body.parent};
  jointTwist = J{i} * vBody;
  twists{i} = parentTwist + jointTwist;
  
  if compute_gradient
    dparentTwist = dtwistsdq{body.parent};
    dJointTwist = matGradMult(dJdq{i}, vBody);
    dtwistsdq{i} = dparentTwist + dJointTwist;
  end
end
end

function [SdotV, dSdotVdq, dSdotVdv] = computeMotionSubspacesDotV(bodies, q, v)
compute_gradients = nargout > 1;

nb = length(bodies);
SdotV = cell(1, nb);
if compute_gradients
  dSdotVdq = cell(1, nb);
  dSdotVdv = cell(1, nb);
end

for i = 2 : nb
  body = bodies(i);
  qi = q(body.position_num);
  vi = v(body.velocity_num);
  if compute_gradients
    [SdotV{i}, dSdotVdq{i}, dSdotVdv{i}] = motionSubspaceDotTimesV(body, qi, vi);
  else
    SdotV{i} = motionSubspaceDotTimesV(body, qi, vi);
  end
end
end
>>>>>>> 89b3f7f... Now doing joint transform properly (including rotations/translations about axes other than z) in jointTransform.m

end