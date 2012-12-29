function kinsol=doKinematics(model,q,b_compute_second_derivatives,use_mex)
% Computes the (forward) kinematics of the manipulator.
%
% @retval kinsol a certificate containing the solution (or information
% about the solution) that must be passed to model.forwardKin() in order to
% be evaluated.  Note: the contents of kinsol may change with
% implementation details - do not attempt to use kinsol directly (our
% contract is simply that it can always be passed to forwardKin to retrieve
% the answer).
%

if nargin<4, use_mex = true; end
if nargin<3, b_compute_second_derivatives=false; end

kinsol.q = q;

if (use_mex && model.mex_model_ptr && isnumeric(q))
  doKinematicsmex(model.mex_model_ptr,q,b_compute_second_derivatives);
  kinsol.mex = true;
else
  kinsol.mex = false;
  if isnumeric(q) && all(abs(q-[model.body.cached_q]')<1e-8)  % todo: make this tolerance a parameter
    % then my kinematics are up to date, don't recompute
    % the "isnumeric" check is for the sake of taylorvars
    if b_compute_second_derivatives 
      if ~any(cellfun(@isempty,{model.body.ddTdqdq}))
        % also make sure second derivatives are already computed, if they
        % are requested
        return
      end
    else
      persistent kin_inefficiency_counter;  % would be better to do this on a per model basis, but it's not worth returning the model for it. 
      if isempty(kin_inefficiency_counter)
        warning('Drake:RigidBodyManipulator:IneffecientKinematics','FYI - you''ve computed this kinematics solution twice - first with second derivatives off, then again with second derivatives on');
        kin_inefficiency_counter = 1;
%      else  % no point, since I can't read it back out.  but this will
%      help me remember that it is what I would do if I pushed it to being
%      a property of the model class
%        kin_inefficiency_counter = kin_inefficiency_counter+1;
      end
    else
      return;
    end
  end
  nq = model.featherstone.NB;
  for i=1:length(model.body)
    body = model.body(i);
    if (isempty(body.parent))
      body.T = body.Ttree;
      body.dTdq = zeros(4*nq,4);
      body.ddTdqdq = zeros(4*nq*nq,4);
    else
      qi = q(body.dofnum);
      
      TJ = Tjcalc(body.pitch,qi);
      body.T=body.parent.T*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      
      % todo: consider pulling this out into a
      % "doKinematicsAndVelocities" version?  but I'd have to be
      % careful with caching.
      
      dTJ = dTjcalc(body.pitch,qi);
      % note the unusual format of dTdq (chosen for efficiently calculating jacobians from many pts)
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
      body.dTdq = body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      this_dof_ind = body.dofnum+0:nq:4*nq;
      body.dTdq(this_dof_ind,:) = body.dTdq(this_dof_ind,:) + body.parent.T*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint;
      
      if (b_compute_second_derivatives)
        % ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        body.ddTdqdq = body.parent.ddTdqdq*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
        
        ind = 4*nq*(body.dofnum-1) + (1:4*nq);  %ddTdqdqi
        body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint;
        
        ind = reshape(reshape(body.dofnum+0:nq:4*nq*nq,4,[])',[],1); % ddTdqidq
        body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint;
        
        ind = 4*nq*(body.dofnum-1) + this_dof_ind;  % ddTdqidqi
        body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.T*body.Ttree*inv(body.T_body_to_joint)*ddTjcalc(body.pitch,qi)*body.T_body_to_joint;  % body.jsign^2 is there, but unnecessary (since it's always 1)
      else
        body.ddTdqdq = {};
      end
      body.cached_q = q(body.dofnum);
    end
  end
end
end