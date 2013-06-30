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

checkDirty(model);
if nargin<4, use_mex=true; end
if nargin<3, b_compute_second_derivatives=false; end

kinsol.q = q;

if (use_mex && model.mex_model_ptr~=0 && isnumeric(q))
  doKinematicspmex(model.mex_model_ptr.getData,q,b_compute_second_derivatives);
  kinsol.mex = true;
else
  kinsol.mex = false;
  
  nq = model.featherstone.NB;
  nb = length(model.body);
  kinsol.T = cell(1,nb);
  kinsol.dTdq = cell(1,nb);
  kinsol.Tdot = cell(1,nb);
  kinsol.dTdqdot = cell(1,nb);
  kinsol.ddTdqdq = cell(1,nb);
  for i=1:length(model.body)
    body = model.body(i);
    if (body.parent<1)
      kinsol.T{i} = body.Ttree;
      kinsol.dTdq{i} = zeros(3*nq,3);
      if (b_compute_second_derivatives)
        kinsol.ddTdqdq{i} = zeros(3*nq*nq,3);
      end
    else
      qi = body.jsign*q(body.dofnum);
      
      TJ = Tjcalcp(body.jcode,qi);
      dTJ = dTjcalcp(body.jcode,qi)*body.jsign;
      
      kinsol.T{i}=kinsol.T{body.parent}*body.Ttree*TJ;
      
      % todo: consider pulling this out into a
      % "doKinematicsAndVelocities" version?  but I'd have to be
      % careful with caching.
      
      % note the unusual format of dTdq (chosen for efficiently calculating jacobians from many pts)
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
      kinsol.dTdq{i} = kinsol.dTdq{body.parent}*body.Ttree*TJ;
      this_dof_ind = body.dofnum+0:nq:3*nq;
      kinsol.dTdq{i}(this_dof_ind,:) = kinsol.dTdq{i}(this_dof_ind,:) + kinsol.T{body.parent}*body.Ttree*dTJ;
      
      if (b_compute_second_derivatives)
        % ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        kinsol.ddTdqdq{i} = kinsol.ddTdqdq{body.parent}*body.Ttree*TJ;
        
        ind = 3*nq*(body.dofnum-1) + (1:3*nq);  %ddTdqdqi
        kinsol.ddTdqdq{i}(ind,:) = kinsol.ddTdqdq{i}(ind,:) + kinsol.dTdq{body.parent}*body.Ttree*dTJ;
        
        ind = reshape(reshape(body.dofnum+0:nq:3*nq*nq,3,[])',[],1); % ddTdqidq
        kinsol.ddTdqdq{i}(ind,:) = kinsol.ddTdqdq{i}(ind,:) + kinsol.dTdq{body.parent}*body.Ttree*dTJ;
        
        ind = 3*nq*(body.dofnum-1) + this_dof_ind;  % ddTdqidqi
        kinsol.ddTdqdq{i}(ind,:) = kinsol.ddTdqdq{i}(ind,:) + kinsol.T{body.parent}*body.Ttree*ddTjcalcp(body.jcode,qi);  % body.jsign^2 is there, but unnecessary (since it's always 1)
      end
    end
  end
end
end