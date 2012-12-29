function doKinematics(model,q,b_compute_second_derivatives,use_mex)

if nargin<4, use_mex=true; end
if nargin<3, b_compute_second_derivatives=false; end
if (use_mex && model.mex_model_ptr && isnumeric(q))
  doKinematicspmex(model.mex_model_ptr,q,b_compute_second_derivatives);
else
  if isnumeric(q) && all(abs(q-[model.body.cached_q]')<1e-8)  % todo: make this tolerance a parameter
    % then my kinematics are up to date, don't recompute
    % the "isnumeric" check is for the sake of taylorvars
    if b_compute_second_derivatives && ~any(cellfun(@isempty,{model.body.ddTdqdq}))
      % also make sure second derivatives are already computed, if they
      % are requested
      return
    end
  end
  nq = model.featherstone.NB;
  for i=1:length(model.body)
    body = model.body(i);
    if (isempty(body.parent))
      body.T = body.Ttree;
      body.dTdq = zeros(3*nq,3);
      body.ddTdqdq = zeros(3*nq*nq,3);
    else
      qi = body.jsign*q(body.dofnum);
      
      TJ = Tjcalcp(body.jcode,qi);
      dTJ = dTjcalcp(body.jcode,qi)*body.jsign;
      
      body.T=body.parent.T*body.Ttree*TJ;
      
      % todo: consider pulling this out into a
      % "doKinematicsAndVelocities" version?  but I'd have to be
      % careful with caching.
      
      % note the unusual format of dTdq (chosen for efficiently calculating jacobians from many pts)
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
      body.dTdq = body.parent.dTdq*body.Ttree*TJ;
      this_dof_ind = body.dofnum+0:nq:3*nq;
      body.dTdq(this_dof_ind,:) = body.dTdq(this_dof_ind,:) + body.parent.T*body.Ttree*dTJ;
      
      if (b_compute_second_derivatives)
        % ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        body.ddTdqdq = body.parent.ddTdqdq*body.Ttree*TJ;
        
        ind = 3*nq*(body.dofnum-1) + (1:3*nq);  %ddTdqdqi
        body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.dTdq*body.Ttree*dTJ;
        
        ind = reshape(reshape(body.dofnum+0:nq:3*nq*nq,3,[])',[],1); % ddTdqidq
        body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.dTdq*body.Ttree*dTJ;
        
        ind = 3*nq*(body.dofnum-1) + this_dof_ind;  % ddTdqidqi
        body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.T*body.Ttree*ddTjcalcp(body.jcode,qi);  % body.jsign^2 is there, but unnecessary (since it's always 1)
      else
        body.ddTdqdq = {};
      end
      body.cached_q = q(body.dofnum);
    end
  end
end
end