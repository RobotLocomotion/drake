function kinsol=doKinematics(model,q,b_compute_second_derivatives,use_mex,qd)
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
  doKinematicsmex(model.mex_model_ptr.getData,q,b_compute_second_derivatives,qd);
  kinsol.mex = true;
else
  kinsol.mex = false;
  if isnumeric(q) && all(abs(q-[model.body.cached_q]')<1e-8) && (isempty(qd) || all(abs(qd-[model.body.cached_qd]')<1e-8))  % todo: make this tolerance a parameter
    % then my kinematics are up to date, don't recompute
    % the "isnumeric" check is for the sake of taylorvars
    
    if b_compute_second_derivatives 
      if ~any(cellfun(@isempty,{model.body.ddTdqdq}))
        % also make sure second derivatives are already computed, if they
        % are requested
        return
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
      end
    else
      return;
    end
  end
  nq = getNumDOF(model);
  for i=1:length(model.body)
    body = model.body(i);
    if (isempty(body.parent))
      body.T = body.Ttree;
      body.dTdq = zeros(3*nq,4);
      body.Tdot = zeros(4);
      body.dTdqdot = zeros(3*nq,4);
      body.ddTdqdq = zeros(3*nq*nq,4);
    elseif body.floating==1
      qi = q(body.dofnum); % qi is 6x1
      [rx,drx,ddrx] = rotx(qi(4)); [ry,dry,ddry] = roty(qi(5)); [rz,drz,ddrz] = rotz(qi(6));
      TJ = [rz*ry*rx,qi(1:3);zeros(1,3),1];
      body.T=body.parent.T*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      
      body.dTdq = [];

      % see notes below
      body.dTdq = body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      
      dTJ{1} = sparse(1,4,1,4,4);
      dTJ{2} = sparse(2,4,1,4,4);
      dTJ{3} = sparse(3,4,1,4,4);
      dTJ{4} = [rz*ry*drx,zeros(3,1); zeros(1,4)];
      dTJ{5} = [rz*dry*rx,zeros(3,1); zeros(1,4)];
      dTJ{6} = [drz*ry*rx,zeros(3,1); zeros(1,4)];
      
      for j=1:6
        this_dof_ind = body.dofnum(j)+0:nq:3*nq;
        body.dTdq(this_dof_ind,:) = body.dTdq(this_dof_ind,:) + body.parent.T(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ{j}*body.T_body_to_joint;
      end
      
      if (b_compute_second_derivatives)
        error('floating base second derivatives not implemented yet');
      end
      
      if isempty(qd)
        body.Tdot = [];
        body.dTdqdot = [];
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

        body.Tdot = body.parent.Tdot*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + body.parent.T*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        body.dTdqdot = body.parent.dTdqdot*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        for j=1:6
          this_dof_ind = body.dofnum(j)+0:nq:3*nq;
          body.dTdqdot(this_dof_ind,:) = body.dTdqdot(this_dof_ind,:) + body.parent.Tdot(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ{j}*body.T_body_to_joint + body.parent.T(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJdot{j}*body.T_body_to_joint;
        end
        body.cached_qd = qdi';
      end
      
      body.cached_q = qi';
    elseif body.floating==2
      qi = q(body.dofnum);  % qi is 7x1
      TJ = [quat2rotmat(qi(4:7)),qi(1:3);zeros(1,3),1];
      body.T=body.parent.T*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;

      body.dTdq = [];
      warning('first derivatives of quaternion floating base not implemented yet'); 

      body.cached_q = qi;
    else
      qi = q(body.dofnum);
      
      TJ = Tjcalc(body.pitch,qi);
      body.T=body.parent.T*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      
      % note the unusual format of dTdq (chosen for efficiently calculating jacobians from many pts)
      % dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,:),dq1 ...]
      dTJ = dTjcalc(body.pitch,qi);
      body.dTdq = body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
      this_dof_ind = body.dofnum+0:nq:3*nq;
      body.dTdq(this_dof_ind,:) = body.dTdq(this_dof_ind,:) + body.parent.T(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint;
      
      if (b_compute_second_derivatives)
        % ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        body.ddTdqdq = body.parent.ddTdqdq*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
        
        ind = 3*nq*(body.dofnum-1) + (1:3*nq);  %ddTdqdqi
        body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint;
        
        ind = reshape(reshape(body.dofnum+0:nq:3*nq*nq,3,[])',[],1); % ddTdqidq
        body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint;
        
        ddTJ = ddTjcalc(body.pitch,qi);
        ind = 3*nq*(body.dofnum-1) + this_dof_ind;  % ddTdqidqi
        body.ddTdqdq(ind,:) = body.ddTdqdq(ind,:) + body.parent.T(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*ddTJ*body.T_body_to_joint;  % body.jsign^2 is there, but unnecessary (since it's always 1)
      else
        body.ddTdqdq = [];
      end
      
      if isempty(qd)
        body.Tdot = [];
        body.dTdqdot = [];
      else
        qdi = qd(body.dofnum);
        TJdot = dTJ*qdi;
        dTJdot = ddTjcalc(body.pitch,qi)*qdi;
        body.Tdot = body.parent.Tdot*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + body.parent.T*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        body.dTdqdot = body.parent.dTdqdot*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        body.dTdqdot(this_dof_ind,:) = body.dTdqdot(this_dof_ind,:) + body.parent.Tdot(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint + body.parent.T(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJdot*body.T_body_to_joint;
        body.cached_qd = qdi;
      end
      
      body.cached_q = qi';
    end
  end
end
end