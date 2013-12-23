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
  doKinematicsmex(model.mex_model_ptr,q,b_compute_second_derivatives,qd);
  kinsol.mex = true;
else
  kinsol.mex = false;
  
  nq = getNumDOF(model);
  nb = length(model.body);
  kinsol.T = cell(1,nb);
  kinsol.dTdq = cell(1,nb);
  kinsol.ddTdqdq = cell(1,nb);
  for i=1:length(model.body)
    body = model.body(i);
    if body.parent<1
      kinsol.T{i} = body.Ttree;
      kinsol.dTdq{i} = sparse(3*nq,4);
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
    end
  end
  
  kinsol.twist = computeTwists(model.body, kinsol.T, q, qd);
  kinsol.Tdot = computeTdots(kinsol.T, kinsol.twist);
  kinsol.dTdqdot = computeDTdqdots(model, kinsol.T, kinsol.dTdq, kinsol.Tdot, q, qd);
end

end

function twists = computeTwists(bodies, T, q, qd)
nb = length(bodies);
twists = cell(1, nb);
for i = 1 : nb
  body = bodies(i);
  if body.parent > 0 && ~isempty(qd)
    parentTwist = twists{body.parent};
    qBody = q(body.dofnum);
    vBody = qd(body.dofnum);
    jointTwist = motionSubspace(body, qBody) * vBody;
    twists{i} = transformTwists(T{i} \ T{body.parent}, parentTwist) + jointTwist;
  else
    twistSize = 6;
    twists{i} = zeros(twistSize, 1);
  end
end
end

function Tdot = computeTdots(T, twist)
Tdot = cell(length(T), 1);
for i = 1 : length(T)
  Tdot{i} = T{i} * twistToTildeForm(twist{i});
end
end

function dTdqdot = computeDTdqdots(model, T, dTdq, Tdot, q, qd)
nb = length(model.body);
dTdqdot = cell(nb);
nq = getNumDOF(model);

for i = 1 : nb
  body = model.body(i);
  if body.parent<1
    if ~isempty(qd)
      dTdqdot{i} = sparse(3*nq,4);
    end
  elseif body.floating==1
    if isempty(qd)
      dTdqdot{i} = [];
    else
      qi = q(body.dofnum); % qi is 6x1
      qdi = qd(body.dofnum);
      
      [rx,drx,ddrx] = rotx(qi(4)); [ry,dry,ddry] = roty(qi(5)); [rz,drz,ddrz] = rotz(qi(6));
      TJ = [rz*ry*rx,qi(1:3);zeros(1,3),1];
      dTJ{1} = sparse(1,4,1,4,4);
      dTJ{2} = sparse(2,4,1,4,4);
      dTJ{3} = sparse(3,4,1,4,4);
      dTJ{4} = [rz*ry*drx,zeros(3,1); zeros(1,4)];
      dTJ{5} = [rz*dry*rx,zeros(3,1); zeros(1,4)];
      dTJ{6} = [drz*ry*rx,zeros(3,1); zeros(1,4)];
      
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
      
      dTdqdot{i} = dTdqdot{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
      for j=1:6
        this_dof_ind = body.dofnum(j)+0:nq:3*nq;
        dTdqdot{i}(this_dof_ind,:) = dTdqdot{i}(this_dof_ind,:) + Tdot{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ{j}*body.T_body_to_joint + T{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJdot{j}*body.T_body_to_joint;
      end
    end
  elseif body.floating==2
    warning('first derivatives of quaternion floating base not implemented yet');
  else
    if ~isempty(qd)
      qi = q(body.dofnum);
      qdi = qd(body.dofnum);
      TJ = Tjcalc(body.pitch,qi);
      dTJ = dTjcalc(body.pitch,qi);
      TJdot = dTJ*qdi;
      dTJdot = ddTjcalc(body.pitch,qi)*qdi;
      this_dof_ind = body.dofnum+0:nq:3*nq;
      dTdqdot{i} = dTdqdot{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + dTdq{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
      dTdqdot{i}(this_dof_ind,:) = dTdqdot{i}(this_dof_ind,:) + Tdot{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint + T{body.parent}(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJdot*body.T_body_to_joint;
    end
  end
end


end