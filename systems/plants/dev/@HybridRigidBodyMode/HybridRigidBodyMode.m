classdef HybridRigidBodyMode < RigidBodyManipulator

  properties (SetAccess=protected)
    joint_limit_state % joint_limit_state = 0 if no joint limit is incurred
                      %                   = 1 if joint_limit_min is reached
                      %                   = 2 if joint_limit_max is reached
    contact_state     % contact state = 0 if not in contact
                      %               = 1 in contact, static friction
                      %               = 2 in contact, sliding friction
  end

  methods
    function obj = HybridRigidBodyMode(urdf_filename,joint_limit_state,contact_state,options)
      if (nargin<4) options=struct(); end
      w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      obj = obj@RigidBodyManipulator(urdf_filename,options);
      warning(w);

      sizecheck(joint_limit_state,[obj.num_positions,1]);
      obj.joint_limit_state = joint_limit_state;

      sizecheck(contact_state,[obj.num_contacts,1]);
      obj.contact_state = contact_state;

      % a reminder that somwhere I need to implement the sliding friction
      if any(contact_state>1) error('not implemented yet'); end

      obj = compile(obj);
    end

    function obj = compile(obj)
      obj = compile@RigidBodyManipulator(obj);

      if (obj.num_position_constraints  || obj.num_velocity_constraints)
        error('still need to handle the case whether there are other constraints involved, too');
      end

      obj = setNumPositionConstraints(obj,sum(obj.joint_limit_state~=0)+3*sum(obj.contact_state>0));
      obj = setNumVelocityConstraints(obj,0*sum(obj.contact_state==1));
    end

    function [phi,dphi,ddphi] = positionConstraints(obj,q)
      % phi = [phi_joint_limits;phi_contact_normal]
      % phi_contact_normal -- The distance between the active contact
      % points to the contact surface

      if(nargout == 1)
        phi_j = jointLimitConstraints(obj,q);
        pc_normal = contactNormalConstraints(obj,q);
      elseif(nargout == 2)
        [phi_j,dphi_j] = jointLimitConstraints(obj,q);
        [pc_normal,dpc_normal] = contactNormalConstraints(obj,q);
      elseif(nargout == 3)
        [phi_j,dphi_j,ddphi_j] = jointLimitConstraints(obj,q);
        [pc_normal,dpc_normal,ddpc_normal] = contactNormalConstraints(obj,q);
      end
      phi = [phi_j;pc_normal];
      if(nargout>1)
        dphi = [dphi_j;dpc_normal];
      end
      if(nargout>2)
        ddphi = [ddphi_j; ddpc_normal];
      end
    end

%    function psi = velocityConstraints(obj,q,qd)
%      [~,Jt] = contactTangentialConstraints(obj,q);
%      psi = cellfun(@(J) J*qd,Jt(:),'UniformOutput',false);
%      psi = vertcat(psi{:});
%    end

    function [phi_j,dphi_j,ddphi_j] = jointLimitConstraints(obj,q)
      % only the active joint limit
      joint_ind = (1:obj.num_positions)';
      joint_ind1 = joint_ind(obj.joint_limit_state == 1);
      joint_ind2 = joint_ind(obj.joint_limit_state == 2);
      phi_j = [q(joint_ind1)-obj.joint_limit_min(joint_ind1);...
        obj.joint_limit_max(joint_ind2)-q(joint_ind2)];
      if(nargout>1)
        dphi_j = [sparse((1:length(joint_ind1))',joint_ind1,ones(length(joint_ind1),1),length(joint_ind1),obj.num_positions);...
          sparse((1:length(joint_ind2))',joint_ind2,-ones(length(joint_ind2),1),length(joint_ind2),obj.num_positions)];
        if(nargout>2)
          ddphi_j = sparse((length(joint_ind1)+length(joint_ind2)),obj.num_positions*obj.num_positions);
        end
      end
    end

    function [pc_normal,dpc_normal,ddpc_normal] = contactNormalConstraints(obj,q)
      % only the normal of the active contact points
      if(nargout>2)
        [pc,dpc,ddpc] = activeContactPositions(obj,q);
      elseif(nargout>1)
        [pc,dpc] = activeContactPositions(obj,q);
      else
        pc = activeContactPositions(obj,q);
      end
      num_active_contacts = sum(obj.contact_state == 1);
      pos = collisionDetect(obj,pc);
      pc_normal = reshape(pc-pos,[],1);  % by definition, the relative pos to the closest point is the normal
      if(nargout>1)
        dpc_normal = dpc;
        if(nargout>2)
          ddpc_normal = ddpc;
        end
      end
    end

    function [pc_tan,dpc_tan,ddpc_tan] = contactTangentialConstraints(obj,q)
      % pc_tan -- horizontal position of the active contact points on the
      % contact surface
      if(nargout>2)
        [pc,dpc,ddpc] = activeContactPositions(obj,q);
      elseif(nargout>1)
        [pc,dpc] = activeContactPositions(obj,q);
      else
        pc = activeContactPositions(obj,q);
      end
      [pos,vel,normal,mu] = collisionDetect(obj,pc);
      relpos = pc-pos;
      d = obj.surfaceTangents(normal);
      m = length(d);
      pc_tan = cell(1,m);
      num_active_contacts = sum(obj.contact_state == 1);
      if(nargout>1)
        dpc_tan = cell(1,m);
        if(nargout>2)
          ddpc_tan = cell(1,m);
        end
      end
      for k = 1:m
        pc_tan{k} = sum(pc.*d{k},1)';
        if nargout>1
          dpc_tan{k} = sparse(repmat(1:num_active_contacts,3,1),1:3*num_active_contacts,d{k}(:))*dpc;
          if nargout>2
            ddpc_tan{k} = reshape(sparse(repmat(1:num_active_contacts,3,1),1:3*num_active_contacts,d{k}(:))*ddpc,numel(dpc_tan{k}),[]);
          end
        end
      end
    end

    function [p,J,dJ] = activeContactPositions(obj,q)
      %@retval p(:,i) = [px;py;pz] is the position of the ith active
      %contact points
      contact_pts_ind = 1:obj.num_contacts;
      contact_pts_ind1 = contact_pts_ind(obj.contact_state == 1);

      kinsol = doKinematics(obj,q,nargout>2);
      contact_pos = zeros(3,length(contact_pts_ind1))*q(1);
      if nargout>1
        J = zeros(3*length(contact_pts_ind1),obj.num_positions)*q(1);
        if(nargout>2)
          dJ = zeros(3*length(contact_pts_ind1),obj.num_positions^2)*q(1);
        end
      end
      count = 0;
      active_contact_count = 0;
      for i = 1:length(obj.body)
        nC = size(obj.body(i).contact_pts,2);
        if nC>0
          body_contact_pts = contact_pts_ind1(contact_pts_ind1>count&contact_pts_ind1<=count+nC);
          body_contact_pts = body_contact_pts-count;
          if body_contact_pts
            if(nargout>2)
              [contact_pos(:,active_contact_count+(1:length(body_contact_pts))),...
                J(3*active_contact_count+(1:3*length(body_contact_pts)),:),...
                dJ(3*active_contact_count+(1:3*length(body_contact_pts)),:)]...
                = forwardKin(obj,kinsol,i,obj.body(i).contact_pts(:,body_contact_pts));
            elseif(nargout>1)
              [contact_pos(:,active_contact_count+(1:length(body_contact_pts))),...
                J(3*active_contact_count+(1:3*length(body_contact_pts)),:)]...
                = forwardKin(obj,kinsol,i,obj.body(i).contact_pts(:,body_contact_pts));
            else
              contact_pos(:,active_contact_count+(1:length(body_contact_pts)))...
                = forwardKin(obj,kinsol,i,obj.body(i).contact_pts(:,body_contact_pts));
            end
          end
          count = count+nC;
          active_contact_count = active_contact_count + length(body_contact_pts);
        end
      end
      p = contact_pos;
    end

    function [c,dc] = frictionConeConstraints(obj,q,lambda_contact)
      % c>=0 means the friction force is in the friction cone
      contact_ind = (1:obj.num_contacts)';
      contact_ind1 = contact_ind(obj.contact_state == 1);
      contact_length1 = length(contact_ind1);
      lambda_contact = reshape(lambda_contact,[],contact_length1);
      lambda_tan = lambda_contact(1:end-1,:);
      lambda_normal = lambda_contact(end,:);
      [contact_pos,dcontact_pos] = obj.activeContactPositions(q);
      [~,~,normal,mu] = obj.collisionDetect(contact_pos);
      tan = obj.surfaceTangents(normal);
      if(iscell(tan))
        dim = 3;
        num_tan = length(tan);
        lambda_tan = permute(reshape(repmat(lambda_tan,1,dim),size(lambda_tan,1),size(lambda_tan,2),dim),[3,2,1]);
        friction = dot(cat(3,tan{:}),lambda_tan,3);
        normal_force = repmat(lambda_normal,3,1).*normal;
        c = sum((repmat(mu,dim,1).*normal_force).^2,1)-sum(friction.^2,1);
        c = c(:);
        if(nargout>1)
          dc_row = [];
          dc_col = [];
          dc_val = [];
          dc_row = [dc_row;(1:contact_length1)'];
          dc_col = [dc_col;obj.num_positions+reshape((1:contact_length1)*(1+num_tan),[],1)];
          dc_val = [dc_val; reshape(2*lambda_normal.*mu.^2.*sum(normal.^2,1),[],1)];
          for k = 1:num_tan
            dc_row = [dc_row;(1:contact_length1)'];
            dc_col = [dc_col;obj.num_positions+reshape((0:(contact_length1-1))*(1+num_tan)+k,[],1)];
            dc_val = [dc_val;reshape(-2*sum(friction.*tan{k},1),[],1)];
          end
          dc = sparse(dc_row,dc_col,dc_val,contact_length1,obj.num_positions+numel(lambda_contact));
        end
      end

    end

    function [x,success] = resolveConstraints(obj,x0,v)
      if (nargin<3) v=[]; end
      [x,success] = resolveConstraints@SecondOrderSystem(obj,x0,v);
      %  the manipulator resolve constraints add inequality constraints for
      %  joint limits and contacts.  this call skips over that method and
      %  goes right to the normal second order system method, since those
      %  inequalities are irrelevant here.
    end

  end
end
