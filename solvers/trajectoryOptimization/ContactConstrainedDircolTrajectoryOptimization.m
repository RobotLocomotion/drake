classdef ContactConstrainedDircolTrajectoryOptimization < ConstrainedDircolTrajectoryOptimization
  properties
  end
  
  methods
    function obj = ContactConstrainedDircolTrajectoryOptimization(plant,N,duration,indices,options)
      if nargin < 5
        options = struct();
      end
      if ~isfield(options,'contact_q0')
        options.contact_q0 = zeros(plant.getNumPositions,1);
      end
      
      
      [phi,normal,d,xA,xB,idxA,idxB,mu] = plant.contactConstraints(options.contact_q0,false,struct('terrain_only',true));
      mu = mu(1);
      for i=1:length(indices),
        xA_i = xA(:,indices(i));
        xB_i = xB(:,indices(i));
        
        if plant.dim == 2,
          xA_i = plant.T_2D_to_3D'*xA_i;
          xB_i = plant.T_2D_to_3D'*xB_i;
        end
        
        if idxA(indices(i)) == 1,
          position_fun = drakeFunction.kinematic.RelativePosition(plant,idxB(indices(i)),idxA(indices(i)),xB_i);
          position_constraint = DrakeFunctionConstraint(xA_i,xA_i,position_fun);
        else
          position_fun = drakeFunction.kinematic.RelativePosition(plant,idxA(indices(i)),idxB(indices(i)),xA_i);
          position_constraint = DrakeFunctionConstraint(xB_i,xB_i,position_fun);
        end
        position_constraint.grad_level = 2;
        
        plant = plant.addPositionEqualityConstraint(position_constraint);
      end
      
      if plant.dim == 2
        options.relative_constraints = repmat([true;false],length(indices),1);
      else
        options.relative_constraints = repmat([true;true;false],length(indices),1);
      end

      obj = obj@ConstrainedDircolTrajectoryOptimization(plant,N,duration,options);
      
      nq = plant.getNumPositions;
      
      nonpen_inds = setdiff((1:length(phi))',indices);
      for i=1:N,
        nonPenetrationConstraint = FunctionHandleConstraint(zeros(length(nonpen_inds),1),inf(length(nonpen_inds),1),nq,@(q) obj.nonPenetration(q,nonpen_inds));
        obj = obj.addConstraint(nonPenetrationConstraint,obj.x_inds(1:nq,i));
      end
      
      if plant.dim == 2
        lz_inds = reshape([obj.l_inds(2:2:end,:) obj.lc_inds(2:2:end,:)],[],1);
        lx_inds = reshape([obj.l_inds(1:2:end,:) obj.lc_inds(1:2:end,:)],[],1);
        nlz = length(lz_inds);
        
        %lz >= 0
        obj = obj.addConstraint(BoundingBoxConstraint(zeros(nlz,1),inf(nlz,1)),lz_inds);
        
        % mulz >= lx, mulz >= -lx
        A_fric = [mu*eye(nlz) eye(nlz);mu*eye(nlz) -eye(nlz)];
        obj = obj.addConstraint(LinearConstraint(zeros(2*nlz,1),inf(2*nlz,1),A_fric),[lz_inds;lx_inds]);
      else
        lz_inds = reshape([obj.l_inds(2:3:end,:) obj.lc_inds(2:3:end,:)],[],1);
        lx_inds = reshape([obj.l_inds(1:3:end,:) obj.l_inds(2:3:end,:) obj.lc_inds(1:3:end,:) obj.lc_inds(2:3:end,:)],[],1);
        nlz = length(lz_inds);
        
        %lz >= 0
        obj = obj.addConstraint(BoundingBoxConstraint(zeros(nlz,1),inf(nlz,1)),lz_inds);
        
        % lz^2 >= |lx|^2
        A_fric = [eye(nlz) eye(nlz);eye(nlz) -eye(nlz)];
        obj = obj.addConstraint(LinearConstraint(zeros(2*nlz,1),inf(2*nlz,1),A_fric),[lz_inds;lx_inds]);
      end
    end
    
    function [f,df] = nonPenetration(obj,q,inds)
      [phi,normal,d,xA,xB,idxA,idxB,mu,n] = obj.plant.contactConstraints(q,false,struct('terrain_only',true));
      f = phi(inds);
      df = n(inds,:);
    end

  end
end