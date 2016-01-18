classdef ContactConstrainedDircolTrajectoryOptimization < AccelConstrainedDircolTrajectoryOptimization
  % Contact specific implementation of the DIRCON algorithm found in
  % "Optimization and stabilization of trajectories for constrained
  %  dynamical systems" by Posa, Kuindersma and Tedrake 2016.
  %
  % Extends AccelConstrainedDircolTrajectoryOptimization by incorporating
  % contact information as position constraints.
  %
  % Use the constructor:
  %   obj = ContactConstrainedDircolTrajectoryOptimization(plant,N,duration,indices,options)
  % which accepts the default Dircol arguments as well as some the contact
  % indices and additional options
  %   @param indices : a vector of indices into the list of possible
  %   contacts coming from plant.contactConstraints()
  %   options.contact_q0 : the position vector used to determine the
  %     nominal contact position. Used for the normal direction nominally, and
  %     the tangential direction only if relative_constraints=false.
  %     Defaults to all zeros.
  %   options.relative_constraints : Determines whether the tangential
  %     directions are relative, or are fixed to the position given by
  %     contact_q0. Defaults to TRUE, unlike superclass
  %   options.friction_limits :  Activates friction constraints on contact
  %     forces at knot points. Defaults to true.
  %   options.collocation_friction_limits : Activates friction constraints
  %     at collocation points. Defaults to true.
  %   options.non_penetration : Enforces non-penetration constraints for
  %     inactive contacts. Defaults to true.
  %   options.additional_constraints : An additional set of position
  %    constraints to be added to the plant, aside from those derived from
  %    contact

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
      
      if ~isfield(options,'friction_limits')
        options.friction_limits = true;
      end
      
      if ~isfield(options,'collocation_friction_limits')
        options.collocation_friction_limits = true;
      end
      
      if ~isfield(options,'non_penetration')
        options.non_penetration = true;
      end
      
      if ~isfield(options,'relative_constraints')
        options.relative_constraints = true;
      end
      
      if ~isfield(options,'additional_constraints')
        options.additional_constraints = [];
      end
      
      if length(options.relative_constraints) > 1
        error('ContactConstrainedDircolTrajectoryOptimization only supports a single value for options.relative_constraints');
      end
      
      
      [phi,normal,d,xA,xB,idxA,idxB,mu] = plant.contactConstraints(options.contact_q0,false,struct('terrain_only',true));
      mu = mu(1);
      for i=1:length(indices),
        xA_i = xA(:,indices(i));
        xB_i = xB(:,indices(i));
        
        if plant.dim == 2,
          if idxA(indices(i)) == 1,
            xA_i = plant.T_2D_to_3D'*xA_i;
          else
            xB_i = plant.T_2D_to_3D'*xB_i;
          end
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
      
      for i=1:length(options.additional_constraints)
        plant = plant.addPositionEqualityConstraint(options.additional_constraints{i});
      end
      
      if plant.dim == 2
        options.relative_constraints = repmat([options.relative_constraints;false],length(indices),1);
      else
        options.relative_constraints = repmat([options.relative_constraints;options.relative_constraints;false],length(indices),1);
      end

      obj = obj@AccelConstrainedDircolTrajectoryOptimization(plant,N,duration,options);
      
      nq = plant.getNumPositions;
      
      nonpen_inds = setdiff((1:length(phi))',indices);
      
      if obj.options.non_penetration
        for i=1:N,
          nonPenetrationConstraint = FunctionHandleConstraint(zeros(length(nonpen_inds),1),inf(length(nonpen_inds),1),nq,@(q) obj.nonPenetration(q,nonpen_inds));
          obj = obj.addConstraint(nonPenetrationConstraint,obj.x_inds(1:nq,i));
        end
      end
      
      if obj.options.friction_limits
        if plant.dim == 2
          if obj.options.collocation_friction_limits
            lz_inds = reshape([obj.l_inds(2:2:end,:) obj.lc_inds(2:2:end,:)],[],1);
            lx_inds = reshape([obj.l_inds(1:2:end,:) obj.lc_inds(1:2:end,:)],[],1);
          else
            lz_inds = reshape(obj.l_inds(2:2:end,:),[],1);
            lx_inds = reshape(obj.l_inds(1:2:end,:),[],1);
          end
          nlz = length(lz_inds);
          
          %lz >= 0
          obj = obj.addConstraint(BoundingBoxConstraint(zeros(nlz,1),inf(nlz,1)),lz_inds);
          
          % mulz >= lx, mulz >= -lx
          A_fric = [mu*eye(nlz) eye(nlz);mu*eye(nlz) -eye(nlz)];
          obj = obj.addConstraint(LinearConstraint(zeros(2*nlz,1),inf(2*nlz,1),A_fric),[lz_inds;lx_inds]);
        else
          mu = .7;
          if obj.options.collocation_friction_limits
            lz_inds = reshape([obj.l_inds(3:3:end,:) obj.lc_inds(3:3:end,:)],[],1);
            lx_inds = reshape([obj.l_inds(1:3:end,:) obj.lc_inds(1:3:end,:)],[],1);
            ly_inds = reshape([obj.l_inds(2:3:end,:) obj.lc_inds(2:3:end,:)],[],1);
          else
            lz_inds = reshape(obj.l_inds(3:3:end,:),[],1);
            lx_inds = reshape(obj.l_inds(1:3:end,:),[],1);
            ly_inds = reshape(obj.l_inds(2:3:end,:),[],1);
          end
          
          nlz = length(lz_inds);
          
          %lz >= 0
          obj = obj.addConstraint(BoundingBoxConstraint(zeros(nlz,1),inf(nlz,1)),lz_inds);
          
          % using l1 norm for nowlz^2 >= |lx|^2
          A_fric = [mu*eye(nlz) eye(nlz);mu*eye(nlz) -eye(nlz)];
          obj = obj.addConstraint(LinearConstraint(zeros(2*nlz,1),inf(2*nlz,1),A_fric),[lz_inds;lx_inds]);
          obj = obj.addConstraint(LinearConstraint(zeros(2*nlz,1),inf(2*nlz,1),A_fric),[lz_inds;ly_inds]);
        end
      end
    end
    
    function [f,df] = nonPenetration(obj,q,inds)
      [phi,normal,d,xA,xB,idxA,idxB,mu,n] = obj.plant.contactConstraints(q,false,struct('terrain_only',true));
      f = phi(inds);
      df = n(inds,:);
    end

  end
end