classdef PlanarTimeSteppingRBM < DrakeSystem
  % A discrete time system which simulates (an Euler approximation of) the
  % manipulator equations, with contact / limits resolved using the linear
  % complementarity problem formulation of contact in Stewart96.
  
  properties
    manip  % the CT manipulator
    timestep
  end
  
  methods
    function obj=PlanarTimeSteppingRBM(manipulator,timestep)

      checkDependency('pathlcp_enabled');
      
      switch class(manipulator)
        case {'char','PlanarRigidBodyModel'}
          % then assume it's a urdf file
          S = warning('off','Drake:PlanarRigidBodyManipulator:UnsupportedJointLimits');
          manipulator = PlanarRigidBodyManipulator(manipulator);
          warning(S);
      end
      typecheck(manipulator,'PlanarRigidBodyManipulator');
      obj = obj@DrakeSystem(0,manipulator.getNumStates(),manipulator.getNumInputs(),manipulator.getNumOutputs(),manipulator.isDirectFeedthrough(),manipulator.isTI());
      obj.manip = manipulator;
      
      typecheck(timestep,'double');
      sizecheck(timestep,1);
      obj.timestep = timestep;
      
      obj = setSampleTime(obj,[timestep;0]);
      
      if (getNumStateConstraints(obj.manip)>0)
        error('not implemented yet');
      end
      
      obj = setInputFrame(obj,getInputFrame(obj.manip));
      obj = setStateFrame(obj,getStateFrame(obj.manip));
      obj = setOutputFrame(obj,getOutputFrame(obj.manip));
      
    end
    
    function xdn = update(obj,t,x,u)
      % do LCP time-stepping
      num_q = obj.manip.num_q;
      q=x(1:num_q); qd=x((num_q+1):end);
      h = obj.timestep;
      
      [H,C,B] = manipulatorDynamics(obj.manip,q,qd);
      
      jointForces = B*u;
      
      
      mu = 1;
      
%      nC = size(obj.contact_pts,2);
      nC = 0;
      nL = sum([obj.manip.joint_limit_min~=-inf;obj.manip.joint_limit_max~=inf]); % number of joint limits
      nL_bi = 0;
%      nonContactPConst = obj.num_bilateral_constraints - nC;
%      nL_bi = nonContactPConst - nL;
      
      if (nC+nL+nL_bi==0)
        qd_out = qd - h*(H\(C+jointForces));
        q_out = q + h*qd_out;
        xdn = [q_out; qd_out];
        return;
      end      

%      doKinematics(obj.manip.model,q,qd);
      
%      % Handle translational springs and tendons
%      nSpring = length(obj.model.spring);
%      if nSpring > 0
%        fSpring = obj.springForce(q,qd,clutch);
%        jointForces = jointForces + fSpring;
%      end
      
%      if (~isempty(obj.model.passive_K) && ~isempty(obj.model.passive_B))
%        % determine which passive joints are active
%        active_ind = (q < obj.model.passive_max+1e-6).*(q > obj.model.passive_min-1e-6);
%        
%        passive_force = (-obj.model.passive_K*(q - obj.model.passive_nom) -...
%          obj.model.passive_B*qd).*active_ind + obj.model.passive_offset;
%        
%        jointForces = jointForces + passive_force;
%      end
      
      HinvJointForces = H\(jointForces);
      
      if nL_bi > 0
        [phi_bi,J_bi] = obj.positionConstraints(q);
        phi_bi = [phi_bi;-phi_bi];
        J_bi = [J_bi;-J_bi];
      else
        phi_bi = zeros(0,1);
        J_bi = zeros(0,num_q);
      end
      
      if (nL > 0)
        [phi_u,J_u] = obj.manip.jointLimits(q);
      else
        phi_u = zeros(0,1);
        J_u = zeros(0,num_q);
      end
      
      if nC > 0
        [phi_c, ~,  ~, ~, J_c]  = contactPositionsAndVelocities(obj,q,qd);
      else
        phi_c = zeros(0,1);
        J_c = zeros(0,num_q);
      end
      
      M = zeros(4*nC+nL+2*nL_bi);
      w = zeros(4*nC+nL+2*nL_bi,1);
      
      J_z = J_c(2:2:end,:);
      J_x = J_c(1:2:end,:);
      
      if nC > 0
        w(1:nC) = phi_c(2:2:end) + h*J_z*qd - h^2*J_z*(H\C) + h^2*J_z*HinvJointForces;
        M(1:nC,:) = [h*J_z*(H\J_z') h*J_z*(H\J_x') -h*J_z*(H\J_x') zeros(nC) h*J_z*(H\J_u') h*J_z*(H\J_bi')];
        
        w(nC+(1:nC)) = J_x*qd - h*J_x*(H\C) + h*J_x*HinvJointForces;
        M(nC+(1:nC),:) = [J_x*(H\J_z') J_x*(H\J_x') -J_x*(H\J_x') eye(nC) J_x*(H\J_u') h*J_x*(H\J_bi')];
        
        w(2*nC+(1:nC)) = -J_x*qd + h*J_x*(H\C) - h*J_x*HinvJointForces;
        M(2*nC+(1:nC),:) = [-J_x*(H\J_z') -J_x*(H\J_x') J_x*(H\J_x') eye(nC) -J_x*(H\J_u') -h*J_x*(H\J_bi')];
        
        M(3*nC+(1:nC),:) = [mu*eye(nC) -eye(nC) -eye(nC) zeros(nC, nC+nL+2*nL_bi)];
      end
      
      if nL > 0
        w(4*nC+(1:nL)) = phi_u + h*J_u*qd - h^2*J_u*(H\C) + h^2*J_u*HinvJointForces;
        M(4*nC+(1:nL),:) = [h*J_u*(H\J_z') h*J_u*(H\J_x') -h*J_u*(H\J_x') zeros(nL,nC) h*J_u*(H\J_u') h*J_u*(H\J_bi')];
      end
      
      if nL_bi > 0
        w(4*nC+nL+(1:2*nL_bi)) = phi_bi + h*J_bi*qd - h^2*J_bi*(H\C) + h^2*J_bi*HinvJointForces;
        M(4*nC+nL+(1:2*nL_bi),:) = [h*J_bi*(H\J_z') h*J_bi*(H\J_x') -h*J_bi*(H\J_x') zeros(2*nL_bi,nC) h*J_bi*(H\J_u') h*J_bi*(H\J_bi')];
      end
      
      % z >= 0, Mz + w >= 0, z'*(Mz + w) = 0
      % z = [lambda_z;lambda_x+;lambda_x-;lambda_u;lambda_b];
      z = pathlcp(M,w);
      
      J_full = [J_u; J_bi(1:nL_bi,:);J_c];
      lambda_z = z(1:nC);
      lambda_x = z(nC + (1:nC)) - z(2*nC + (1:nC));
      lambda_u = z(4*nC + (1:nL));
      lambda_bi = z(4*nC + nL + (1:nL_bi)) -  z(4*nC + nL + nL_bi + (1:nL_bi));
      lambda = [lambda_u; lambda_bi; reshape([lambda_x lambda_z]',[],1)];
      
      qd_out = qd - h*(H\C) + h*HinvJointForces + (H\J_full')*lambda;
      q_out = q + h*qd_out;
      xdn = [q_out; qd_out];
    end
    
    function y = output(obj,t,x,u)
      y = output(obj.manip,t,x,u);
    end

    function v = constructVisualizer(obj)
      v = constructVisualizer(obj.manip);
    end

  end
  
  
end
  