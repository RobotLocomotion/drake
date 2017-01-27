classdef KukaArm < RigidBodyManipulator

  properties
    hand_name = 'iiwa_link_ee';
    disturbance_type = 1; % 1 ee-force, 2-state error, 3-torque
  end
  
  methods
  
    function obj = KukaArm(options)
      if nargin < 1
        options = struct();
      end
      
      if ~isfield(options,'floating')
        options.floating = false;
      end
      if ~isfield(options,'urdf')
        urdf = 'urdf/iiwa14.urdf';
      end
      if ~isfield(options,'with_weight')
        options.with_weight = false;
      end
      if ~isfield(options,'with_box')
        options.with_box = false;
      end

      if options.with_weight
        urdf = 'urdf/iiwa14_fixed_gripper.urdf';
      end
      
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
      warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
      obj = obj@RigidBodyManipulator(urdf,options);

      if options.with_weight
        options_hand.weld_to_link = findLinkId(obj,obj.hand_name);
        obj = obj.addRobotFromURDF('urdf/robotiq_simple.urdf', [0;0;0.099], [pi/2;0;0], options_hand);
      end
      if options.with_box
        obj = obj.addRobotFromURDF('urdf/box.urdf', [0.6;0;1.4], [0;0;0]);
      end
      if options.with_shelf
        obj = obj.addRobotFromURDF('urdf/shelf.urdf', [0.6;0;0.88], [0;0;0]);
      end
      
      obj = obj.removeCollisionGroupsExcept({'manip'});
      obj = compile(obj);

    end
    
    function nw = getNumDisturbances(obj)
        switch obj.disturbance_type
            case 1
                nw = 3;
            case 2
                nw = obj.getNumContStates();
            case 3
                nw = obj.getNumInputs();
            otherwise
                error('Unknown disturbance type');
        end
    end
    
    function [f,df,d2f] = dynamics_w(obj,t,x,u,w)
        
        [f,df] = dynamics_w_ee(obj,t,x,u,w);
        
        if nargout == 3
            %Finite diff to get 2nd derivatives
            nx = length(x);
            nu = length(u);
            nw = length(w);
            
            Dx = 1e-6*eye(nx);
            Du = 1e-6*eye(nu);
            Dw = 1e-6*eye(nw);
            
            d2f = zeros(nx, 1+nx+nu+nw, 1+nx+nu+nw);
            for k = 1:nx
                [~,df_p] = dynamics_w_ee(obj,t,x+Dx(:,k),u,w);
                d2f(:,:,1+k) = df_p-df;
            end
            for k = 1:nu
                [~,df_p] = dynamics_w_ee(obj,t,x,u+Du(:,k),w);
                d2f(:,:,1+nx+k) = df_p-df;
            end
            for k = 1:nw
                [~,df_p] = dynamics_w_ee(obj,t,x,u,w+Dw(:,k));
                d2f(:,:,1+nx+nu+k) = df_p-df;
            end
            
            d2f = reshape(d2f,nx,(1+nx+nu+nw)*(1+nx+nu+nw));
            
        end

    end
    
    function [f,df] = dynamics_w_ee(obj,t,x,u,w)
      % w should be a 3x1 force vector in world coordinates acting at the
      % robot's end effector

      nq = obj.getNumPositions;
      q=x(1:nq); 
      
      if (nargout>1)
        nx = obj.getNumStates;
        nu = obj.getNumInputs;

        kinsol = doKinematics(obj, q, [], struct('compute_gradients', true));
        [~,J,dJ] = obj.forwardKin(kinsol,findLinkId(obj,obj.hand_name),[0;0;0]);
        uw = J'*w;
 
        dJtw = zeros(nq,nq);
        for i=1:nq
          dJtw(:,i) = dJ(:,(i-1)*nq+(1:nq))'*w;
        end
      
        [f,df] = dynamics(obj,t,x,u+uw);
        df_du = df(:,1+nx+(1:nu)); 
        df_dq = df(:,1+(1:nq)) + df_du*dJtw;
        df_dqd = df(:,1+nq+(1:nq));
        df_dw = df_du*J';
        
        df = [df(:,1),df_dq,df_dqd,df_du,df_dw];
      else
        kinsol = doKinematics(obj, q, []);
        [~,J] = obj.forwardKin(kinsol,findLinkId(obj,obj.hand_name),[0;0;0]);
        uw = J'*w;
      
        [f,df] = dynamics(obj,t,x,u+uw);
      end

    end
    
    function [f,df] = dynamics_w_x(obj,t,x,u,w)
      % w is a state error vector
      [f,df] = dynamics(obj,t,x+w,u);
      df = [df,df(:,1+(1:obj.getNumStates))];
    end
    
    function [f,df] = dynamics_w_u(obj,t,x,u,w)
      % u is a state error vector
      [f,df] = dynamics(obj,t,x,u+w);
      df = [df,df(:,1+obj.getNumStates+(1:obj.getNumInputs))];
    end 
  end
    
  
end

