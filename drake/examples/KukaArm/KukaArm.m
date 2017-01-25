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
        urdf = 'urdf/iiwa14_with_weight.urdf';
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
    end

    function [f,df] = dynamics_w(obj,t,x,u,w)
     switch obj.disturbance_type
       case 1
         % ee force
         [f,df] = dynamics_w_ee(obj,t,x,u,w);       
       case 2
         % x error
         [f,df] = dynamics_w_x(obj,t,x,u,w);       
       case 3
         % u error
         [f,df] = dynamics_w_u(obj,t,x,u,w);       
       otherwise 
         error('Unknown disturbance type');
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

