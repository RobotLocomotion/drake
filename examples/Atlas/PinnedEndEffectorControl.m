classdef PinnedEndEffectorControl < MIMODrakeSystem
% A simple jacobian-based end-effector controller for the non-floating base
% Atlas model
  methods
    function obj = PinnedEndEffectorControl(sys,r,k_nom)
      % @param sys the PD-controlled closed-loop system
      % @param r the time-stepping manipulator
      % @param k_nom (optional) the nominal configuration gain
      
      typecheck(sys,'DrakeSystem');
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      qnomframe = AtlasJointConfig(r,false);

      input_frame = MultiCoordinateFrame({qnomframe,sys.getStateFrame});
      output_frame = AtlasPositionRef(r);

      obj = obj@MIMODrakeSystem(0,sys.getInputFrame.dim,input_frame,output_frame,false,true);
      obj = setSampleTime(obj,[.01;0]); % update at 50 Hz
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      obj.manip = r;
      
      if nargin > 2
        typecheck(k_nom,'double');
        obj.k_nom = k_nom;
      end
      
      x0 = r.getInitialState();
      B = r.getB();
      obj = setInitialState(obj,B' * x0(1:r.getNumDOF()));
      
      % set max commanded positions to joint limits
      [obj.q_d_min,obj.q_d_max] = obj.manip.getJointLimits();
      obj.q_d_max(obj.q_d_max == inf) = 1e10;
      obj.q_d_min(obj.q_d_min == -inf) = -1e10;
      obj.q_d_max = B' * obj.q_d_max;
      obj.q_d_min = B' * obj.q_d_min;
    end   
    
    function obj = addEndEffector(obj,end_effector)
      typecheck(end_effector,'EndEffector');
      obj.end_effectors = [end_effector,obj.end_effectors];
      input_frame = MultiCoordinateFrame({end_effector.frame,obj.getInputFrame.frame{:}});
      obj = obj.setNumInputs(obj.getNumInputs+end_effector.frame.dim);
      obj = obj.setInputFrame(input_frame);
    end
    
    function q_d0 = getInitialState(obj)
      q_d0 = obj.q_d0;
    end
        
    function q_dn = mimoUpdate(obj,t,q_d,varargin)
      for i=1:length(obj.end_effectors)
        ee_goals{i} = varargin{i};
      end
      q_nom = varargin{length(obj.end_effectors)+1};
      x = varargin{length(obj.end_effectors)+2};

      nq = obj.manip.getNumDOF();
      q = x(1:nq);
      st = obj.getSampleTime();
      dt = st(1);

      % debug
      colors = 'rgbyk';
      % compute end effector deltas
      dq_ee = zeros(nq,1);
      N_ee{1} = eye(nq);
      num_ee = length(obj.end_effectors);
      for i=num_ee:-1:1
        active = (ee_goals{i}(1) ~= 0);
        if active
          ee_goal_i = ee_goals{i}(2:4);
          [ee,Jee] = obj.end_effectors(i).doKin(q);
          err_ee = ee_goal_i - ee;
          % debug
          scope('Atlas',strcat('ee_err',int2str(i)),t,norm(err_ee),struct('linespec',colors(i+1),'scope_id',1));
          Pm = obj.end_effectors(i).P_mask;
          J1 = Jee*Pm;
          dq_ee_i = N_ee{num_ee-i+1} * obj.end_effectors(i).gain * pinv(J1) * err_ee;
          if (norm(dq_ee_i)>obj.end_effectors(i).normbound)
            dq_ee_i = obj.end_effectors(i).normbound*dq_ee_i/norm(dq_ee_i);
          end  
          dq_ee = dq_ee + dq_ee_i;
          % assume end effectors are priortized in the order they 
          % are given for now
          N_ee{num_ee-i+2} = N_ee{num_ee-i+1} * (eye(nq) - pinv(J1) * J1);
        else
          N_ee{num_ee-i+2} = N_ee{num_ee-i+1};
        end
      end
      
      % cascaded end effector null space projection matrix
      Nee = N_ee{length(obj.end_effectors)+1};
      
      % compute nominal position error
      err_nom = q_nom - q;
      dq_nom = obj.k_nom * err_nom;

      dq_des = dq_ee + Nee*dq_nom;
      
      % map to input frame
      dq_des = obj.manip.getB()' * dq_des;

      q_dn = q_d + dt*dq_des;
      q_dn = min(max(q_dn,obj.q_d_min),obj.q_d_max);
      
    end
    
    function y = mimoOutput(obj,t,q_d,varargin)
      y = q_d;
    end
    
    function obj = setInitialState(obj,q_d0)
      typecheck(q_d0,'double');
      sizecheck(q_d0,obj.getNumStates());
      obj.q_d0 = q_d0;
    end
  end
  
  properties
    manip
    k_nom = 0.25
    q_d0
    q_d_max
    q_d_min
    end_effectors = []
  end
end
