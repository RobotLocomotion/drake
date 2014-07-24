classdef BodyMotionControlBlock < DrakeSystem

  properties
    nq;
    Kp;
    Kd;
    dt;
    controller_data; % pointer to shared data handle containing qtraj
    robot;
    body_ind;
  end
  
  methods
    function obj = BodyMotionControlBlock(r,name,controller_data,options)
      typecheck(r,'Biped');
      typecheck(controller_data,'QPControllerData');
      
      input_frame = getStateFrame(r);
      output_frame = BodySpatialAcceleration(r,name);
      obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.controller_data = controller_data;
      obj.nq = getNumDOF(r);

      if nargin<4
        options = struct();
      end
      
      if isfield(options,'Kp')
        typecheck(options.Kp,'double');
        sizecheck(options.Kp,[6 1]);
        obj.Kp = options.Kp;
      else
        obj.Kp = [100; 100; 100; 150; 150; 150];
      end        
        
      if isfield(options,'Kd')
        typecheck(options.Kd,'double');
        sizecheck(options.Kd,[6 1]);
        obj.Kd = options.Kd;
      else
        obj.Kd = [10; 10; 10; 10; 10; 10];
      end        
        
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.001;
      end
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate
      obj.robot = r;
      obj.body_ind = findLinkInd(r,name);
    end
   
    function y=output(obj,t,~,x)
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);
      kinsol = doKinematics(obj.robot,q,false,true,qd);

      % TODO: this should be updated to use quaternions/spatial velocity
      ctrl_data = obj.controller_data;
      [p,J] = forwardKin(obj.robot,kinsol,obj.body_ind,[0;0;0],1); 
      
      link_con_ind = [ctrl_data.link_constraints.link_ndx]==obj.body_ind;
      body_des = fasteval(ctrl_data.link_constraints(link_con_ind).traj,t);
      err = [body_des(1:3)-p(1:3);angleDiff(p(4:end),body_des(4:end))];
      
      body_vel_des = fasteval(ctrl_data.link_constraints(link_con_ind).dtraj,t);
      body_acc_des = fasteval(ctrl_data.link_constraints(link_con_ind).ddtraj,t);
      
      body_acc = obj.Kp.*err + obj.Kd.*(body_vel_des-J*qd) + body_acc_des;
      y = [obj.body_ind;body_acc];
    end
  end
  
end
