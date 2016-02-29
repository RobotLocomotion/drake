classdef BodyMotionControlBlock < DrakeSystem
 % A simple PD control block for regulating a body pose given a desired position, velocity, and acceleration.   

  properties
    nq;
    Kp;
    Kd;
    dt;
    controller_data; % pointer to shared data handle containing qtraj
    robot;
    body_ind;
    use_plan_shift;
  end
  
  methods
    function obj = BodyMotionControlBlock(r,name,controller_data,options)
      typecheck(r,'Biped');
      typecheck(controller_data,'QPControllerData');
      
      input_frame = getStateFrame(r);
      output_frame = atlasFrames.BodySpatialAcceleration(r,name);
      obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.controller_data = controller_data;
      obj.nq = getNumPositions(r);

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

      if isfield(options,'use_plan_shift')
        typecheck(options.use_plan_shift,{'logical', 'double'});
        sizecheck(options.use_plan_shift, [1 1]);
        obj.use_plan_shift = options.use_plan_shift;
	  else
        obj.use_plan_shift = false;
      end
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate
      obj.robot = r;
      obj.body_ind = findLinkId(r,name);
    end
   
    function y=output(obj,t,~,x)
      ctrl_data = obj.controller_data;
      
      if isfield(ctrl_data.link_constraints,'traj')
        ind = [ctrl_data.link_constraints.link_ndx]==obj.body_ind;
        body_des = fasteval(ctrl_data.link_constraints(ind).traj,t);
        if isfield(ctrl_data.link_constraints(ind),'dtraj') && ~isempty(ctrl_data.link_constraints(ind).dtraj)
          body_v_des = fasteval(ctrl_data.link_constraints(ind).dtraj,t);
        else
          body_v_des = [0;0;0;0;0;0];
        end
        if isfield(ctrl_data.link_constraints(ind),'ddtraj') && ~isempty(ctrl_data.link_constraints(ind).ddtraj)
          body_vdot_des = fasteval(ctrl_data.link_constraints(ind).ddtraj,t);
        else
          body_vdot_des = [0;0;0;0;0;0];
        end         
      else
        link_con_ind = [ctrl_data.link_constraints.link_ndx]==obj.body_ind;
        body_traj_ind = find(ctrl_data.link_constraints(link_con_ind).ts<=t,1,'last');
        tt = t-ctrl_data.link_constraints(link_con_ind).ts(body_traj_ind);
        coefs = ctrl_data.link_constraints(link_con_ind).coefs(:,body_traj_ind,:);
        [body_des,body_v_des,body_vdot_des] = evalCubicSplineSegment(tt,coefs);
      end

      if obj.use_plan_shift
        body_des(1:3) = body_des(1:3) - ctrl_data.plan_shift(1:3);
      end
      
      % lcmgl = LCMGLClient(sprintf('link_%d_desired', obj.body_ind));
      % lcmgl.sphere(body_des(1:3), 0.03, 20, 20);
      % lcmgl.switchBuffers();

      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);
      kinsol = doKinematics(obj.robot,q,false,true,qd);

      % TODO: this should be updated to use quaternions/spatial velocity
      [p,J] = forwardKin(obj.robot,kinsol,obj.body_ind,[0;0;0],1); 

      err = [body_des(1:3)-p(1:3);angleDiff(p(4:end),body_des(4:end))];

      body_vdot = obj.Kp.*err + obj.Kd.*(body_v_des-J*qd) + body_vdot_des; 
      y = [obj.body_ind;body_vdot];
    end
  end
  
end
