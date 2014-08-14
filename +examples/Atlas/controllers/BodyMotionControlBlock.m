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
    mex_ptr;
    use_mex;
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

      if isfield(options,'use_mex')
        sizecheck(options.use_mex,1);
        obj.use_mex = uint32(options.use_mex);
        rangecheck(obj.use_mex,0,2);
        if (obj.use_mex && exist('bodyMotionControlmex','file')~=3)
          error('can''t find bodyMotionControlmex.  did you build it?');
        end
      else
        obj.use_mex = 1;
      end

      if (obj.use_mex>0)
        obj.mex_ptr = SharedDataHandle(bodyMotionControlmex(0,obj.robot.getMexModelPtr.ptr,obj.Kp,obj.Kd,obj.body_ind));
      end
    end
   
    function y=output(obj,t,~,x)
      link_con = obj.controller_data.link_constraints;
      ind = [link_con.link_ndx]==obj.body_ind;
      body_des = fasteval(link_con(ind).traj,t);
      if isfield(link_con(ind),'dtraj') && ~isempty(link_con(ind).dtraj)
        body_v_des = fasteval(link_con(ind).dtraj,t);
      else
        body_v_des = [0;0;0;0;0;0];
      end
      if isfield(link_con(ind),'ddtraj') && ~isempty(link_con(ind).ddtraj)
        body_vdot_des = fasteval(link_con(ind).ddtraj,t);
      else
        body_vdot_des = [0;0;0;0;0;0];
      end 
      if (obj.use_mex == 0 || obj.use_mex==2)
        q = x(1:obj.nq);
        qd = x(obj.nq+1:end);
        kinsol = doKinematics(obj.robot,q,false,true,qd);

        % TODO: this should be updated to use quaternions/spatial velocity
        [p,J] = forwardKin(obj.robot,kinsol,obj.body_ind,[0;0;0],1); 

        err = [body_des(1:3)-p(1:3);angleDiff(p(4:end),body_des(4:end))];

        body_vdot = obj.Kp.*err + obj.Kd.*(body_v_des-J*qd) + body_vdot_des; 
        if obj.use_mex == 2
          % check that matlab/mex agree
          body_vdot_mex = bodyMotionControlmex(obj.mex_ptr.data,x,body_des,body_v_des,body_vdot_des);  
          valuecheck(body_vdot_mex,body_vdot);
        end
      else
        body_vdot = bodyMotionControlmex(obj.mex_ptr.data,x,body_des,body_v_des,body_vdot_des);  
      end
      y = [obj.body_ind;body_vdot];
    end
  end
  
end
