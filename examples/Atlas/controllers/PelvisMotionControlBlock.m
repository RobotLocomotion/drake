classdef PelvisMotionControlBlock < DrakeSystem

  properties
    nq;
    Kp;
    Kd;
    dt;
    controller_data; % pointer to shared data handle containing qtraj
    robot;
    body_ind;
    rfoot_ind;
    lfoot_ind;
    mex_ptr;
    alpha;
    nominal_pelvis_height;
    use_mex;
  end
  
  methods
    function obj = PelvisMotionControlBlock(r,name,controller_data,options)
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
        obj.Kp = [0; 0; 150; 200; 200; 200];
      end        

      if isfield(options,'Kd')
        typecheck(options.Kd,'double');
        sizecheck(options.Kd,[6 1]);
        obj.Kd = options.Kd;
      else
        obj.Kd = [0; 0; 50; 70; 70; 70];
      end        
        
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.001;
      end
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate
      

      if isfield(options,'use_mex')
        sizecheck(options.use_mex,1);
        obj.use_mex = uint32(options.use_mex);
        rangecheck(obj.use_mex,0,2);
        if (obj.use_mex && exist('pelvisMotionControlmex','file')~=3)
          error('can''t find pelvisMotionControlmex.  did you build it?');
        end
      else
        obj.use_mex = 1;
      end

      obj.alpha = 0.95;
      obj.nominal_pelvis_height = 0.75;
      obj.robot = r;

      if (obj.use_mex>0)
        obj.mex_ptr = SharedDataHandle(pelvisMotionControlmex(0,obj.robot.getMexModelPtr.ptr,obj.alpha,obj.nominal_pelvis_height,obj.Kp,obj.Kd));
      end

      obj.body_ind = findLinkInd(r,name);
      obj.lfoot_ind = findLinkInd(r,'l_foot');
      obj.rfoot_ind = findLinkInd(r,'r_foot');
    end
   
    function y=output(obj,t,~,x)
      if (obj.use_mex == 0)
        persistent z_prev
        q = x(1:obj.nq);
        qd = x(obj.nq+1:end);
        kinsol = doKinematics(obj.robot,q,false,true,qd); 
      
        % TODO: this must be updated to use quaternions/spatial velocity
        [p,J] = forwardKin(obj.robot,kinsol,obj.body_ind,[0;0;0],1); 
        
        % terrible hack
        lfoot = forwardKin(obj.robot,kinsol,obj.lfoot_ind,[0;0;0],1);
        rfoot = forwardKin(obj.robot,kinsol,obj.rfoot_ind,[0;0;0],1);
        
        if isempty(z_prev)
          z_prev = p(3);
        end
        z_des = obj.alpha*z_prev + (1-obj.alpha)*(min([lfoot(3),rfoot(3)])+obj.nominal_pelvis_height); % X cm above feet
        z_prev = z_des;
        
        body_des = [nan;nan;z_des;0;0;mean([lfoot(6) rfoot(6)])]; 
        err = [body_des(1:3)-p(1:3);angleDiff(p(4:end),body_des(4:end))];

        body_vdot = obj.Kp.*err - obj.Kd.*(J*qd);
      else
        body_vdot = pelvisMotionControlmex(obj.mex_ptr.data,x);  
      end
      y = [obj.body_ind;body_vdot];
    end
  end
  
end
