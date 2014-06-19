classdef QTrajEvalBlock < MIMODrakeSystem
% NOTEST
  % passes through the robot state and
  % reads evals qtraj at the current t
  properties
    robot;
    controller_data;
    dt;
    use_error_integrator;
  end
  
  methods
    function obj = QTrajEvalBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      
      ctrl_data = getData(controller_data);
      if ~isfield(ctrl_data,'qtraj')
        error('QTrajEvalBlock: controller_data must contain qtraj field');
      end
      
      if nargin<3
        options = struct();
      end

      if isfield(options,'use_error_integrator')
        typecheck(options.use_error_integrator,'logical');
        if options.use_error_integrator
          if ~isfield(ctrl_data,'integral') || ~isfield(ctrl_data,'integral_gains')
            error('controller_data must contain integral and integral_gains variables');
          else
            typecheck(ctrl_data.integral,'double');
            typecheck(ctrl_data.integral_gains,'double');
            sizecheck(ctrl_data.integral_gains,[getNumDOF(r) 1]);
          end
        end
      else
        options.use_error_integrator = false;
      end
      
      atlas_state = getStateFrame(r);
      input_frame = atlas_state;
      output_frame = MultiCoordinateFrame({AtlasCoordinates(r),atlas_state});
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.003;
      end
      obj = setSampleTime(obj,[obj.dt;0]);
      
      obj.robot = r;
      obj.controller_data = controller_data;
      obj.use_error_integrator = options.use_error_integrator;
    end
       
    function [qdes,x]=mimoOutput(obj,t,~,x)
      qtraj = obj.controller_data.data.qtraj;
      if isa(qtraj,'double')
        qdes=qtraj;
      else
        % pp trajectory
        qdes = fasteval(qtraj,t);
      end
      if obj.use_error_integrator
        q = x(1:end/2);
        i_clamp = obj.controller_data.data.integral_clamps;
        newintg = obj.controller_data.data.integral + obj.controller_data.data.integral_gains.*(qdes-q)*obj.dt;
        newintg = max(-i_clamp,min(i_clamp,newintg));
        setField(obj.controller_data,'integral', newintg);
        qdes = qdes + newintg;
        [jlmin,jlmax] = getJointLimits(obj.robot);
        qdes = max(jlmin-i_clamp,min(jlmax+i_clamp,qdes)); % allow it to go delta above and below jlims
      end
    end
  end
  
end
