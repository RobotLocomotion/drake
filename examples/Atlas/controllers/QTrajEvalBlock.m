classdef QTrajEvalBlock < MIMODrakeSystem
  % passes through the robot state and
  % reads evals qtraj at the current t
  properties
    robot;
    controller_data;
    dt;
    use_error_integrator;
    jlmin;
    jlmax;
    nq;
  end
  
  methods
    function obj = QTrajEvalBlock(r,controller_data,options)
      typecheck(r,'Biped');
      typecheck(controller_data,'ControllerData');
      
      if nargin<3
        options = struct();
      end

      if isfield(options,'use_error_integrator')
        typecheck(options.use_error_integrator,'logical');
        if options.use_error_integrator
          sizecheck(controller_data.integral_gains,[getNumPositions(r) 1]);
          typecheck(controller_data,{'AtlasManipControllerData','AtlasQPControllerData'});
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
        obj.dt = 0.001;
      end
      obj = setSampleTime(obj,[obj.dt;0]);
      
      obj.robot = r;
      obj.controller_data = controller_data;
      obj.use_error_integrator = options.use_error_integrator;
      [obj.jlmin,obj.jlmax] = getJointLimits(obj.robot);
      obj.nq = getNumPositions(r);

    end
       
    function [qdes,x]=mimoOutput(obj,t,~,x)
      qtraj = obj.controller_data.qtraj;
      if isa(qtraj,'double')
        qdes=qtraj;
      elseif isa(qtraj,'struct')
        % ppform
        qdes = ppval(qtraj,min(t,qtraj.breaks(end)));
      else
        % pp trajectory
        qdes = fasteval(qtraj,t);
      end
      if obj.use_error_integrator
        q = x(1:obj.nq);
        i_clamp = obj.controller_data.integral_clamps;
        newintg = obj.controller_data.integral + obj.controller_data.integral_gains.*(qdes-q)*obj.dt;
        newintg = max(-i_clamp,min(i_clamp,newintg));
        obj.controller_data.integral = newintg;
        qdes = qdes + newintg;
        qdes = max(obj.jlmin-i_clamp,min(obj.jlmax+i_clamp,qdes)); % allow it to go delta above and below jlims
      end
    end
  end
  
end
