classdef QTrajEvalBlock < MIMODrakeSystem
  % passes through the robot state and
  % reads evals qtraj at the current t
  properties
    robot;
    controller_data;
    dt;
  end
  
  methods
    function obj = QTrajEvalBlock(r,controller_data,options)
      typecheck(r,'Biped');
      typecheck(controller_data,'ControllerData');
      
      if nargin<3
        options = struct();
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
    end
  end
  
end
