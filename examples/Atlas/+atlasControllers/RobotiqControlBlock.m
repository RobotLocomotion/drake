classdef RobotiqControlBlock < MIMODrakeSystem

	properties
  end
  
  methods
    function obj = RobotiqControlBlock(r, ind, name, options)
      % @param r ts rigid body manipulator instance with hands
      % @param ind Index of hand -- depends on order they were added to r
      % @param name A name to indentify this hand
      % @param options struct
      % @option nothing yet -- nothing yet

      typecheck(r,'TimeSteppingRigidBodyManipulator');
       
      if nargin<4
        options = struct();
      end
      
      input_frame = r.getOutputFrame.getFrameByName([name, 'atlasFrames.HandState']);
      output_frame = r.getInputFrame.getFrameByName([name, 'atlasFrames.HandInput']);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
			
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.001;
      end
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate
      
    end
   
    function y=mimoOutput(obj,t,~,x)      
      % Input is hand state, output is hand inputs
      % For now just issue time-varying force
      outFrame = getOutputFrame(obj);
      y = 0.5*cos(6*t)*ones(outFrame.dim, 1);
		end
  end
  
end
