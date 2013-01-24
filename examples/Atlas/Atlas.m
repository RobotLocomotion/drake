classdef Atlas < TimeSteppingRigidBodyManipulator
  
  methods
    
    function obj=Atlas(urdf,options)
      typecheck(urdf,'char');

      if nargin < 2
        options = struct();
      end
      if ~isfield(options,'dt')
        options.dt = 0.001;
      end
      if ~isfield(options,'floating')
        options.floating = true;
      end
      
      addpath(fullfile(pwd,'frames'));
  
      obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      
      if options.floating
        % could also do fixed point search here
        obj = obj.setInitialState(obj.manip.resolveConstraints(zeros(obj.getNumStates(),1)));
      else
        % TEMP HACK to get by resolveConstraints
        for i=1:length(obj.manip.body), obj.manip.body(i).contact_pts=[]; end
        obj.manip = compile(obj.manip);
        obj = obj.setInitialState(zeros(obj.getNumStates(),1));
      end
    end
    
    function obj = compile(obj)
      obj = compile@TimeSteppingRigidBodyManipulator(obj);

      state_frame = AtlasState(obj);
      obj = obj.setStateFrame(state_frame);
      obj = obj.setOutputFrame(state_frame);
    
      input_frame = AtlasInput(obj);
      obj = obj.setInputFrame(input_frame);
    end

    function obj = setInitialState(obj,x0)
      typecheck(x0,'double');
      sizecheck(x0,obj.getNumStates());
      obj.x0 = x0;
    end
    
    function x0 = getInitialState(obj)
      x0 = obj.x0;
    end    
  end
  
  properties (SetAccess = protected, GetAccess = public)
    x0
  end
end
