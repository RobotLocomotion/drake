classdef Observer < DrakeSystem
  % An interface class that sets up a system as an observer, or
  % state-estimator, for an existing system.  
  % The inputs of the observer are the inputs and (noisy) outputs of the model
  % system, and the output is the estimated state of the observed system. 
  
  properties
    forward_model
  end
  
  methods
    function obj = Observer(sys_to_observe,num_xc,num_xd,direct_feedthrough_flag,time_invariant_flag)
      typecheck(sys_to_observe,'DrakeSystem');  % todo:  shouldn't be hard to generalize this to any dynamical system
      if (nargin<2), num_xc=0; end
      if (nargin<3), num_xd=0; end
      if (nargin<4), direct_feedthrough_flag = true; end
      if (nargin<5), time_invariant_flag = false; end
      obj = obj@DrakeSystem(num_xc,num_xd,getNumInputs(sys_to_observe)+getNumOutputs(sys_to_observe),getNumStates(sys_to_observe),direct_feedthrough_flag,time_invariant_flag);

      % setup input/output frames
      obj = setInputFrame(obj,MultiCoordinateFrame.constructFrame({getInputFrame(sys_to_observe),getOutputFrame(sys_to_observe)}));
      obj = setOutputFrame(obj,getStateFrame(sys_to_observe));
      
      obj = setSampleTime(obj,getSampleTime(sys_to_observe));
    end
    
    function sys = constructErrorSystem(obj)
      sys = ObserverErrorSystem(obj)
    end
    
    function x0 = getInitialState(obj)
        x0 = zeros(obj.num_xc + obj.num_xd, 1);
    end
  end
  
end