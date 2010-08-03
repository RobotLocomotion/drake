classdef Control 

  methods
    function obj = Control(num_states,num_inputs)
      obj = setNumStates(obj,num_states);
      obj = setNumInputs(obj,num_inputs);
    end
  
  % methods that MUST be implemented
  methods (Abstract=true)
    u = control(obj,t,x);
  end
  
  % methods that CAN be implemented/overridden
  methods
    function du = controlGradients(obj,t,x,order)
      error('control gradients not implemented yet');
    end
    
    function obj = setNumStates(obj,num_states)
      % setNumStates
      %   a simple guard since inconsistencies can arise if the number of
      %   states is changed after construction
      if (num_states<0) error('num_states must be >=0'); end
      obj.num_states = num_states;
    end
        
    function obj = setNumInputs(obj,num_inputs)
      % setNumInputs
      %   a simple guard since inconsistencies can arise if the number of
      %   inputs is changed after construction. 
      if (num_inputs<1) error('num_inputs must be >0'); end
      obj.num_inputs = num_inputs;
    end

    
  end
    
  properties (SetAccess=protected)
    num_states
    num_inputs
    control_dt = 0;  % zero for ct control
  end
  
  
end