classdef Control 
% An abstract class that provides the control interface and support routines.
  
  methods
    function obj = Control(num_states,num_inputs)
      % Construct new controller which expects num_states and outputs num_inputs.
      obj = setNumStates(obj,num_states);
      obj = setNumInputs(obj,num_inputs);
    end
  end
  
  % methods that MUST be implemented
  methods (Abstract=true)
    u = control(obj,t,x);
  end
  
  % methods that CAN be implemented/overridden
  methods
    function du = controlGradients(obj,t,x,order)
      % Computes the Taylor-expansion of the control function.
      error('control gradients not implemented yet');
    end
    
    function obj = setNumStates(obj,num_states)
      % Guards the number of feedback state variables
      if (num_states<0) error('num_states must be >=0'); end
      obj.num_states = num_states;
    end
        
    function obj = setNumInputs(obj,num_inputs)
      % Guards the number of dynamics inputs (outputs of the controller)
      if (num_inputs<1) error('num_inputs must be >0'); end
      obj.num_inputs = num_inputs;
    end

    
  end
    
  properties (SetAccess=protected)
    num_states;      % the number of dynamics states (feedback inputs to controller)
    num_inputs;      % the number of dynamics inputs (outputs of the controller)
    control_dt = 0;  % zero for ct control
  end
  
  
end