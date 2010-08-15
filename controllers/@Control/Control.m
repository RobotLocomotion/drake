classdef Control 
% An abstract class that provides the control interface and support routines.
  
  methods
    function obj = Control(num_x,num_u)
      % Construct new controller which expects num_x and outputs num_u.
      obj = setNumX(obj,num_x);
      obj = setNumU(obj,num_u);
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
    
    function obj = setNumX(obj,num_x)
      % Guards the number of feedback state variables
      if (num_x<0) error('num_x must be >=0'); end
      obj.num_x = num_x;
    end
        
    function obj = setNumU(obj,num_u)
      % Guards the number of dynamics inputs (outputs of the controller)
      if (num_u<1) error('num_u must be >0'); end
      obj.num_u = num_u;
    end

    
  end
    
  properties (SetAccess=protected)
    num_x;      % the number of dynamics states (feedback inputs to controller)
    num_u;      % the number of dynamics inputs (outputs of the controller)
    control_dt = 0;  % zero for ct control
  end
  
end