classdef FunctionHandleSystem < DrakeSystem

properties (SetAccess=private)
  hDynamics
  hUpdate
  hOutput
end


methods
  function obj = FunctionHandleSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag,hDynamics,hUpdate,hOutput)
    obj = obj@DrakeSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag);
    obj.hDynamics = hDynamics;
    obj.hUpdate = hUpdate;
    obj.hOutput = hOutput;
  end
  
  function xcdot = dynamics(obj,t,x,u)
    xcdot = feval(obj.hDynamics,t,x,u);
  end
    
  function xdn = update(obj,t,x,u)
    xdn = feval(obj.hUpdate,t,x,u);
  end
  
  function y = output(obj,t,x,u)
    y = feval(obj.hOutput,t,x,u);
  end
end

end
