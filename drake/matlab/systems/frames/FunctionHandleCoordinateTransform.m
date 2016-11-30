classdef FunctionHandleCoordinateTransform < FunctionHandleSystem & CoordinateTransform
  
  methods
    function obj = FunctionHandleCoordinateTransform(num_xc,num_xd,from,to,direct_feedthrough_flag,time_invariant_flag,hDynamics,hUpdate,hOutput)
      typecheck(from,'CoordinateFrame');
      typecheck(to,'CoordinateFrame');
      obj = obj@FunctionHandleSystem(num_xc,num_xd,from.dim,to.dim,direct_feedthrough_flag,time_invariant_flag,hDynamics,hUpdate,hOutput);
      obj = obj@CoordinateTransform(from,to,direct_feedthrough_flag,time_invariant_flag);
    end
  end
end