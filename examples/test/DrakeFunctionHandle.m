classdef DrakeFunctionHandle < drakeFunction.DrakeFunction
  properties
    fun
  end
  methods
    function obj = DrakeFunctionHandle(num_in,num_out,fun)
      input_frame = CoordinateFrame('input_frame',num_in);
      output_frame = CoordinateFrame('output_frame',num_out);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.fun = fun;
    end
    
    function [f,df,ddf] = eval(obj,x)
      [f,df,ddf] = obj.fun(x);
    end
    
  end
  
end