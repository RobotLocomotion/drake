classdef FunctionHandleVisualizer < Visualizer
  % visualizer class where the draw methods is provided by a function
  % handle
  
  properties
    draw_fcn;
  end
  
  methods 
    function obj = FunctionHandleVisualizer(input_frame,draw_fcn)
      % @param draw_fcn must take draw_fcn(t,y)
      obj = obj@Visualizer(input_frame);
      typecheck(draw_fcn,'function_handle');
      obj.draw_fcn = draw_fcn;
    end

    function draw(obj,t,y)
      % overloads the draw method and calls the function handle
      obj.draw_fcn(t,y);
    end
  end
end
