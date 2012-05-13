classdef CoordinateTransform < RobotLibSystem 
    
  methods
    function obj=CoordinateTransform(from,to)
      obj=obj@RobotLibSystem(0,0,0,0,false,false);
      obj.input_frame = from;
      obj.output_frame = to;
    end
  end
end
