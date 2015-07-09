classdef AcrobotPlantCpp < AcrobotPlant
% Overloads the matlab implementation of the manipulator dynamics with a
% c++ implementation.  this is intended as a demonstration (and test) of
% how we can mix matlab and c++ implementations.

  methods
    function f = dynamics(obj,t,x,u)
      f = dynamics@Manipulator(obj,t,x,u);
    end
  end

end