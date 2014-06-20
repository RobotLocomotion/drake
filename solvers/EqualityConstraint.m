classdef EqualityConstraint < BoundingBoxConstraint
  % enforce an equality constraint, x = desired_value
  
  methods
    function obj = EqualityConstraint(desired_value)
      obj = obj@BoundingBoxConstraint(desired_value,desired_value);
    end
  end
  
end
  
  