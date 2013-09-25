classdef DrakeConstraintMexPointer < DrakeMexPointer
% This would be used to determine if a mex pointer refers to a Constraint
% object
    
  methods 
    function obj = DrakeConstraintMexPointer(ptr, delete_fcn, name)
      obj = obj@DrakeMexPointer(ptr,delete_fcn,name);
    end
  end
end