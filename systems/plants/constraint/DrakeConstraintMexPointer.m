classdef DrakeConstraintMexPointer < DrakeMexPointer
% This would be used to determine if a mex pointer refers to a RigidBodyConstraint
% object
    
  methods 
    function obj = DrakeConstraintMexPointer(ptr, delete_fcn, name)
      obj = obj@DrakeMexPointer(ptr,delete_fcn,name);
    end
  end
end