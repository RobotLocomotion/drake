classdef DrakeMexPointer < handle
% My attempt to clean-up the mex-matlab sharing pointer interface
% 
%      mex_ptr = DrakeMexPtr(mexFunction(initialization_data));
%      mexFunction(mex_ptr.ptr,other_data);

  properties (SetAccess=private,GetAccess=public)
    ptr
    delete_fcn_ptr=0;
  end
    
  methods 
    function obj = DrakeMexPointer(ptr, delete_fcn_ptr)
      obj.ptr = ptr;
      if (nargin>1)
        obj.delete_fcn_ptr = delete_fcn_ptr;
      end
    end
    
    function delete(obj)
      % todo: write a mex function that just calls delete
    end
  end
end