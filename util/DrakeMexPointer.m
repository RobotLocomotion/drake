classdef DrakeMexPointer < handle
% My attempt to clean-up the mex-matlab sharing pointer interface
% 
%      mex_ptr = DrakeMexPtr(mexFunction(initialization_data));
%      mexFunction(mex_ptr.ptr,other_data);

  properties (SetAccess=private,GetAccess=public)
    name = '';
    ptr
    delete_fcn='';
  end
    
  methods 
    function obj = DrakeMexPointer(ptr, delete_fcn, name)
      obj.ptr = ptr;
      if (nargin>1) obj.delete_fcn = delete_fcn; end
      if (nargin>2) obj.name = name; end
    end
    
    function delete(obj)
      if 0 %~isempty(obj.delete_fcn)  % useful for debugging
        fprintf(1,'Calling %s to delete ', obj.delete_fcn);
        if isempty(obj.name)
          fprintf(1,'unnamed drake mex pointer\n');
        else
          fprintf(1,'drake mex pointer %s\n',obj.name);
        end
        feval(obj.delete_fcn,obj);
      end
    end
  end
end