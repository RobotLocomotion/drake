classdef DrakeMexPointer < handle
% My attempt to clean-up the mex-matlab sharing pointer interface
%
%      mex_ptr = myMexFunction(...);  // the mex function calls createDrakeMexPointer
%      myMexFunctions(mex_ptr,other_data);  // these call getDrakeMexPointer

  properties (SetAccess=private,GetAccess=public)
    ptr
    name = '';
    type_id = -1;
  end
  properties (SetAccess=private,GetAccess=private)
    delete_fcn='';
    delete_fcn_additional_inputs={};
  end

  methods
    function obj = DrakeMexPointer(ptr, delete_fcn, name, type_id, varargin)
      obj.ptr = ptr;
      if (nargin>1) obj.delete_fcn = delete_fcn; end
      if (nargin>2) obj.name = name; end
      if (nargin>3) obj.type_id = type_id; end
      if (nargin>4)
        obj.delete_fcn_additional_inputs=varargin;
      end
    end

    function delete(obj)
      if 0 %~isempty(obj.delete_fcn)  % useful for debugging
        fprintf(1,'Calling %s(',obj.delete_fcn);
        celldisp(obj.delete_fcn_additional_inputs);
        fprintf(1,') to delete ');
        if isempty(obj.name)
          fprintf(1,'unnamed drake mex pointer\n');
        else
          fprintf(1,'drake mex pointer %s\n',obj.name);
        end
      end
      if ~isempty(obj.delete_fcn)
        feval(obj.delete_fcn,obj.delete_fcn_additional_inputs{:},obj);
      end
    end
  end

  methods (Static)

    function obj = loadobj(obj)
      obj.delete_fcn = '';
      obj = 0;
    end

  end
end
