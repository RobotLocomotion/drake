classdef SharedDataHandleMutable < handle
% a utility class to make it convenient to make a piece of data behave like
% a handle class.  this version permits using setData() as well as getData().
%  example usage:
%      mex_model_ptr = SharedDataHandle(mex_model_ptr);
%    now mex_model_ptr continues to act like it did before, but
%    will internally be accessing the shared data structure. 
  
  properties (SetAccess=private,GetAccess=private)
    data
    deleteFcn=[];
  end
  
  methods 
    function obj = SharedDataHandleMutable(data, deleteFcn)
      obj.data = data;
      if (nargin>1)
        typecheck(deleteFcn,'function_handle');
%        valuecheck(nargin(deleteFcn),1);
        obj.deleteFcn = deleteFcn;
      end
    end
    
    function data = getData(obj)
      data = obj.data;
    end
    
    function setData(obj, newData)
      obj.data = newData;
    end
    
    function delete(obj)
      if ~isempty(obj.deleteFcn)
        feval(obj.deleteFcn,obj.data);
      end
    end
  end
end
