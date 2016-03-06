classdef SharedDataHandle < handle
% a utility class to make it convenient to make a piece of data behave like
% a handle class
%  example usage:
%      mex_model_ptr = SharedDataHandle(mex_model_ptr);
%    now mex_model_ptr continues to act like it did before, but
%    will internally be accessing the shared data structure. 

  properties (SetAccess=private,GetAccess=public)
    deleteFcn=[];
  end
  
  properties
    data;
  end
  
  methods 
    function obj = SharedDataHandle(data, deleteFcn)
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
    
    function delete(obj)
      if ~isempty(obj.deleteFcn)
        feval(obj.deleteFcn,obj.data);
      end
    end
    
    function setData(obj,data)
      obj.data = data;
    end
    
    function setField(obj,fieldname,val)
 %     typecheck(obj.data,'struct');
      obj.data.(fieldname)=val;
    end
        
    function d=getField(obj,fieldname)  % recommend using obj.data.fieldname instead
%      typecheck(obj.data,'struct');
      d = obj.data.(fieldname);
    end
  end
end