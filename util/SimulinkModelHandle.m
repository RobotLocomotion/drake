classdef SimulinkModelHandle < handle
% a utility class to make it convenient to make a piece of data behave like
% a handle class
%  example usage:
%      mex_model_ptr = SharedDataHandle(mex_model_ptr);
%    now mex_model_ptr continues to act like it did before, but
%    will internally be accessing the shared data structure. 

  properties (SetAccess=private,GetAccess=public)
    additional_delete_fcn=[];
  end
  
  properties
    name;
  end
  
  methods 
    function obj = SimulinkModelHandle(mdl, additional_delete_fcn)
      obj.name = mdl;
      if (nargin>1)
        typecheck(additional_delete_fcn,'function_handle');
        obj.additional_delete_fcn = additional_delete_fcn;
      end
    end
    
    function delete(obj)
      close_system(obj.name,0);
      if ~isempty(obj.additional_delete_fcn)
        feval(obj.additional_delete_fcn,obj.name);
      end
    end
    
    function str = horzcat(varargin)
      % the model name is often used in string concatenations to build out
      % other models
      for i=1:numel(varargin)
        if isa(varargin{i},'SimulinkModelHandle')
          varargin{i} = varargin{i}.name;
        end
      end
      str = horzcat(varargin{:});
    end
    
    function varargout = add_line(obj,varargin)
      varargout=cell(1,max(nargout,1));
      varargout{:} = add_line(obj.name,varargin{:});
    end
    
    function retval = get_param(obj,varargin)
      retval = get_param(obj.name,varargin{:});
    end
    
    function set_param(obj,varargin)
      set_param(obj.name,varargin{:});
    end
    
    function varargout = sim(obj,varargin)
      varargout=cell(1,max(nargout,1));
      varargout{:} = sim(obj.name,varargin{:});
    end
    
    function varargout = find_system(obj,varargin)
      varargout=cell(1,max(nargout,1));
      varargout{:} = find_system(obj.name,varargin{:});
    end
    
  end
end