classdef SimulinkModelHandle < handle
% a utility class to make it convenient to make a piece of data behave like
% a handle class
%  example usage:
%      mex_model_ptr = SharedDataHandle(mex_model_ptr);
%    now mex_model_ptr continues to act like it did before, but
%    will internally be accessing the shared data structure. 

  properties
    name;
    base_workspace_variables={};
  end
  
  methods 
    function obj = SimulinkModelHandle(mdl)
      obj.name = mdl;
    end
    
    function delete(obj)
%      disp(['closing ',obj.name]);
      try
        close_system(obj.name,0);
      catch
        feval(obj.name,[],[],[],'term');
        close_system(obj.name,0);
      end
      
      for i=1:length(obj.base_workspace_variables)
        count = evalin('base',[obj.base_workspace_variables{i},'_c']);
        if count<=1
%          disp(['deleting ',obj.base_workspace_variables{i}]);
          evalin('base',['clear ',obj.base_workspace_variables{i},' ',obj.base_workspace_variables{i},'_c;']); 
        else
          evalin('base',[obj.base_workspace_variables{i},'_c=',num2str(count-1),';']);
        end
      end
    end
    
    function base_workspace_variable_name = registerParameter(obj,var,simple_name)
      % to share data with the simulink model, it needs to be copied to the
      % base workspace.  this sets up a simple garbage collector.
      % it's slightly non-trivial, because if the contents of the model are
      % copied to a different model, I have to keep track of how many
      % references are left.

      base_workspace_variable_name = [obj.name,'_',simple_name];
      if length(base_workspace_variable_name)>=60
        % MATLAB has a maximum name length of 63 characters
        base_workspace_variable_name = base_workspace_variable_name(end-59:end);
      end
      if evalin('base',['exist(''',base_workspace_variable_name,''',''var'');'])
        % make it unique
        base_workspace_variable_name = registerParameter(obj,var,[simple_name,'_',datestr(now,'MMSSFFF')]);
        return
      end
      
      assignin('base',base_workspace_variable_name,var);
      assignin('base',[base_workspace_variable_name,'_c'],1);
      
      obj.base_workspace_variables{end+1}=base_workspace_variable_name;
    end
    
    function inheritParameters(obj,from_mdl)
      for i=1:length(from_mdl.base_workspace_variables)
        evalin('base',[from_mdl.base_workspace_variables{i},'_c=',from_mdl.base_workspace_variables{i},'_c+1;']);
      end
      obj.base_workspace_variables = horzcat(obj.base_workspace_variables,from_mdl.base_workspace_variables);
    end
    
    function addSubsystem(obj,subsystem_name,subsystem_mdl)
      load_system('simulink3');
      name = [obj.name,'/',subsystem_name];
      add_block('simulink3/Subsystems/Subsystem',name);
      Simulink.SubSystem.deleteContents(name);
      Simulink.BlockDiagram.copyContentsToSubSystem(subsystem_mdl.name,name);
      obj.inheritParameters(subsystem_mdl);
    end
    
  end
  
  
  
  % overload methods so I can treat it like the simulink model name (which
  % I was using everywhere before)
  methods
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
    
    function varargout = feval(obj,varargin)
      if nargout>0
        varargout=cell(1,nargout);
        varargout{:} = feval(obj.name,varargin{:});
      else
        feval(obj.name,varargin{:});
      end
    end
    
  end
end