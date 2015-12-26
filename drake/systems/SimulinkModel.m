classdef SimulinkModel < DynamicalSystem
% Implements the DynamicalSystem interface for any existing simulink model
% with one (vector) input and one (vector) output

  % constructor
  methods
    function obj = SimulinkModel(mdl,num_u)
      % Construct a simulink model DynamicalSystem
      %
      % @param mdl a string containing the name of an existing simulink model

      if isa(mdl,'SimulinkModelHandle')
        obj.mdl = mdl;
        mdl = mdl.name;
      else
        load_system(mdl);
        obj.mdl = SimulinkModelHandle(mdl);
      end
      
      set_param(mdl,'InheritedTsInSrcMsg','none');
      set_param(mdl,'SolverPrmCheckMsg','none');
      
%      feval(mdl,[],[],[],'compile');  % seemed to be working ok (except
%      bug 1022) before having this.  so keeping it out.
      sys = feval(mdl,[],[],[],'sizes');
      obj.num_xc = sys(1);
      obj.num_xd = sys(2);
      obj.num_x = obj.num_xc+obj.num_xd;
      obj.num_y = sys(3);
      if (nargin<2)
        obj.num_u = sys(4);
      else
        obj.num_u = num_u;  % temporary hack to get around bug 1022
      end

      obj=setInputFrame(obj,CoordinateFrame([mdl,'Input'],obj.num_u,'u'));
      obj=setStateFrame(obj,CoordinateFrame([mdl,'State'],obj.num_xc+obj.num_xd,'x'));
      obj=setOutputFrame(obj,CoordinateFrame([mdl,'Output'],obj.num_y,'y'));
      obj=setParamFrame(obj,CoordinateFrame([mdl,'Params'],0,'p'));
      
      if (~strcmp(get_param(obj.mdl,'SimulationStatus'),'paused'))
        set_param(obj.mdl,'StopTime',num2str(inf));
        feval(obj.mdl,[],[],[],'compile');
      end
    end
  end
  
  % default methods - these should be implemented or overwritten
  % 
  methods
    function n = getNumContStates(obj)
      n = obj.num_xc;
    end
    function n = getNumDiscStates(obj)
      n = obj.num_xd;
    end
    function n = getNumInputs(obj)
      n = obj.num_u;
    end
    function n = getNumOutputs(obj)
      n = obj.num_y;
    end
    function ts = getSampleTime(obj)
      [sys,x0,str,ts] = feval(obj.mdl.name,[],[],[],'sizes');
      ts = ts'; 
    end
    function mdl = getModel(obj)
      mdl = obj.mdl;
    end
    
    function x0 = getInitialState(obj)
      if ~isempty(obj.initial_state)
        x0 = obj.initial_state;
        return;
      end
      if (getNumStates(obj)>0)
        x0 = Simulink.BlockDiagram.getInitialState(obj.mdl);
        x0 = stateStructureToVector(obj,x0);
      else
        x0 = [];
      end
    end
    
    function [xcdot,df] = dynamics(obj,t,x,u,yes_i_promise_that_i_called_output_first)
      x = stateVectorToStructure(obj,x);
      if (~strcmp(get_param(obj.mdl,'SimulationStatus'),'paused'))
        feval(obj.mdl,[],[],[],'compile');
      end

%      feval(obj.mdl,[],[],[],'all');   % can't see this in the documentation.. don't know where I got it from?

      if (nargin<5 || ~yes_i_promise_that_i_called_output_first)
        feval(obj.mdl,t,x,u,'outputs'); % have to call this before derivs (see email thread with mathworks in bug 695)
      end
      xcdot = feval(obj.mdl,t,x,u,'derivs');
      
      xcdot = stateStructureToVector(obj,xcdot);

      if (nargout>1)
        [A,B] = linearize(obj,t,x,u);
        df = [zeros(obj.num_xc,1), A, B];
      end
      
    end
    
    function xdn = update(obj,t,x,u,yes_i_promise_that_i_called_output_first)
      x = stateVectorToStructure(obj,x);
      if (~strcmp(get_param(obj.mdl,'SimulationStatus'),'paused'))
        set_param(obj.mdl,'StopTime',num2str(inf));
        feval(obj.mdl,[],[],[],'compile');
      end

%      feval(obj.mdl,[],[],[],'all');   % can't see this in the documentation.. don't know where I got it from?

      if (nargin<5 || ~yes_i_promise_that_i_called_output_first)
        feval(obj.mdl,t,x,u,'outputs'); % have to call this before derivs (see email thread with mathworks in bug 695)
      end
      xdn = feval(obj.mdl,t,x,u,'update');
      xdn = stateStructureToVector(obj,xdn);
    end
    
    function [y,dy] = output(obj,t,x,u)
      if (obj.num_x>0)  % speeds things up, since the call from inside DynamicalSystem is virtual
        x = stateVectorToStructure(obj,x);
      else
        x=[];
      end
      if (~strcmp(get_param(obj.mdl,'SimulationStatus'),'paused'))
        set_param(obj.mdl,'StopTime',num2str(inf));
        feval(obj.mdl,[],[],[],'compile');
      end

%      feval(obj.mdl,[],[],[],'all');   % can't see this in the documentation.. don't know where I got it from?

      y = feval(obj.mdl,t,x,u,'outputs');
      
      if ~isnumeric(y) % why is y sometimes numeric, and sometimes a struct?
        y = y.signals.values(:);
      end
      
      if (nargout>1)
        [A,B,C,D] = linearize(obj,t,x,u);  % should it ever be dlinearize?
        dy = [zeros(obj.num_y,1), C, D];
      end
    end
    
  end

  properties (SetAccess=private, GetAccess=private)
    num_xc; % number of continuous state variables
    num_xd; % number of dicrete(-time) state variables
    num_x;  % dimension of x (= num_xc + num_xd)
    num_u;  % dimension of u
    num_y;  % dimension of the output y
    mdl;    % the SimulinkModelHandle
  end
  
  
end
