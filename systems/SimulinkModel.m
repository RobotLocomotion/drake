classdef SimulinkModel < DynamicalSystem
% Implements the DynamicalSystem interface for any existing simulink model
% with one (vector) input and one (vector) output

  % constructor
  methods
    function obj = SimulinkModel(mdl)
      % Construct a simulink model DynamicalSystem
      %
      % @param mdl a string containing the name of an existing simulink model
      
      load_system(mdl);
      obj.mdl = mdl;
      sys = feval(mdl,[],[],[],'sizes');
      obj.num_xc = sys(1);
      obj.num_xd = sys(2);
      obj.num_y = sys(3);
      obj.num_u = sys(4);

      obj=obj.setInputFrame(CoordinateFrame([mdl,'Input'],obj.num_u,'u'));
      obj=obj.setStateFrame(CoordinateFrame([mdl,'State'],obj.num_xc+obj.num_xd,'x'));
      obj=obj.setOutputFrame(CoordinateFrame([mdl,'Output'],obj.num_y,'y'));
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
      [sys,x0,str,ts] = feval(obj.mdl,[],[],[],'sizes');
    end
    function mdl = getModel(obj)
      mdl = obj.mdl;
    end
    
    function x0 = getInitialState(obj)
      x0 = Simulink.BlockDiagram.getInitialState(obj.mdl);
      x0 = stateStructureToVector(obj,x0);
    end
    
    function [xcdot,df] = dynamics(obj,t,x,u)
      x = stateVectorToStructure(obj,x);
      if (~strcmp(get_param(obj.mdl,'SimulationStatus'),'paused'))
        feval(obj.mdl,[],[],[],'compile');
      end
      feval(obj.mdl,t,x,u,'outputs'); % have to call this before derivs (see email thread with mathworks in bug 695)
      xcdot = feval(obj.mdl,t,x,u,'derivs');
      
      xcdot = stateStructureToVector(obj,xcdot);

      if (nargout>1)
        [A,B] = linearize(obj,t,x,u);
        df = [zeros(obj.num_xc,1), A, B];
      end
      
    end
    
    function xdn = update(obj,t,x,u)
      x = stateVectorToStructure(obj,x);
      if (~strcmp(get_param(obj.mdl,'SimulationStatus'),'paused'))
        feval(obj.mdl,[],[],[],'compile');
      end
      feval(obj.mdl,t,x,u,'outputs'); % have to call this before derivs (see email thread with mathworks in bug 695)
      xdn = feval(obj.mdl,t,x,u,'update');
      xdn = stateStructureToVector(obj,xdn);
    end
    
    function [y,dy] = output(obj,t,x,u)
      x = stateVectorToStructure(obj,x);
      if (~strcmp(get_param(obj.mdl,'SimulationStatus'),'paused'))
        feval(obj.mdl,[],[],[],'compile');
      end
      y = feval(obj.mdl,t,x,u,'outputs');

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
    mdl;    % a string name for the simulink model
  end
  
  
end
