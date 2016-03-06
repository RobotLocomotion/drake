classdef Reduction < DynamicalSystem
  
  properties
    original_sys
    reduced_sys
  end

  methods (Abstract=true)  % These must be implemented
    u_red = inputMapping(obj,u_orig);   % do any of these need to depend on other vars?  e.g., state-dependent input mapping?
    x_red = stateMapping(obj,x_orig);
    y_orig = outputMapping(obj,y_red);
  end
  
  methods % These should be implemented if possible
    function u_orig = inputInvMapping(obj,u_red)
      error('not implemented');
    end
    function x_orig = stateInvMapping(obj,x_red)
      error('not implemented');
    end
    function y_red = outputInvMapping(obj,y_orig)
      error('not implemented');
    end  
  end
  
  methods % worker methods
    function obj = Reduction(original_sys, reduced_sys)
      typecheck(original_sys,'DynamicalSystem');
      typecheck(reduced_sys,'DynamicalSystem');
      obj.original_sys = original_sys;
      obj.reduced_sys = reduced_sys;
    end

    function n = getNumContStates(obj)
      n = obj.reduced_sys.getNumContStates();
    end
    function n = getNumDiscStates(obj)
      n = obj.reduced_sys.getNumDiscStates();
    end
    function n = getNumInputs(obj)
      n = obj.original_sys.getNumInputs();
    end
    function n = getNumOutputs(obj)
      n = obj.original_sys.getNumOutputs();
    end
    function ts = getSampleTime(obj)
      ts = obj.reduced_sys.getSampleTime();
    end
    function mdl = getModel(obj)
      error('not implemented yet');
      % just need to cascade the static transformations with the reduced
      % system
    end
    function x0 = getInitialState(obj)
      if ~isempty(obj.initial_state)
        x0 = obj.initial_state;
        return;
      end

      x0 = obj.reduced_sys.getInitialState();
    end

    function xcdot = dynamics(obj,t,x,u)
      xcdot = dynamics(obj.reduced_sys,t,x,obj.inputMapping(u));
    end
    function xdn = update(obj,t,x,u)
      xdn = dynamics(obj.reduced_sys,t,x,obj.inputMapping(u));
    end
    function y = output(obj,t,x,u)
      y = outputMapping(obj,output(obj.reduced_sys,t,x,obj.inputMapping(u));
    end
  end
end
