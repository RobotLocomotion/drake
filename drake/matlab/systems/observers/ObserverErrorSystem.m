classdef ObserverErrorSystem < DrakeSystem
  % The input of this system is the input to observed system, and the
  % output is the error in the estimate of the state of the observed system
  
  properties
    observer
  end
  
  methods
    function obj = ObserverErrorSystem(observer)
      typecheck(observer,'Observer');
      
      if isa(observer,'HybridDrakeSystem') || isa(observer.forward_model,'HybridDrakeSystem')
        error('Drake:ObserverErrorSystem:NoHybridSupport','hybrid systems not implemented yet.');
      end
      
      if isa(observer,'StochasticDrakeSystem') || isa(observer.forward_model,'StochasticDrakeSystem')
        warning('Drake:ObserverErrorSystem:LimitedStochasticSupport','Your observer and/or forward model are stochastic, but so far the error system just analyzes the deterministic dynamics (with w=0).  This is obviously something important to finish.');
      end
      
      obj = obj@DrakeSystem(...
        getNumContStates(observer)+getNumContStates(observer.forward_model), ...
        getNumDiscStates(observer)+getNumDiscStates(observer.forward_model), ...
        getNumInputs(observer.forward_model), ...
        getNumStates(observer.forward_model), ...
        true, ...
        isTI(observer) && isTI(observer.forward_model));
      
      obj = setInputFrame(obj,getInputFrame(observer.forward_model));
      obj = setOutputFrame(obj,getStateFrame(observer.forward_model));

      obj = setStateFrame(obj,MultiCoordinateFrame.constructFrame( ...
        { getStateFrame(observer.forward_model),getStateFrame(observer) }, ...
        [ ones(getNumContStates(observer.forward_model),1); ...
          2*ones(getNumContStates(observer),1); ...
          ones(getNumDiscStates(observer.forward_model),1); ...
          2*ones(getNumDiscStates(observer),1) ] ) );
      
      assert( getStateFrame(observer.forward_model) == getOutputFrame(observer) );
      % todo: assert that the input to the observer is [u;y]
      
      obj = setSampleTime(obj,[getSampleTime(observer),getSampleTime(observer.forward_model)]);

      obj.observer = observer;
    end
    
    function xcdot = dynamics(obj,t,x,u)
      [x_model,x_observer] = splitCoordinates(getStateFrame(obj),x);

      y = output(obj.observer.forward_model,t,x_model,u);
      
      xcdot = [ dynamics(obj.observer.forward_model,t,x_model,u); ...
        dynamics(obj.observer,t,x_observer,[u;y]) ];
    end
    
    function xdn = update(obj,t,x,u)
      [x_model,x_observer] = splitCoordinates(getStateFrame(obj),x);

      y = output(obj.observer.forward_model,t,x_model,u);
      
      xdn = [ update(obj.observer.forward_model,t,x_model,u); ...
        update(obj.observer,t,x_observer,[u;y]) ];
    end
    
    function y = output(obj,t,x,u)
      [x_model,x_observer] = splitCoordinates(getStateFrame(obj),x);

      y = output(obj.observer.forward_model,t,x_model,u);
      xhat_model = output(obj.observer,t,x_observer,[u;y]);
      
      y = x_model - xhat_model;
    end
  end
  
end