classdef ClosedLoopObserverErrorSystem < DrakeSystem
  % Creates a system that is the result of the feedback combination of the
  % plant, observer, and controller, with the output of the system being
  % the estimation error of the observer
  
  properties
    observer
    controller
  end
  
  methods
    function obj = ClosedLoopObserverErrorSystem(observer,controller)
      typecheck(observer,'Observer');
      typecheck(controller,'DrakeSystem');
      
      controller = inInputFrame(controller,getStateFrame(observer.forward_model));
      controller = inOutputFrame(controller,getInputFrame(observer.forward_model));
      
      if isa(observer,'HybridDrakeSystem') || isa(observer.forward_model,'HybridDrakeSystem') || isa(controller,'HybridDrakeSystem')
        error('Drake:ClosedLoopObserverErrorSystem:NoHybridSupport','hybrid systems not implemented yet.');
      end
      
      if isa(observer,'StochasticDrakeSystem') || isa(observer.forward_model,'StochasticDrakeSystem') || isa(controller,'StochasticDrakeSystem')
        warning('Drake:ClosedLoopObserverErrorSystem:LimitedStochasticSupport','Your observer, forward model, and/or controller are stochastic, but so far the error system just analyzes the deterministic dynamics (with w=0).  This is obviously something important to finish.');
      end
      
      assert(~isDirectFeedthrough(observer) || ~isDirectFeedthrough(controller));
      
      obj = obj@DrakeSystem(...
        getNumContStates(observer)+getNumContStates(observer.forward_model)+getNumContStates(controller), ...
        getNumDiscStates(observer)+getNumDiscStates(observer.forward_model)+getNumDicStates(controller), ...
        0, ...
        getNumStates(observer.forward_model), ...
        false, ...
        isTI(observer) && isTI(observer.forward_model) && isTI(controller));
      
      obj = setOutputFrame(obj,getStateFrame(observer.forward_model));

      obj = setStateFrame(obj,MultiCoordinateFrame.constructFrame( ...
        { getStateFrame(observer.forward_model),getStateFrame(observer),getStateFrame(controller) }, ...
        [ ones(getNumContStates(observer.forward_model),1); ...
          2*ones(getNumContStates(observer),1); ...
          3*ones(getNumContStates(controller),1); ...
          ones(getNumDiscStates(observer.forward_model),1); ...
          2*ones(getNumDiscStates(observer),1); ...
          3*ones(getNumDiscStates(controller),1) ] ) );
      
      assert( getStateFrame(observer.forward_model) == getOutputFrame(observer) );
      % todo: assert that the input to the observer is [u;y]
      
      obj = setSampleTime(obj,[getSampleTime(observer),getSampleTime(observer.forward_model),getSampleTime(controller)]);

      obj.observer = observer;
      obj.controller = controller;
    end
    
    
    function xcdot = dynamics(obj,t,x,~)
      [x_model,x_observer,x_controller,u,y,xhat_model] = computePrerequisites(obj,t,x);

      xcdot = [ dynamics(obj.observer.forward_model,t,x_model,u); ...
        dynamics(obj.observer,t,x_observer,[u;y]); ...
        dynamics(obj.controller,t,x_controller,xhat_model) ];
    end
    
    function xdn = update(obj,t,x,~)
      [x_model,x_observer,x_controller,u,y,xhat_model] = computePrerequisites(obj,t,x);
      
      xdn = [ update(obj.observer.forward_model,t,x_model,u); ...
        update(obj.observer,t,x_observer,[u;y]); ... 
        update(obj.controller,t,x_controller,xhat_model) ];
    end
    
    function xerr = output(obj,t,x,~)
      [x_model,~,~,~,~,xhat_model] = computePrerequisites(obj,t,x);
      xerr = x_model - xhat_model;
    end
  end
  
  methods (Access=private)
    function [x_model,x_observer,x_controller,u,y,xhat_model] = computePrerequisites(obj,t,x)
      [x_model,x_observer,x_controller] = splitCoordinates(getStateFrame(obj),x);

      if ~isDirectFeedthrough(obj.observer)
        xhat_model = output(obj.observer,t,x_observer);
        u = output(obj.controller,t,x_controller,xhat_model);
        y = output(obj.observer.forward_model,t,x_model,u);
      else % controller must not be feedthrough
        u = output(obj.controller,t,x_controller);
        y = output(obj.observer.forward_model,t,x_model,u);
        xhat_model = output(obj.observer,t,x_observer,[u;y]);
      end
    end
  end
  
end