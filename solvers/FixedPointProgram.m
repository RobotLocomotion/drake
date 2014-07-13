classdef FixedPointProgram < NonlinearProgramWConstraintObjects

  properties
    sys % the drake system
  end
  
  methods
    function obj = FixedPointProgram(sys)
      typecheck(sys,'DrakeSystem');
      if ~isTI(sys), error('only makes sense for time invariant systems'); end

      num_x = getNumStates(sys);
      num_u = getNumInputs(sys);
      obj = obj@NonlinearProgramWConstraintObjects(num_x+num_u,vertcat(getCoordinateNames(sys.getStateFrame), getCoordinateNames(sys.getInputFrame)));
      obj.sys = sys;
      obj = addDynamicConstraints(obj);
      
      function [c,dc] = state_constraints(x)
        [c,dc] = geval(@sys.stateConstraints,x);
      end
        
      if (sys.getNumStateConstraints>0)
        num_xcon = sys.getNumStateConstraints();
        obj = addConstraint(obj,FunctionHandleConstraint(zeros(num_xcon,1),zeros(num_xcon,1),num_x,@state_constraints),1:num_x);
      end
      
      function [c,dc] = unilateral_constraints(x)
        [c,dc] = geval(@sys.unilateralConstraints,x);
      end
      
      if (sys.getNumUnilateralConstraints > 0)
        nc = sys.getNumUnilateralConstraints;
        obj = addConstraint(obj,FunctionHandleConstraint(zeros(nc,1),inf(nc,1),num_x,@unilateral_constraints),1:sys.num_x);
      end
    end
    
    function obj = addDynamicConstraints(obj)
      sys = obj.sys;
      
      num_x = getNumStates(sys);
      num_u = getNumInputs(sys);
      num_xc = getNumContStates(sys);
      num_xd = getNumDiscStates(sys);
      function [c,dc] = dynamics_constraints(w)
        x=w(1:num_x);
        u=w(num_x + (1:num_u));
        [c,dc] = geval(@sys.dynamics,0,x,u);
        dc = dc(:,2:end);
      end
      
      if (num_xc>0)
        obj = addConstraint(obj,FunctionHandleConstraint(zeros(num_xc,1),zeros(num_xc,1),num_x+num_u,@dynamics_constraints));
      end
      
      function [c,dc] = update_constraints(w)
        x=w(1:num_x);
        u=w(num_x + (1:num_u));
        [c,dc] = geval(@sys.update,0,x,u);
        c = c - x;
        dc = dc(:,2:end) - [eye(num_x),zeros(num_x,num_u)];
      end
      
      if (num_xd>0)
        obj = addConstraint(obj,FunctionHandleConstraint(zeros(num_xd,1),zeros(num_xd,1),num_x+num_u,@update_constraints));
      end
    end
    
    function [xstar,ustar,info] = findFixedPoint(obj,x0,u0)
      if isa(x0,'Point')
        x0 = double(x0.inFrame(obj.getStateFrame));
      end
      if isa(u0,'Point')
        u0 = double(u0.inFrame(obj.getInputFrame));
      end
      w0 = [x0;u0];
      [wstar,info] = solve(obj,w0);
      xstar = Point(obj.sys.getStateFrame,wstar(1:length(x0)));
      ustar = Point(obj.sys.getInputFrame,wstar(length(x0)+(1:length(u0))));
    end
  end

end