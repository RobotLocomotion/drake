classdef FixedPointProgram < NonlinearProgram

  properties
    sys % the drake system
  end
  
  methods
    function obj = FixedPointProgram(sys,x_dimensions_to_ignore)
      % @param sys a DrakeSystem
      % @param x_dimensions_to_ignore if this is specified then xdot need
      % not be zero in the ignored dimensions (e.g. useful for finding a
      % trim condition of an aircraft instead of a true fixed point)
      
      typecheck(sys,'DrakeSystem');
      if ~isTI(sys), error('only makes sense for time invariant systems'); end
      if nargin<2, x_dimensions_to_ignore = []; end
      
      num_x = getNumStates(sys);
      num_u = getNumInputs(sys);
      obj = obj@NonlinearProgram(num_x+num_u,vertcat(getCoordinateNames(sys.getStateFrame), getCoordinateNames(sys.getInputFrame)));
      obj.sys = sys;
      obj = addDynamicConstraints(obj,x_dimensions_to_ignore);
      
      obj = sys.addStateConstraintsToProgram(obj,1:num_x);
      obj = sys.addInputConstraintsToProgram(obj,num_x+(1:num_u));
    end
    
    function obj = addDynamicConstraints(obj,x_dimensions_to_ignore)
      sys = obj.sys;
      
      num_x = getNumStates(sys);
      num_u = getNumInputs(sys);
      num_xc = getNumContStates(sys);
      num_xd = getNumDiscStates(sys);

      xc_indices = 1:num_xc;  xc_indices(x_dimensions_to_ignore(x_dimensions_to_ignore<num_xc))=[];
      num_xc_constraints = numel(xc_indices);
        
      function [c,dc] = dynamics_constraints(w)
        x=w(1:num_x);
        u=w(num_x + (1:num_u));
        [c,dc] = geval(@sys.dynamics,0,x,u);
        c = c(xc_indices);
        dc = dc(xc_indices,2:end);
      end
      
      if (num_xc>0)
        obj = addConstraint(obj,FunctionHandleConstraint(zeros(num_xc_constraints,1),zeros(num_xc_constraints,1),num_x+num_u,@dynamics_constraints));
      end
      
      xd_indices = 1:num_xd;  xd_indices(x_dimensions_to_ignore(x_dimensions_to_ignore>num_xc)-num_xc)=[];
      num_xd_constraints = numel(xd_indices);
      
      function [c,dc] = update_constraints(w)
        x=w(1:num_x);
        u=w(num_x + (1:num_u));
        [c,dc] = geval(@sys.update,0,x,u);
        c = c - x;
        dc = dc(:,2:end) - [eye(num_x),zeros(num_x,num_u)];
        c = c(xd_indices);
        dc = dc(xd_indices,:);
      end
      
      if (num_xd>0)
        obj = addConstraint(obj,FunctionHandleConstraint(zeros(num_xd_constraints,1),zeros(num_xd_constraints,1),num_x+num_u,@update_constraints));
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