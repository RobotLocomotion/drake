classdef SmoothRobotLibSystem < RobotLibSystem
% A RobotLibSystem where dynamics, update, and output functions are 
% smooth (continuous with continuous first derivatives) functions of t,x,u
  
  % constructor
  methods
    function obj = SmoothRobotLibSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag)
      % Construct a SmoothRobotLibSystem
      %
      % @param num_xc number of continuous-time state variables
      % @param num_xd number of discrete-time state variables
      % @param num_u number of inputs
      % @param num_y number of outputs
      % @param direct_feedthrough_flag true means that the output depends
      %   directly on the input.  Set to false if possible.
      % @param time_invariant_flag true means that the
      %   dynamics/update/output do not depend on time.  Set to true if
      %   possible.

      obj = obj@RobotLibSystem(num_xc,num_xd,num_u,num_y);
      if (nargin>=4 && ~isempty(num_y)) obj = setNumOutputs(obj,num_y); end
      if (nargin>=5 && ~isempty(direct_feedthrough_flag)) obj = setDirectFeedthrough(obj,direct_feedthrough_flag); end
      if (nargin>=6 && ~isempty(time_invariant_flag)) obj = setTIFlag(obj,time_invariant_flag); end
    end      
  end
      
  % utility methods
  methods
    function [A,B,C,D,x0dot,y0] = linearize(obj,t0,x0,u0)
      if (~isCT(obj) || getNumDiscStates(obj)>0)  % boot if it's not the simple case
        [A,B,C,D,x0dot,y0] = linearize@DynamicalSystem(obj,t0,x0,u0);
      end
      
      nX = getNumContStates(obj);
      nU = getNumInputs(obj);
      [f,df] = geval(@obj.dynamics,t0,x0,u0);
      A = df(:,1+(1:nX));
      B = df(:,nX+1+(1:nU));
      
      if (nargout>2)
        [y,dy] = geval(@obj.output,t0,x0,u0);
        C = dy(:,1+(1:nX));
        D = dy(:,nX+1+(1:nU));
        if (nargout>4)
          x0dot = dynamics(obj,t0,x0,u0);
          if (nargout>5)
            y0 = output(obj,t0,x0,u0);
          end
        end
      end
    end

    function traj = simulateODE(obj,tspan,x0,options)
      % Simulates the system using the ODE45 suite of solvers
      % instead of the simulink solvers.  
      %
      % @param tspan a 1x2 vector of the form [t0 tf]
      % @param x0 a vector of length(getNumStates) which contains the initial
      % state
      % @param options options structure
      %
      % No options implemented yet

      if (nargin<3) x0=getInitialState(obj); end
      
      if (obj.num_xcon>0) warning('system has constraints, but they are not explicitly satisfied during simulation (yet - it should be an easy fix in the ode suite)'); end
      
      odeoptions = obj.simulink_params;
      odefun = @(t,x)obj.dynamics(t,x,zeros(obj.getNumInputs(),1));
      if (isfield(obj.simulink_params,'Solver'))
        sol = feval(obj.simulink_params.Solver,odefun,tspan,x0,odeoptions);
      else
        sol = ode45(odefun,tspan,x0,odeoptions);
      end
      xtraj = ODESolTrajectory(sol);
      traj = FunctionHandleTrajectory(@(t)obj.output(t,xtraj.eval(t),zeros(obj.getNumInputs(),1)),[obj.getNumOutputs,1],tspan);
    end
    
    function sys=feedback(sys1,sys2,options)
      % Constructs a feedback combination of sys1 and sys2.  
      % Tries to keep the system as a SmoothRobotLibSystem, but will fail
      % if one of the systems has discontinuities, for instance from an
      % input saturation.  In that case, it returns a DynamicalSystem
      % object.
      %
      % @param sys1 first DynamicalSystem (on the forward path)
      % @param sys2 second DynamicalSystem (on the backward path)
      % @option try_to_be_robotlibsystem set to false if you want to return
      % a simulink model. @default true
      %
      % The input to the feedback model is added to the output of sys2
      % before becoming the input for sys1.  The output of the feedback
      % model is the output of sys1.
      try 
        sys=FeedbackSystem(sys1,sys2);  % try to keep it a smoothrobotlibsystem
      catch
        sys=feedback@DynamicalSystem(sys1,sys2);
      end
    end
    
    function sys=cascade(sys1,sys2)
      try
        sys=CascadeSystem(sys1,sys2);   % try to keep it a smoothrobotlibsystem 
      catch
        sys=cascade@DynamicalSystem(sys1,sys2);
      end
    end
    
  end
    

end
