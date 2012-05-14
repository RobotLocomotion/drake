classdef RobotLibSystem < DynamicalSystem
% A DynamicalSystem with the functionality (dynamics, update, outputs, 
% etc) implemented in matlab, so that it is amenable to, for instance, symbolic
% manipulations.  These functions are wrapped as an S-Function in
% RLCSFunction.cpp.

  % constructor
  methods
    function obj = RobotLibSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag)
      % Construct a RobotLibSystem
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
      
      obj.uid = datestr(now,'MMSSFFF');

      if (nargin>0)
        obj = setNumContStates(obj,num_xc);
        obj = setNumDiscStates(obj,num_xd);
        obj = setNumInputs(obj,num_u);
        if (nargin>=4) obj = setNumOutputs(obj,num_y); end
        if (nargin>=5) obj = setDirectFeedthrough(obj,direct_feedthrough_flag); end
        if (nargin>=6) obj = setTIFlag(obj,time_invariant_flag); end
        if (num_u) obj=setInputFrame(obj,CoordinateFrame([class(obj),'Input'],num_u,'u')); end
        if (num_xc+num_xd) obj=setStateFrame(obj,CoordinateFrame([class(obj),'State'],num_xc+num_xd,'x')); end
        if (num_y) obj=setOutputFrame(obj,CoordinateFrame([class(obj),'Output'],num_y,'y')); end
      end
      
    end      
  end
  
  % default methods - these should be implemented or overwritten
  % 
  methods
    function x0 = getInitialState(obj)
      x0 = zeros(obj.num_xd+obj.num_xc,1);
    end
    
    function xcdot = dynamics(obj,t,x,u)
      error('RobotLib:RobotLibSystem:AbstractMethod','systems with continuous states must implement Derivatives (ie overload dynamics function)');
    end
    
    function xdn = update(obj,t,x,u)
      error('RobotLib:RobotLibSystem:AbstractMethod','systems with discrete states must implement Update (ie overload update function)');
    end
    
    function y = output(obj,t,x,u)
      error('RobotLib:RobotLibSystem:AbstractMethod','default is intentionally not implemented');
    end
    
    function zcs = zeroCrossings(obj,t,x,u)
      error('RobotLib:RobotLibSystem:AbstractMethod','systems with zero crossings must implement the zeroCrossings method'); 
    end
    
    function con = stateConstraints(obj,x)
      error('RobotLib:RobotLibSystem:AbstractMethod','systems with state constraints must implement the constraints method');
    end
  end
  
  % access methods
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
    function x0 = getInitialStateWInput(obj,t,x,u)  
      % Hook in case a system needs to initial state based on current time and/or input.  
      % This gets called after getInitialState(), and unfortunately will override inputs supplied by simset.
      x0=x;  % by default, do nothing. 
    end
    function ts = getSampleTime(obj)  
      % As described at http://www.mathworks.com/help/toolbox/simulink/sfg/f6-58760.html
      % to set multiple sample times, specify one *column* for each sample
      % time/offset pair.
      % The default behavior is continuous time for systems with only continuous
      % states, and discrete time (with sample period 1s) for systems only
      % discrete states, and inherited for systems with no states.  For
      % systems with both discrete and continuous states, an error is
      % thrown saying that this function should be overloaded to set the
      % desired behavior.  

      if ~isempty(obj.ts)
        ts = obj.ts;
      elseif (obj.num_xc>0 && obj.num_xd==0)
        ts = [0;0];  % continuous time, no offset
      elseif (obj.num_xc==0 && obj.num_xd>0)
        ts = [1;0];  % discrete time, with period 1s.
      elseif (obj.num_xc==0 && obj.num_xd==0)
        ts = [-1;0]; % inherited sample time
      else
        error('RobotLib:RobotLibSystem:NotImplemented','systems with both discrete and continuous states must implement the getSampleTime method or call setSampleTime to specify the desired behavior');
      end
    end
    function obj = setSampleTime(obj,ts)
      % robust method for setting default sample time
      % 
      % @param ts a 2-by-n matrix with each column containing a sample time
      %    redundant colums are eliminated automatically.  
      ts = unique(ts','rows')';

      % only a few possibilities are allowed/supported
      %   inherited, single continuous, single discrete, single continuous+single
      %   discrete
      if size(ts,2)>1  % if only one ts, then all is well
        if any(ts(1,:)==-1)  % zap superfluous inherited
          ts=ts(:,find(ts(1,:)~=-1));
        end
        if sum(ts(1,:)==0)>1 % then multiple continuous
          error('cannot define a robotlibsystem using modes that have both ''continuous time'' and ''continuous time, fixed in minor offset'' sample times');
        end
        if sum(ts(1,:)>0)>1 % then multiple discrete
          error('cannot define a robotlibsystem using modes that have different discrete sample times');
        end
      end
      obj.ts = ts;
    end
    function tf = isDirectFeedthrough(obj)
      % Check if the system is direct feedthrough (e.g., if the output
      % depends on the immediate input)
      tf = obj.direct_feedthrough_flag;
    end
    function obj = setDirectFeedthrough(obj,tf);
      % Set the direct feedthrough flag
      obj.direct_feedthrough_flag = tf;
    end
    function mdl = getModel(obj)
      % make a simulink model from this block
      mdl = [class(obj),'_',obj.uid];  % use the class name + uid as the model name
      close_system(mdl,0);  % close it if there is an instance already open
      new_system(mdl,'Model');
      set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
      
      assignin('base',[mdl,'_obj'],obj);
      
      load_system('simulink');
      load_system('simulink3');
      add_block('simulink/User-Defined Functions/S-Function',[mdl,'/RobotLibSys'], ...
        'FunctionName','RLCSFunction', ...
        'parameters',[mdl,'_obj']);
      if (getNumInputs(obj)>0)
        add_block('simulink3/Sources/In1',[mdl,'/in']);
        
        if (any(~isinf([obj.umin,obj.umax]))) % then add saturation block
          add_block('simulink3/Nonlinear/Saturation',[mdl,'/sat'],...
            'UpperLimit',mat2str(obj.umax),'LowerLimit',mat2str(obj.umin));
          add_line(mdl,'in/1','sat/1');
          add_line(mdl,'sat/1','RobotLibSys/1');
        else
          add_line(mdl,'in/1','RobotLibSys/1');
        end
      end
      if (getNumOutputs(obj)>0)
        add_block('simulink3/Sinks/Out1',[mdl,'/out']);
        add_line(mdl,'RobotLibSys/1','out/1');
      end
      
      if (obj.num_xcon>0)
        warning('RobotLib:RobotLibSystem:NotImplemented','system has constraints, but they aren''t enforced in the simulink model yet.');
      end
    end
    
    function x = resolveConstraints(obj,x0,v)
      % attempts to find a x which satisfies the constraints,
      % using x0 as the initial guess.
      %
      % @param x0 initial guess for state satisfying constraints
      % @param v (optional) a visualizer that should be called while the
      % solver is doing it's thing

      if (obj.num_con < 1)
        x=x0;
        return;
      end
      
      function stop=drawme(x,optimValues,state)
        stop=false;
        v.draw(0,x);
      end
        
      if (nargin>1 && ~isempty(v))  % useful for debugging (only but only works for URDF manipulators)
        options=optimset('Display','iter','Algorithm','levenberg-marquardt','OutputFcn',@drawme,'TolX',1e-9);
      else
        options=optimset('Display','off','Algorithm','levenberg-marquardt');
      end
      x = fsolve(@(x)stateConstraints(obj,x),x0,options);      
    end
  end  
  
  % access methods
  methods
    function u = getDefaultInput(obj)
      % Define the default initial input so that behavior is well-defined
      % if no controller is specified or if no control messages have been
      % received yet.
      u = zeros(obj.num_u,1);
    end
    function obj = setNumContStates(obj,num_xc)
      % Guards the num_states variable
      if (num_xc<0) error('num_xc must be >= 0'); end
      obj.num_xc = num_xc;
      obj.num_x = obj.num_xd + obj.num_xc;
    end
    function obj = setNumDiscStates(obj,num_xd)
      % Guards the num_states variable
      if (num_xd<0) error('num_xd must be >= 0'); end
      obj.num_xd = num_xd;
      obj.num_x = obj.num_xc + obj.num_xd;
    end
    function obj = setNumInputs(obj,num_u)
      % Guards the num_u variable.
      %  Also pads umin and umax for any new inputs with [-inf,inf].

      if (num_u<0) error('num_u must be >=0 or DYNAMICALLY_SIZED'); end
      
       % cut umin and umax to the right size, and pad new inputs with
      % [-inf,inf]
      if (length(obj.umin)~=num_u)
        obj.umin = [obj.umin; -inf*ones(max(num_u-length(obj.umin),0),1)];
      end
      if (length(obj.umax)~=num_u)
        obj.umax = [obj.umax; inf*ones(max(num_u-length(obj.umax),0),1)];
      end
      
      obj.num_u = num_u;
    end
    function obj = setInputLimits(obj,umin,umax)
      % Guards the input limits to make sure it stay consistent
      
      if (isscalar(umin)) umin=repmat(umin,obj.num_u,1); end
      if (isscalar(umax)) umax=repmat(umax,obj.num_u,1); end
      
      sizecheck(umin,[obj.num_u,1]);
      sizecheck(umax,[obj.num_u,1]);
      if (any(obj.umax<obj.umin)) error('umin must be less than umax'); end
      obj.umin = umin;
      obj.umax = umax;
    end
    function obj = setNumOutputs(obj,num_y)
      % Guards the number of outputs to make sure it's consistent
      if (num_y<0) error('num_y must be >=0'); end
      obj.num_y = num_y;
    end
    function n = getNumZeroCrossings(obj)
      % Returns the number of zero crossings
      n = obj.num_zcs;
    end
    function obj = setNumZeroCrossings(obj,num_zcs)
      % Guards the number of zero crossings to make sure it's valid.
      if (num_zcs<0) error('num_zcs must be >=0'); end
      obj.num_zcs = num_zcs;
    end
    function n = getNumStateConstraints(obj)
      % Returns the number of zero crossings
      n = obj.num_xcon;
    end
    function obj = setNumStateConstraints(obj,num_xcon)
      % Guards the number of zero crossings to make sure it's valid.
      if (num_xcon<0) error('num_xcon must be >=0'); end
      obj.num_xcon = num_xcon;
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
      
      if (obj.num_zcs>0) warning('system has zero-crossings, but i havne''t passed them to ode45 yet.  (should be trivial)'); end
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
        sys=FeedbackSystem(sys1,sys2);  % try to keep it a robotlibsystem
      catch
        sys=feedback@DynamicalSystem(sys1,sys2);
      end
    end
    
    function sys=cascade(sys1,sys2)
      try
        sys=CascadeSystem(sys1,sys2);   % try to keep it a robotlibsystem 
      catch
        sys=cascade@DynamicalSystem(sys1,sys2);
      end
    end
    
  end
  
  % utility methods
  methods
    function gradTest(obj)
      % Compare numerical and analytical derivatives of dynamics,update,and
      % output
      if (getNumContStates(obj))
        gradTest(@obj.dynamics,0,getInitialState(obj),getDefaultInput(obj),struct('tol',.01))
      end
      if (getNumDiscStates(obj))
        gradTest(@obj.update,0,getInitialState(obj),getDefaultInput(obj),struct('tol',.01))
      end
      if (getNumOutputs(obj))
        gradTest(@obj.output,0,getInitialState(obj),getDefaultInput(obj),struct('tol',.01))
      end
    end
  end
  
  properties (SetAccess=private, GetAccess=protected)
    num_xc=0; % number of continuous state variables
    num_xd=0; % number of dicrete(-time) state variables
    num_x=0;  % dimension of x (= num_xc + num_xd)
    num_u=0;  % dimension of u
    num_y=0;  % dimension of the output y
    num_zcs = 0;  % number of zero-crossings.  @default: 0
    num_xcon = 0; % number of state constraints. @default: 0
    uid;    % unique identifier for simulink models of this block instance
    direct_feedthrough_flag=true;  % true/false: does the output depend on u?  set false if you can!
    ts=[];    % default sample times of the model
  end
  properties (SetAccess=private, GetAccess=public)
    umin=[];   % constrains u>=umin (default umin=-inf)
    umax=[];    % constrains u<=uman (default umax=inf)
  end
  
end
