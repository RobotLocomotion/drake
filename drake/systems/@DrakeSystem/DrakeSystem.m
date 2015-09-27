classdef DrakeSystem < DynamicalSystem
% A DynamicalSystem with the functionality (dynamics, update, outputs,
% etc) implemented in matlab, so that it is amenable to, for instance, symbolic
% manipulations.  These functions are wrapped as an S-Function in
% DCSFunction.cpp.

  % constructor
  methods
    function obj = DrakeSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag)
      % Construct a DrakeSystem
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

      if (nargin>0)
        obj = setNumContStates(obj,num_xc);
        obj = setNumDiscStates(obj,num_xd);
        obj = setNumInputs(obj,num_u);
        if (nargin>=4), obj = setNumOutputs(obj,num_y); else obj = setNumOutputs(obj,0); end
        if (nargin>=5), obj = setDirectFeedthrough(obj,direct_feedthrough_flag); end
        if (nargin>=6), obj = setTIFlag(obj,time_invariant_flag); end
      end
      obj = setParamFrame(obj,CoordinateFrame([class(obj),'Params'],0,'p'));  % no parameters by default
    end
  end

  % default methods - these should be implemented or overwritten
  %
  methods
    function x0 = getInitialState(obj)
      % Return a (potentially random) state double (column)
      % vector of initial conditions
      %
      % Attempts to return the result of resolveConstraints using a
      % small random vector as an initial seed.

      if ~isempty(obj.initial_state)
        x0 = obj.initial_state;
        return;
      end
      
      x0 = .01*randn(obj.num_xd+obj.num_xc,1);
      if ~isempty(obj.state_constraints)
        attempts=0;
        success=false;
        while (~success)
          attempts=attempts+1;
          try
            [x0,success] = resolveConstraints(obj,x0);
          catch ex
            if strcmp(ex.identifier,'Drake:DrakeSystem:FailedToResolveConstraints');
              success=false;
            else
              rethrow(ex);
            end
          end
          if (~success)
            x0 = randn(obj.num_xd+obj.num_xc,1);
            if (attempts>=10)
              error('Drake:DrakeSystem:FailedToResolveConstraints','Failed to resolve state constraints on initial conditions after 10 tries');
            end
          end
        end
      end
      x0 = double(x0);
    end

    function xcdot = dynamics(obj,t,x,u)
      % Placeholder for the dynamics method.  Systems with continuous state
      % must overload this method.
      error('Drake:DrakeSystem:AbstractMethod','systems with continuous states must implement Derivatives (ie overload dynamics function)');
    end

    function xdn = update(obj,t,x,u)
      % Placeholder for the update method.  Systems with discrete state
      % must overload this method.
      error('Drake:DrakeSystem:AbstractMethod','systems with discrete states must implement Update (ie overload update function)');
    end

    function y = output(obj,t,x,u)
      % Placeholder for the output method.  Systems must overload this method.
      error('Drake:DrakeSystem:AbstractMethod','default is intentionally not implemented');
    end

    function zcs = zeroCrossings(obj,t,x,u)
      % Placeholder for the zeroCrossings method: a method
      % phi = zeroCrossings(t,x,u) which triggers a zero crossing
      % event when phi transitions from positive to negative.
      %
      % Systems with zero crossings must overload this method.
      error('Drake:DrakeSystem:AbstractMethod','systems with zero crossings must implement the zeroCrossings method');
    end

  end

  % access methods
  methods
    function n = getNumContStates(obj)
      % Returns the number of continuous states
      n = obj.num_xc;
    end
    function n = getNumDiscStates(obj)
      % Returns the number of discrete states
      n = obj.num_xd;
    end
    function n = getNumInputs(obj)
      % Returns the number of inputs to the system
      n = obj.num_u;
    end
    function n = getNumOutputs(obj)
      % Returns the number of outputs from the system
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
        error('Drake:DrakeSystem:NotImplemented','systems with both discrete and continuous states must implement the getSampleTime method or call setSampleTime to specify the desired behavior');
      end
    end

  end

  methods (Sealed = true)
    function ts = getInputSampleTimes(obj)
      % Returns getSampleTime - a DrakeSystem can only have a single same
      % time associated with it.
      ts = getSampleTime(obj);
    end
    function ts = getOutputSampleTimes(obj)
      % Returns getSampleTime - a DrakeSystem can only have a single same
      % time associated with it.
      ts = getSampleTime(obj);
    end
  end

  methods
    function obj = setSampleTime(obj,ts)
      % robust method for setting default sample time
      %
      % @param ts a 2-by-n matrix with each column containing a sample time
      %    redundant colums are eliminated automatically.
      % only a few possibilities are allowed/supported
      %   inherited, single continuous, single discrete, single continuous+single
      %   discrete (note: disabled single continuous + single discrete
      %   because it wasn't obviously the right thing... e.g. in the
      %   visualizer who asked for the output to be at fixed dt, but after
      %   combination, the output gets called continuously).

      ts = unique(ts','rows')';

      if size(ts,2)>1  % if only one ts, then all is well
        if any(ts(1,:)==-1)  % zap superfluous inherited
          ts=ts(:,ts(1,:)~=-1);
        end
        if sum(ts(1,:)>0)>1 % then multiple discrete
          error('Drake:DrakeSystem:UnsupportedSampleTime','cannot define a drakesystem using modes that have different discrete sample times');
        end
        if sum(ts(1,:)==0)>1 % then multiple continuous
          error('Drake:DrakeSystem:UnsupportedSampleTime','cannot define a drakesystem using modes that have both ''continuous time'' and ''continuous time, fixed in minor offset'' sample times');
        end
        if sum(ts(1,:)>=0)>1 % then both continuous and discrete
          error('Drake:DrakeSystem:UnsupportedSampleTime','cannot define a drakesystem using modes that have both continuous and discrete sample times');
        end
      end
      obj.ts = ts;
    end
    function tf = isDirectFeedthrough(obj)
      % Check if the system is direct feedthrough (e.g., if the output
      % depends on the immediate input)
      tf = obj.direct_feedthrough_flag;
    end
    function obj = setDirectFeedthrough(obj,tf)
      % Set the direct feedthrough flag
      obj.direct_feedthrough_flag = tf;
    end
    function mdl = getModel(obj)
      % Constructs a simulink system block for this system to be used by
      % the simulink engine.
      %
      % @retval mdl string id for the simulink system

      % First, make sure we have a compiled DCSFunction
      if(~exist('DCSFunction','file'))
        errorMsg={'Sorry, you have not run ''make'' yet in the drake root,'
          'which means you do not have the compiled MEX files needed to run this program.'
          'Running configure and make in the drake root directory will fix this.'};
        error('%s\n',errorMsg{:})
      end
      
      uid = datestr(now,'MMSSFFF');
      
      % make a simulink model from this block
      mdl = [class(obj),'_',uid];  % use the class name + uid as the model name
      mdl = regexprep(mdl, '\.', '_'); % take any dots out to make it a valid Matlab function
      mdl = mdl(1:min(59,length(mdl))); % truncate the name so that simulink won't throw a warning about it being too long
      new_system(mdl,'Model');
      set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
      mdl = SimulinkModelHandle(mdl);
      
      load_system('simulink');
      load_system('simulink3');
      add_block('simulink/User-Defined Functions/S-Function',[mdl,'/DrakeSys'], ...
        'FunctionName','DCSFunction', ...
        'parameters',registerParameter(mdl,obj,'DrakeSystem'));
      
      m = Simulink.Mask.create([mdl,'/DrakeSys']);
      m.set('Display',['fprintf(''',class(obj),''')']);
      
      if (getNumInputs(obj)>0)
        add_block('simulink3/Sources/In1',[mdl,'/in']);
        
        if (any(~isinf([obj.umin,obj.umax]))) % then add saturation block
          add_block('simulink3/Nonlinear/Saturation',[mdl,'/sat'],...
            'UpperLimit',mat2str(obj.umax),'LowerLimit',mat2str(obj.umin));
          add_line(mdl,'in/1','sat/1');
          add_line(mdl,'sat/1','DrakeSys/1');
        else
          add_line(mdl,'in/1','DrakeSys/1');
        end
      end
      if (getNumOutputs(obj)>0)
        add_block('simulink3/Sinks/Out1',[mdl,'/out']);
        add_line(mdl,'DrakeSys/1','out/1');
      end
      
      if ~isempty(obj.state_constraints)
        obj.warning_manager.warnOnce('Drake:DrakeSystem:ConstraintsNotEnforced','system has constraints, but they aren''t enforced in the simulink model yet.');
      end
      
    end

    function [xstar,ustar,info] = findFixedPoint(obj,x0,u0)
      % attempts to find a fixed point (xstar,ustar) which also satisfies the constraints,
      % using (x0,u0) as the initial guess.
      %
      % @param x0 initial guess for the state
      % @param u0 initial guess for the input
      %
      % Note: consider manually constructing a FixedPointProgram
      % for a much richer interface where you can set solver
      % parameters and/or add additional objectives/constraints.

      prog = FixedPointProgram(obj);
      [xstar,ustar,info] = findFixedPoint(prog,x0,u0);
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
      if (num_xc<0), error('num_xc must be >= 0'); end
      obj.num_xc = num_xc;
      obj.num_x = obj.num_xd + obj.num_xc;
      if (isempty(obj.getStateFrame) || obj.num_x~=obj.getStateFrame.dim)
        obj=setStateFrame(obj,CoordinateFrame([class(obj),'State'],obj.num_x,'x'));
      end
    end
    function obj = setNumDiscStates(obj,num_xd)
      % Guards the num_states variable
      if (num_xd<0), error('num_xd must be >= 0'); end
      obj.num_xd = num_xd;
      obj.num_x = obj.num_xc + obj.num_xd;
      if (isempty(obj.getStateFrame) || obj.num_x~=obj.getStateFrame.dim)
        obj=setStateFrame(obj,CoordinateFrame([class(obj),'State'],obj.num_x,'x'));
      end
    end
    function obj = setNumInputs(obj,num_u)
      % Guards the num_u variable.
      %  Also pads umin and umax for any new inputs with [-inf,inf].

      if (num_u<0), error('num_u must be >=0 or DYNAMICALLY_SIZED'); end

       % cut umin and umax to the right size, and pad new inputs with
      % [-inf,inf]
      if (length(obj.umin)~=num_u)
        obj.umin = [obj.umin; -inf*ones(max(num_u-length(obj.umin),0),1)];
      end
      if (length(obj.umax)~=num_u)
        obj.umax = [obj.umax; inf*ones(max(num_u-length(obj.umax),0),1)];
      end

      obj.num_u = num_u;
      if (isempty(obj.getInputFrame) || obj.num_u~=obj.getInputFrame.dim)
        obj=setInputFrame(obj,CoordinateFrame([class(obj),'Input'],num_u,'u'));
      end
    end
    function obj = setInputLimits(obj,umin,umax)
      % Guards the input limits to make sure it stay consistent

      if (isscalar(umin)), umin=repmat(umin,obj.num_u,1); end
      if (isscalar(umax)), umax=repmat(umax,obj.num_u,1); end

      sizecheck(umin,[obj.num_u,1]);
      sizecheck(umax,[obj.num_u,1]);
      if (any(obj.umax<obj.umin)), error('umin must be less than umax'); end
      obj.umin = umin;
      obj.umax = umax;
    end
    function obj = setNumOutputs(obj,num_y)
      % Guards the number of outputs to make sure it's consistent
      if (num_y<0), error('num_y must be >=0'); end
      obj.num_y = num_y;
      if (isempty(obj.getOutputFrame) || obj.num_y~=obj.getOutputFrame.dim)
        obj=setOutputFrame(obj,CoordinateFrame([class(obj),'Output'],num_y,'y'));
      end
    end
    function n = getNumZeroCrossings(obj)
      % Returns the number of zero crossings
      n = obj.num_zcs;
    end
    function obj = setNumZeroCrossings(obj,num_zcs)
      % Guards the number of zero crossings to make sure it's valid.
      if (num_zcs<0), error('num_zcs must be >=0'); end
      obj.num_zcs = num_zcs;
    end
    function n = getNumStateConstraints(obj)
      % @retval the total number state *equality* constraints in the program
      n = obj.num_xcon_eq;
    end
    function n = getNumUnilateralConstraints(obj)
      n = obj.num_xcon_ineq;
    end
    
    function [obj,id] = addStateConstraint(obj,con,xind)
      % @param con is a constraint object which takes the state of this
      % system as input
      % @param xind (optional) subset of the state indices

      typecheck(con,'Constraint');
      if nargin<3, 
        xind = 1:obj.num_x; 
      else
        assert(all(xind>=1) && all(xind<=obj.num_x));
      end
      assert(con.xdim == length(xind),'DrakeSystem:InvalidStateConstraint','xdim mismatch');

      obj.state_constraints{end+1} = con;
      obj.state_constraint_xind{end+1} = xind;
      obj.num_xcon_eq = obj.num_xcon_eq + sum(con.lb == con.ub);
      obj.num_xcon_ineq = obj.num_xcon_ineq + sum(con.lb ~= con.ub);
      id = numel(obj.state_constraints);
    end

    function obj = updateStateConstraint(obj,id,con,xind)
      % @param id is the identifier returned from addStateConstraint
      % @param con is a constraint object
      % @param xind (optional) subset of the state indices
      
      rangecheck(id,0,numel(obj.state_constraints));
      typecheck(con,'Constraint');
      if (nargin<4) xind = obj.state_constraint_xind{id}; end
      assert(con.xdim == length(xind),'DrakeSystem:InvalidStateConstraint','xdim mismatch');

      obj.num_xcon_eq = obj.num_xcon_eq - sum(obj.state_constraints{id}.lb == obj.state_constraints{id}.ub);
      obj.num_xcon_ineq = obj.num_xcon_ineq - sum(obj.state_constraints{id}.lb ~= obj.state_constraints{id}.ub);
      obj.state_constraints{id} = con;
      obj.state_constraint_xind{id} = xind;
      obj.num_xcon_eq = obj.num_xcon_eq + sum(con.lb == con.ub);
      obj.num_xcon_ineq = obj.num_xcon_ineq + sum(con.lb ~= con.ub);
    end
    
    function displayStateConstraints(obj)
      for i=1:length(obj.state_constraints)
        disp(obj.state_constraints{i});
      end
    end
    
    function obj = removeAllStateConstraints(obj)
      obj.num_xcon_eq = 0;  
      obj.num_xcon_ineq = 0;      
      obj.state_constraints = {};
      obj.state_constraint_xind = {};
    end
  end

  methods (Sealed)
    function prog = addStateConstraintsToProgram(obj,prog,indices)
      % adds state constraints and unilateral constraints to the
      %   program on the specified indices.
      % @param prog a NonlinearProgram object
      % @indices index into N decision variables in the program upon which
      % we are adding the constraints, where N is the number of state dimensions of this system

      for i=1:numel(obj.state_constraints)
        prog = prog.addConstraint(obj.state_constraints{i},indices(obj.state_constraint_xind{i}));
      end
    end
    function prog = addInputConstraintsToProgram(obj,prog,indices)
      % add bounding box constraint
      % todo: consider whether it makes sense to a list of constraints
      % objects instead of just input limits.  for now, this is sealed just
      % to keep things clean.

      con = BoundingBoxConstraint(obj.umin,obj.umax);
      con = setName(con,cellfun(@(a) [a,'_limit'],obj.getInputFrame.getCoordinateNames(),'UniformOutput',false));

      prog = prog.addBoundingBoxConstraint(con,indices);
    end
    function varargout = stateConstraints(obj,x)
      % Provides the old interface of a single constraint function which
      % evaluates all of the *equality* constraints on the state
      % (which should be == 0)
      %
      % @retval the evaluated *equality* constraints, and potentially their
      % derivatives.

      % Note: if you're tempted to overload this, you should be adding using the
      % addStateConstraint method instead

      varargout = cell(1,nargout);
      for i=1:length(obj.state_constraints)
        if ~isempty(obj.state_constraints{i}.ceq_idx)
          % then evaluate this constraint to the requested derivative level
          v = cell(1,nargout);
          [v{:}] = obj.state_constraints{i}.eval(x);
          v{1} = v{1} - obj.state_constraints{i}.lb;  % center it around 0
          for j=1:nargout
            varargout{j} = vertcat(varargout{j},v{j}(obj.state_constraints{i}.ceq_idx,:));
          end
        end
      end
    end
  end

  % utility methods
  methods
    function [A,B,C,D,x0dot,y0] = linearize(obj,t0,x0,u0)
      % Uses the geval engine to linearize the model around the nominal
      % point, at least for the simple case.

      if (~isCT(obj) || getNumDiscStates(obj)>0)  % boot if it's not the simple case
        [A,B,C,D,x0dot,y0] = linearize@DynamicalSystem(obj,t0,x0,u0);
        return;
      end

      nX = getNumContStates(obj);
      nU = getNumInputs(obj);
      [~,df] = geval(@obj.dynamics,t0,x0,u0);
      A = df(:,1+(1:nX));
      B = df(:,nX+1+(1:nU));

      if (nargout>2)
        [~,dy] = geval(@obj.output,t0,x0,u0);
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

    function varargout = simulate(obj,varargin)
      varargout=cell(1,max(nargout,1));
      if checkDependency('simulink') && exist('DCSFunction','file')
        [varargout{:}] = simulate@DynamicalSystem(obj,varargin{:});
      else
        [varargout{:}] = simulateODE(obj,varargin{:});
      end
    end
    
    function [ytraj,xtraj] = simulateODE(obj,tspan,x0,options)
      % Simulates the system using the ODE45 suite of solvers
      % instead of the simulink solvers.
      %
      % @param tspan a 1x2 vector of the form [t0 tf]
      % @param x0 a vector of length(getNumStates) which contains the initial
      % state
      % @param options options structure
      %
      % No options implemented yet

      if (nargin<3), x0=getInitialState(obj); end

      if (obj.num_zcs>0), warning('Drake:DrakeSystem:UnsupportedZeroCrossings','system has zero-crossings, but i havne''t passed them to ode45 yet.  (should be trivial)'); end
      if (getNumStateConstraints(obj)>0), warning('Drake:DrakeSystem:UnsupportedConstraints','system has constraints, but they are not explicitly satisfied during simulation (yet - it should be an easy fix in the ode suite)'); end

      odeoptions = obj.simulink_params;
      odefun = @(t,x)obj.dynamics(t,x,zeros(obj.getNumInputs(),1));
      if (isfield(obj.simulink_params,'Solver'))
        sol = feval(obj.simulink_params.Solver,odefun,tspan,x0,odeoptions);
      else
        sol = ode45(odefun,tspan,x0,odeoptions);
      end
      xtraj = ODESolTrajectory(sol);
      ytraj = FunctionHandleTrajectory(@(t)obj.output(t,xtraj.eval(t),zeros(obj.getNumInputs(),1)),[obj.getNumOutputs,1],tspan);
      ytraj = setOutputFrame(ytraj,getOutputFrame(obj));
      if nargout>1, xtraj=setOutputFrame(xtraj,getStateFrame(obj)); end
    end

    function sys=feedback(sys1,sys2)
      % Constructs a feedback combination of sys1 and sys2.
      %
      % @param sys1 first DynamicalSystem (on the forward path)
      % @param sys2 second DynamicalSystem (on the backward path)
      %
      % The input to the feedback model is added to the output of sys2
      % before becoming the input for sys1.  The output of the feedback
      % model is the output of sys1.

      if isa(sys2,'DrakeSystem')
        try
          sys=FeedbackSystem(sys1,sys2);  % try to keep it a drakesystem
        catch ex
          if (strcmp(ex.identifier, 'Drake:DrakeSystem:UnsupportedSampleTime'))
            warning('Drake:DrakeSystem:UnsupportedSampleTime','Aborting feedback combination as a DrakeSystem due to incompatible sample times');
            sys = feedback@DynamicalSystem(sys1,sys2);
          elseif (strcmp(ex.identifier, 'Drake:FeedbackSystem:NoHybridSupport') || strcmp(ex.identifier,'Drake:FeedbackSystem:NoStochasticSupport'))
            sys = feedback@DynamicalSystem(sys1,sys2);
          else
            rethrow(ex);
          end
        end
      else
        sys=feedback@DynamicalSystem(sys1,sys2);
      end
    end

    function sys=cascade(sys1,sys2)
      % Constructs a cascade combination of sys1 and sys2.
      %
      % @param sys1 first DynamicalSystem
      % @param sys2 second DynamicalSystem
      %
      % The input to the cascade system is the input to sys1.
      % The output of sys1 is fed to the input of sys2.
      % The output of the cascade system is the output of sys2.

      if isa(sys2,'DrakeSystem')
        try
          sys=CascadeSystem(sys1,sys2);   % try to keep it a drakesystem
        catch ex
          if (strcmp(ex.identifier, 'Drake:DrakeSystem:UnsupportedSampleTime'))
            warning('Drake:DrakeSystem:UnsupportedSampleTime','Aborting cascade combination as a DrakeSystem due to incompatible sample times');
            sys = cascade@DynamicalSystem(sys1,sys2);
          elseif (strcmp(ex.identifier, 'Drake:CascadeSystem:NoHybridSupport') || strcmp(ex.identifier,'Drake:CascadeSystem:NoStochasticSupport'))
            sys = cascade@DynamicalSystem(sys1,sys2);
          else
            rethrow(ex);
          end
        end
      else
        sys=cascade@DynamicalSystem(sys1,sys2);
      end
    end

    function polysys = extractPolynomialSystem(obj)
      % Attempts to symbolically extract the extra structure of a
      % polynomial system from the Drake system
      % Will throw an error if the system is not truly polynomial.
      %
      % See also extractTrigPolySystem, taylorApprox

      t=msspoly('t',1);
      x=msspoly('x',sys.num_x);
      u=msspoly('u',sys.num_u);

      p_dynamics_rhs=[];
      p_dynamics_lhs=[];
      p_update = [];
      p_output = [];
      p_state_constraints = [];

      try
        if (obj.num_xc>0)
          p_dynamics_rhs = dynamics(obj,t,x,u);
        end
        if (obj.num_xd>0)
          p_update = update(obj,t,x,u);
        end
        p_output = output(obj,t,x,u);

        if (obj.num_xcon>0)
          % todo: extract polynomial constraints
          p_state_constraints = stateConstraints(obj,x);
        end
      catch ex
        error('DrakeSystem:ExtractPolynomialSystem:NotPolynomial','This system appears to not be polynomial');
      end
      polysys = SpotPolynomialSystem(getInputFrame(obj),getStateFrame(obj),getOutputFrame(obj),p_dynamics_rhs,p_dynamics_lhs,p_update,p_output,p_state_constraints);

      polysys = setSampleTime(polysys,obj.getSampleTime);
    end

    function sys = extractAffineSystem(obj)
      % Attempts to symbolically extract the extra structure of an
      % affine system from the Drake system
      % Will throw an error if the system is not truly affine.
      %
      % See also taylorApprox

      sys = extractAffineSystem(extractPolynomialSystem(obj));
    end

    function sys = extractLinearSystem(obj)
      % Attempts to symbolically extract the extra structure of a
      % linear system from the Drake system
      % Will throw an error if the system is not truly linear.
      %
      % See also linearize, taylorApprox

      sys = extractLinearSystem(extractAffineSystem(obj));
    end

  end

  methods % deprecated (due to refactoring)
    function polysys = makeTrigPolySystem(obj,options)
      % deprecated method (due to refactoring): please use extractTrigPolySystem instead
      warning('Drake:DeprecatedMethod','makeTrigPolySystem has been refactored and will go away.  Please use extractTrigPolySystem(obj,options) instead');
      polysys = extractTrigPolySystem(obj,options);
    end
  end

  % utility methods
  methods
    function systemGradTest(obj,t,x,u,options)
      % Compare numerical and analytical derivatives of dynamics,update,and
      % output

      if nargin<2, t=0; end
      if nargin<3, x=getInitialState(obj); end
      if nargin<4, u=getDefaultInput(obj); end
      if nargin<5, options=struct('tol',.01); end
      if ~isfield(options,'dynamics'), options.dynamics=true; end
      if ~isfield(options,'update'), options.update=true; end
      if ~isfield(options,'output'), options.output=true; end

      if (options.dynamics && getNumContStates(obj))
        gradTest(@obj.dynamics,t,x,u,options)
      end
      if (options.update && getNumDiscStates(obj))
        gradTest(@obj.update,t,x,u,options);
      end
      if (options.output && getNumOutputs(obj))
        gradTest(@obj.output,t,x,u,options);
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
    direct_feedthrough_flag=true;  % true/false: does the output depend on u?  set false if you can!
    ts=[];    % default sample times of the model
    num_xcon_eq = 0;  % number of state *equality* constraints
    num_xcon_ineq = 0; % number of state *inequality* constraints
    state_constraints={}; % a cell array of constraint objects which depend on the state vector x
    state_constraint_xind={};  % cell array of xindices (one for each state constraint)
  end
  properties (SetAccess=private, GetAccess=public)
    umin=[];   % constrains u>=umin (default umin=-inf)
    umax=[];    % constrains u<=uman (default umax=inf)
  end
end
