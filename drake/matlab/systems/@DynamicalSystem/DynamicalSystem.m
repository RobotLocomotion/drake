classdef DynamicalSystem
% An interface class for a state-space dynamical system 
%  with a single (vector) input u, a single (vector) output y, and a single (vector) state x composed of a combination of continuous time and discrete time variables
                                                   
  methods (Abstract = true)
    % Returns the scalar number of continuous state variables, aka xc
    n = getNumContStates(obj);  

    % Returns the scalar number of discrete state variables, aka xd
    n = getNumDiscStates(obj);

    % Returns the scalar length of the single input vector, aka u
    n = getNumInputs(obj);
    
    % Returns the scalar length of the single output vector, aka y
    n = getNumOutputs(obj);
    
    % Returns a simulink compatible sample time for the system.  
    % See http://www.mathworks.com/help/toolbox/simulink/ug/brrdmmw-5.html
    % for a full description
    ts = getSampleTime(obj);

    % Returns a SimulinkModelHandle to a simulink model which implements the system
    mdl = getModel(obj);

    % Returns a (possibly random) state vector which is a feasible initial
    % condition for the system.  
    % @retval x0 initial state vector, containing discrete and continuous
    % states
    x0 = getInitialState(obj);

    % Implements the differential equation governing the dynamics of the
    % continuous state variables.  
    % @param t time (scalar)
    % @param x state vector, containing discrete and continuous states
    % @param u input vector
    % @retval xcdot derivative vector of ONLY the continuous states
    xcdot = dynamics(obj,t,x,u);
    
    % Implements the difference equation governing the dynamics of the
    % discrete state variables
    % @param t time (scalar)
    % @param x state vector, containing discrete and continuous states
    % @param u input vector
    % @retval xdn new value (e.g. xd[n+1])for ONLY the discrete states in the state vector
    xdn = update(obj,t,x,u);
    
    % Implements the output function
    % @param t time (scalar)
    % @param x state vector, containing discrete and continuous states
    % @param u input vector
    y = output(obj,t,x,u);
  end
  
  % construction methods
  methods
    function sys = DynamicalSystem()
      sys.warning_manager = WarningManager();
    end
        
    function sys = inInputFrame(sys,frame)
      % Ensures that sys has the specified input frame, by
      % searching for and cascading a coordinate transformation to
      % the existing input if necessary.
      if (getInputFrame(sys)~=frame)
        tf = findTransform(frame,getInputFrame(sys),struct('throw_error_if_fail',true));
        sys = cascade(tf,sys);
      end
    end
    
    function sys = inStateFrame(sys,frame)
      % Ensures that sys has the specified state frame
      error('not implemented yet');  % but shouldn't be too hard using functionhandle systems.
    end
    
    function sys = inOutputFrame(sys,frame)
      % Ensures that sys has the specified output frame, by
      % searching for and cascading a coordinate transformation to
      % the existing output if necessary.
      if (frame ~= sys.getOutputFrame)
        tf = findTransform(getOutputFrame(sys),frame,struct('throw_error_if_fail',true));
        sys = cascade(sys,tf);
      end
    end
    
    function newsys = cascade(sys1,sys2)
      % Creates a new system with the output of system 1 connected to the
      % input of system 2. The input coordinate frame of system 2 must match
      % the output coordinate frame of system 1, or system 1's output frame
      % must know how to transform between the frames.
      %
      % @param sys1 the first DynamicalSystem  
      % @param sys2 the second DynamicalSystem
      %
      % @retval newsys the new DynamicalSystem

      sys2 = sys2.inInputFrame(sys1.getOutputFrame);
      
      mdl = ['Cascade_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
      new_system(mdl,'Model');
      set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
      mdl = SimulinkModelHandle(mdl);
      
      mdl.addSubsystem('system1',sys1.getModel());
      mdl.addSubsystem('system2',sys2.getModel());
      add_line(mdl,'system1/1','system2/1');

      if (getNumInputs(sys1)>0)
        add_block('simulink3/Sources/In1',[mdl,'/in']);
        add_line(mdl,'in/1','system1/1');
      end
      
      if (getNumOutputs(sys2)>0)
        add_block('simulink3/Sinks/Out1',[mdl,'/out']);
        add_line(mdl,'system2/1','out/1');
      end
      
      newsys = SimulinkModel(mdl,sys1.getNumInputs);
      newsys = setInputFrame(newsys,getInputFrame(sys1));
      newsys = setStateFrame(newsys,MultiCoordinateFrame.constructFrame( ...
        { getStateFrame(sys1),getStateFrame(sys2) }, ...
        [ ones(getNumContStates(sys1),1); ...
          2*ones(getNumContStates(sys2),1); ...
          ones(getNumDiscStates(sys1),1); ...
          2*ones(getNumDiscStates(sys2),1) ], ...
          true) );
      newsys = setOutputFrame(newsys,getOutputFrame(sys2));
      newsys.time_invariant_flag = sys1.time_invariant_flag && sys2.time_invariant_flag;
      newsys.simulink_params = catstruct(sys1.simulink_params,sys2.simulink_params);
    end
    
    function newsys = feedback(sys1,sys2)
      % Creates a new systems with sys1 and sys2 in a feedback interconnect 
      % (with sys1 on the forward path, and sys2 on the return path). 
      % the input of the new system gets added into the input of sys1
      % the output of sys1 will be the output of the new system.
      
      sys2 = sys2.inInputFrame(sys1.getOutputFrame);
      sys2 = sys2.inOutputFrame(sys1.getInputFrame);
      
      mdl = ['Feedback_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
      new_system(mdl,'Model');
      set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
      mdl = SimulinkModelHandle(mdl);
      
      load_system('simulink3');
      add_block('simulink3/Sources/In1',[mdl,'/in']);
      add_block('simulink3/Math/Sum',[mdl,'/sum']);
      add_line(mdl,'in/1','sum/1');

      mdl.addSubsystem('system1',sys1.getModel());
      mdl.addSubsystem('system2',sys2.getModel());

      add_line(mdl,'sum/1','system1/1');
      add_line(mdl,'system1/1','system2/1');
      add_line(mdl,'system2/1','sum/2');

      add_block('simulink3/Sinks/Out1',[mdl,'/out']);
      add_line(mdl,'system1/1','out/1');
      
      newsys = SimulinkModel(mdl,sys1.getNumInputs);
      newsys = setInputFrame(newsys,getInputFrame(sys1));
      newsys = setStateFrame(newsys,MultiCoordinateFrame.constructFrame( ...
        { getStateFrame(sys1),getStateFrame(sys2) }, ...
        [ ones(getNumContStates(sys1),1); ...
          2*ones(getNumContStates(sys2),1); ...
          ones(getNumDiscStates(sys1),1); ...
          2*ones(getNumDiscStates(sys2),1) ], ...
          true) );
      newsys = setOutputFrame(newsys,getOutputFrame(sys1));
      newsys.time_invariant_flag = sys1.time_invariant_flag && sys2.time_invariant_flag;
      newsys.simulink_params = catstruct(sys1.simulink_params,sys2.simulink_params);
    end
    
    function newsys = parallel(sys1,sys2)
      % Creates a new system that takes the inputs to both sys1 and sys2
      % as a single input (which is "demux"ed and passed independently to
      % the two systems), and outputs the "mux"ed output of the two
      % systems.
      
      mdl = ['Parallel_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
      new_system(mdl,'Model');
      set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
      mdl = SimulinkModelHandle(mdl);
      
      load_system('simulink3');
      
      mdl.addSubsystem('system1',sys1.getModel());
      mdl.addSubsystem('system2',sys2.getModel());
      
      if (getNumInputs(sys1)>0 || getNumInputs(sys2)>0)
        inframe = MultiCoordinateFrame.constructFrame({sys1.getInputFrame,sys2.getInputFrame});
        add_block('simulink3/Sources/In1',[mdl,'/in']);
        in=setupMultiOutput(inframe,mdl,'in');
        sys1in=setupMultiInput(getInputFrame(sys1),mdl,'system1');
        j=1;
        for i=1:getNumFrames(getInputFrame(sys1))
          add_line(mdl,[in,'/',num2str(j)],[sys1in,'/',num2str(i)]);
          j=j+1;
        end          
        sys2in=setupMultiInput(getInputFrame(sys2),mdl,'system2');
        for i=1:getNumFrames(getInputFrame(sys2))
          add_line(mdl,[in,'/',num2str(j)],[sys2in,'/',num2str(i)]);
          j=j+1;
        end          
      end
        
      if (getNumOutputs(sys1)>0 || getNumOutputs(sys2)>0)
        outframe = MultiCoordinateFrame.constructFrame({sys1.getOutputFrame,sys2.getOutputFrame});
        add_block('simulink3/Sinks/Out1',[mdl,'/out']);
        out=setupMultiInput(outframe,mdl,'out');
        sys1out=setupMultiOutput(getOutputFrame(sys1),mdl,'system1');
        j=1;
        for i=1:getNumFrames(getOutputFrame(sys1))
          add_line(mdl,[sys1out,'/',num2str(i)],[out,'/',num2str(j)]);
          j=j+1;
        end          
        sys2out=setupMultiOutput(getOutputFrame(sys2),mdl,'system2');
        for i=1:getNumFrames(getOutputFrame(sys2))
          add_line(mdl,[sys2out,'/',num2str(i)],[out,'/',num2str(j)]);
          j=j+1;
        end          
      end      

      newsys = SimulinkModel(mdl,getNumInputs(sys1)+getNumInputs(sys2));
      if (getNumInputs(newsys)>0)
        newsys = setInputFrame(newsys,inframe);
      end
      newsys = setStateFrame(newsys,MultiCoordinateFrame.constructFrame( ...
        { getStateFrame(sys1),getStateFrame(sys2) }, ...
        [ ones(getNumContStates(sys1),1); ...
          2*ones(getNumContStates(sys2),1); ...
          ones(getNumDiscStates(sys1),1); ...
          2*ones(getNumDiscStates(sys2),1) ], ...
          true) );
      if (getNumOutputs(newsys)>0)
        newsys = setOutputFrame(newsys,outframe);
      end
      newsys.time_invariant_flag = sys1.time_invariant_flag && sys2.time_invariant_flag;
      newsys.simulink_params = catstruct(sys1.simulink_params,sys2.simulink_params);
    end
    
    function newsys = sampledData(sys,tsin,tsout)
      % Creates a new system which is a sampled data (e.g. discretized in time) version of the original system.  
      % This is accomplished by adding rate transition blocks to the inputs and outputs.
      % @param tsin the sample times of the input
      % @param tsout the sample times of the output
      % @retval newsys the new, sampled-data system
      % See getSampleTime for more details about sample times.
      if (nargin<3) tsout = tsin; end
      typecheck(tsin,'double'); 
      sizecheck(tsin,[1 1]);
      typecheck(tsout,'double');
      sizecheck(tsout,[1 1]);
      
      mdl = ['SampledData_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
      new_system(mdl,'Model');
      set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
      mdl = SimulinkModelHandle(mdl);
      
      load_system('simulink3');
      mdl.addSubsystem('system',sys.getModel());

      if (getNumInputs(sys)>0)
        add_block('simulink3/Sources/In1',[mdl,'/in']);
        add_block('simulink/Signal Attributes/Rate Transition',[mdl,'/zoh1'],'OutPortSampleTime',num2str(tsin));
        add_line(mdl,'in/1','zoh1/1');
        add_line(mdl,'zoh1/1','system/1');
      end
      
      if (getNumOutputs(sys)>0)
        add_block('simulink3/Sinks/Out1',[mdl,'/out']);
        add_block('simulink/Signal Attributes/Rate Transition',[mdl,'/zoh2'],'OutPortSampleTime',num2str(tsout));
        add_line(mdl,'system/1','zoh2/1');
        add_line(mdl,'zoh2/1','out/1');
      end
      
      newsys = SimulinkModel(mdl,sys.getNumInputs);
      newsys = setInputFrame(newsys,getInputFrame(sys));
      newsys = setStateFrame(newsys,getStateFrame(sys));
      newsys = setOutputFrame(newsys,getOutputFrame(sys));
      newsys.time_invariant_flag = sys.time_invariant_flag;
      newsys.simulink_params = sys.simulink_params;  
    end
    
    function [x,success,prog] = resolveConstraints(obj,x0,v,constraints)
      % attempts to find a x which satisfies the constraints,
      % using x0 as the initial guess.
      %
      % @param x0 initial guess for state satisfying constraints
      % @param v (optional) a visualizer that should be called while the
      % solver is doing it's thing
      % @param constraints (optional) additional constraints to pass to the
      % solver

      if nargin<3, v=[]; end
      if getNumStates(obj)<1, 
        x=Point(obj.getStateFrame); success=true; prog=[]; return;
      end
      if isa(x0,'Point')
        x0 = double(x0.inFrame(obj.getStateFrame));
      end
            
      nx = getNumStates(obj);
      prog = NonlinearProgram(nx,getCoordinateNames(getStateFrame(obj)));

      prog = addStateConstraintsToProgram(obj,prog,1:nx);
      
      if nargin>3,
        prog = addConstraint(prog,constraints);
      end
      
      if ~isempty(v)
        prog = addDisplayFunction(prog,@(x)v.drawWrapper(0,x));
      end
      
      [x,~,exitflag,infeasible_constraint_name] = solve(prog,x0);
      success=(exitflag<10);
      if ~isempty(infeasible_constraint_name)
        infeasible_constraint_name
      end
      if (nargout<2 && ~success)
        error('Drake:DynamicalSystem:ResolveConstraintsFailed','failed to resolve constraints');
      end
      x = Point(obj.getStateFrame,x);
    end
    
    function prog = addStateConstraintsToProgram(obj,prog,indices)
      % adds state constraints and unilateral constriants to the 
      %   program on the specified indices.  derived classes can overload 
      %   this method to add additional constraints.
      % 
      % @param prog a NonlinearProgram class
      % @param indices the indices of the state variables in the program
      %        @default 1:nX

      typecheck(prog,'NonlinearProgram');
      nx = getNumStates(obj);
      if nargin<3, indices=1:nx; end
      
      % add state constraints
      nc = getNumStateConstraints(obj);
      if nc>0
        con = FunctionHandleConstraint(zeros(nc,1),zeros(nc,1),nx,@obj.stateConstraints);
        con.grad_method = 'user_then_taylorvar';
        prog = addConstraint(prog,con,indices);
    end
    
      % add unilateral constraints
      nc = getNumUnilateralConstraints(obj);
      if nc>0
        con = FunctionHandleConstraint(zeros(nc,1),inf(nc,1),nx,@obj.unilateralConstraints);
        con.grad_method = 'user_then_taylorvar';
        prog = addConstraint(prog,con,indices);
      end
    end
    
  end
  
  % utility methods
  methods
    function n = getNumStates(obj)
      % Returns the total number of states (discrete + continuous) in the system
      n = getNumDiscStates(obj) + getNumContStates(obj);
    end
    function ts = getInputSampleTimes(obj)
      % Returns the sample time of the input
      % See getSampleTime for more details about sample times.
      mdl = getModel(obj);
      inport = find_system(mdl,'SearchDepth',1,'BlockType','Inport');
      ts = [];
      for i=1:length(inport)
        ts = [ts,getSimulinkSampleTimes(inport{i})];
      end
    end
    function ts = getOutputSampleTimes(obj)
      % Returns the sample time of the output
      % See getSampleTime for more details about sample times.
      mdl = getModel(obj);
      outport = find_system(mdl,'SearchDepth',1,'BlockType','Outport');
      ts = [];
      for i=1:length(outport)
        ts = [ts,getSimulinkSampleTimes(outport{i})];
      end
    end
    
    function tf = isDT(obj)
      % Returns true if the system has no states, or has states and only one sample time [a b], with a>0
      ts = getSampleTime(obj);
      tf = getNumStates(obj)==0 || (size(ts,2)==1 && ts(1)>0); % only one sample time and not continuous
    end
    function tf = isCT(obj)
      % Returns true if the system has only one sample time [a b], with a==0
      ts = getSampleTime(obj);
      tf = (size(ts,2)==1 && ts(1)==0); % only one sample time and continuous
    end
    function tf = isInheritedTime(obj)
      % Returns true if the system has only one sample time [a b], with b==-1
      ts = getSampleTime(obj);
      tf = (size(ts,2)==1 && ts(1)==-1); % only one sample time and continuous
    end
    
    function tf = isTI(obj)  
      % Returns true if the system is time-invariant
      tf = obj.time_invariant_flag;
    end
    function obj = setTIFlag(obj,bval)
      % Sets the time invariant flag.
      % Set TI=true if you know the system is time-invariant.  It simplifies many things.
      obj.time_invariant_flag = bval;
    end

    function fr=getInputFrame(obj)
      % Access the CoordinateFrame object that defines the input to this
      % system
      fr=obj.input_frame;
    end
    function obj=setInputFrame(obj,fr)
      % Set the CoordinateFrame object that defines the input of this
      % system
      % @param fr CoordinateFrame object which must have the correct
      % dimension
      typecheck(fr,'CoordinateFrame');
      if (fr.dim ~= obj.getNumInputs()) error('frame dimension does not match number of inputs'); end
      obj.input_frame=fr;
    end
    function fr=getStateFrame(obj)
      % Access the CoordinateFrame object that defines the state of this
      % system
      fr=obj.state_frame;
    end
    function obj=setStateFrame(obj,fr)
      % Set the CoordinateFrame object that defines the state of this
      % system
      % @param fr CoordinateFrame object which must have the correct
      % dimension
      typecheck(fr,'CoordinateFrame');
      if (fr.dim ~= obj.getNumStates()) error('frame dimension does not match number of states'); end
      obj.state_frame=fr;
    end
    function fr=getOutputFrame(obj)
      % Access the CoordinateFrame object that defines the output to this
      % system
      fr=obj.output_frame;
    end
    function obj=setOutputFrame(obj,fr)
      % Set the CoordinateFrame object that defines the output of this
      % system
      % @param fr CoordinateFrame object which must have the correct
      % dimension
      typecheck(fr,'CoordinateFrame');
      if (fr.dim ~= obj.getNumOutputs()) error('frame dimension does not match number of outputs'); end
      obj.output_frame=fr;
    end
    
    function n = getNumStateConstraints(obj);
      % Returns the scalar number of state constraints (of the form phi(x)=0)
      n = 0;  % default behavior is n=0
    end
    
    function con = stateConstraints(obj,x)
      % defines state equality constraints in the form phi(x)=0
      error('Drake:DynamicalSystem:AbstractMethod','systems with state constraints must implement the constraints method');
    end
    
    function con = unilateralConstraints(obj,x)
      % defines state unilateral constraints in the form phi(x)>=0
      error('Drake:DynamicalSystem:AbstractMethod','systems with unilateral constraints must implement the constraints method');
    end
    
    function n = getNumUnilateralConstraints(obj)
      % Returns the scalar number of state constraints (of the form phi(x)>=0)
      n = 0;  % default behavior is n=0
    end
    
    function obj = setSimulinkParam(obj,varargin)
      % Sets parameters of the simulink model
      % Syntax
      % setSimulinkParam(obj,param_name,param_value[,param_name,param_value,...])
      % Se
      % @param param_name the string parameter name
      % @param param_value the *string* representing the parameter value
      % @retval obj the DynamicalSystem object with the updated parameters
      % 
      % Try 'doc set_param' in matlab to see the simulink model and block
      % parameters.
      % Note that this supports many of the same parameter values that you
      % would send to odeset.
      if (mod(length(varargin),2)) error('invalid input'); end        
      i=1;
      while(i<length(varargin))
        if (length(varargin{i+1})==0) 
          obj.simulink_params = rmfield(obj.simulink_params,varargin{i});
        else
          if (~ischar(varargin{i+1})) error('simulink params should all be strings (unfortunately)'); end
          obj.simulink_params = setfield(obj.simulink_params,varargin{i},varargin{i+1});
          set_param(obj.getModel(),varargin{i},varargin{i+1});
        end
        i=i+2;
      end
    end
    
    function [A,B,C,D,xdot0,y0] = linearize(obj,t0,x0,u0)
      % Linearize the system about an operating point (continuous time)
      % @param t0 nominal scalar time
      % @param x0 nomimal vector of discrete+continuous state
      % @param u0 nominal vector input
      % @retval A,B,C,D,xdot0,y0   linear dynamics such that xdot=Ax+Bu+xdot0, y=C*x+Du+y0

      % note: also worth looking into using linearize instead of linmod: http://www.mathworks.com/help/toolbox/slcontrol/ug/linearize.html
      mdl = getModel(obj);
      if (~strcmp(get_param(mdl,'SimulationStatus'),'stopped'))
        feval(mdl,[],[],[],'term');
      end
      [A,B,C,D] = linmod(mdl.name,x0,u0,[1e-5,t0]);
      
      if (nargout>4) 
        xdot0 = dynamics(obj,t0,x0,u0);
        if (length(xdot0)~=size(A,1)) 
          % linmod did something clever, like adding states to the model.  
          error('have to handle this case more carefully'); 
        end
        if (nargout>5)
          y0 = output(obj,t0,x0,u0);
        end
      end
    end
    function [A,B,C,D,xn0,y0] = dlinearize(obj,ts,t0,x0,u0)
      % Linearize the system about an operating point (discrete time)
      %
      % @param ts sample time for linearization
      % @param t0 nominal scalar time
      % @param x0 nomimal vector of discrete+continuous state
      % @param u0 nominal vector input
      %
      % @retval A,B,C,D,xn0,y0   linear dynamics such that x[n+1]=Ax[n]+Bu[n]+xn0, y[n]=C*x[n]+Du[n]+y0

      mdl = getModel(obj);
      if (~strcmp(get_param(mdl,'SimulationStatus'),'stopped'))
        feval(mdl,[],[],[],'term');
      end
      [A,B,C,D] = dlinmod(mdl.name,ts,x0,u0,[1e-5,t0]);
      if (nargout>4) 
        xn0 = update(obj,t0,x0,u0);
        if (length(xn0)~=size(A,1)) 
          % linmod did something clever, like adding states to the model.  
          error('have to handle this case more carefully'); 
        end
        if (nargout>5)
          y0 = output(obj,t0,x0,u0);
        end
      end
    end
    
  end

  methods % for handling system parameters

    function obj = setParams(obj,p)
      % This default setParams method attempts to set class properties of the
      % system according to the coordinate names in the parameter frame.

      if isa(p,'Point')
        p = double(inFrame(p,obj.param_frame));
      else
        sizecheck(p,obj.param_frame.dim);
      end
      c = obj.param_frame.getCoordinateNames();
      for i=1:length(c)
        obj.(c{i}) = p(i);
      end
    end
    
    function p = getParams(obj)
      % This default getParams method attempts to get class properties of the
      % system according to the coordinate names in the parameter frame.

      c = obj.param_frame.getCoordinateNames();
      p=zeros(obj.param_frame.dim,1);
      for i=1:length(c)
        p(i) = obj.(c{i});
      end
      p = Point(obj.param_frame,p);
    end
    
    function n = getNumParams(obj)
      n = obj.param_frame.dim;
    end
    
    function obj = setParamFrame(obj,fr)
      % Set the CoordinateFrame object which describes any system
      % parameters
      typecheck(fr,'CoordinateFrame');
      obj.param_frame = fr;
      if numel(obj.pmin)>fr.dim
        obj.pmin=-inf(fr.dim,1);
        obj.pmax=inf(fr.dim,1);
      elseif numel(obj.pmin)<fr.dim
        obj.pmin=vertcat(obj.pmin,-inf(fr.dim-numel(obj.pmin),1));
        obj.pmax=vertcat(obj.pmax,inf(fr.dim-numel(obj.pmax),1));
      end
    end
    function fr = getParamFrame(obj)
      % Returns the CoordinateFrame object which describes any system
      % parameters
      fr = obj.param_frame;
    end
    
    function obj = setParamLimits(obj,pmin,pmax)
      % Set lower and upper bounds on the system parameters
      d = prod(obj.param_frame.dim);
      sizecheck(pmin,[d 1]);
      obj.pmin = pmin;
      if nargin>2,
        sizecheck(pmax,[d 1]);
        obj.pmax = pmax; 
      end
    end
    
    function [pmin,pmax] = getParamLimits(obj)
      % Returns the current lower and upper bounds on the system parameters
      pmin = obj.pmin;
      pmax = obj.pmax;
    end
  end
  
  
  methods (Access=protected)
    function [sys1ind,sys2ind] = stateIndicesForCombination(sys1,sys2)
      % Helper method to figure out the indices of the discrete and
      % continuous states after sys1 and sys2 have been combined into a
      % single system.
      ind=0;
      n=sys1.getNumDiscStates();
      sys1ind = ind+(1:n)';
      ind=ind+n;
      n=sys2.getNumDiscStates();
      sys2ind=  ind+(1:n)'; ind=ind+n;

      n=sys1.getNumContStates();
      sys1ind = [sys1ind; ind+(1:n)'];  ind=ind+n;
      n=sys2.getNumContStates();
      sys2ind = [sys2ind; ind+(1:n)'];  
    end
   
    function xs = stateVectorToStructure(obj,xv,mdl)
      % Converts the vector state xv to the structure xs for simulink state
      % @param xv the state (in vector form)
      % @param mdl optional - pass in a model (default: getModel(obj))
      
      if (getNumStates(obj)<1) xs=[]; return; end
      if (nargin<3) mdl = getModel(obj); end
      
      % note: obj is not being returned from this method, so this probably
      % happens every time.
      if (isempty(obj.structured_x))
        obj.structured_x = getSimulinkStateStructure(mdl.name);
      end
      xs = obj.structured_x;
      if (length(xs.signals)>1)
        l = {xs.signals(:).label};
        ind = [find(strcmp(l,'DSTATE')), find(strcmp(l,'CSTATE'))];
        c = 1;
        for i=ind
          d = xs.signals(i).dimensions;
          xs.signals(i).values = xv(c+[1:d]-1);
          c = c+d;
        end
      else
        xs.signals.values = xv;
      end
    end
    
    function xv = stateStructureToVector(obj,xs)
      % Converts the simulink state structure representation back to the vector state
      % @param xs the state (in simulink structure format)
      if (length(xs.signals)>1)
        l = {xs.signals(:).label};
        ind = [find(strcmp(l,'DSTATE')), find(strcmp(l,'CSTATE'))];
        c = 1;
        for i=ind
          d = xs.signals(i).dimensions;
          xv(c+[1:d]-1) = xs.signals(i).values;
          c = c+d;
        end
      else
        xv = xs.signals.values;
      end
      xv = xv';
    end    
    
  end

  methods 
    function phat = parameterEstimation(obj,data,options)
      % Estimate parameter values from input-output data
      %
      % @param data an instance of iddata (see 'help iddata' for more info)
      
      error('todo: implement a generic nonlinear optimization approach here');
    end
  end
    
  properties (SetAccess=private,GetAccess=private)
    time_invariant_flag = false;  % set to true if you know the system is time invariant
    structured_x;                 % simulink state structure (cached for efficiency)

    input_frame;  % CoordinateFrame for the system input
    state_frame;  % CoordinateFrame for the system state
    output_frame; % CoordinateFrame for the system output
    param_frame;  % CoordinateFrame for the system parameters
    pmin;         % minimum values for system parameters
    pmax;         % maximum values for system parameters
  end

  properties (SetAccess=private,GetAccess=protected)
    simulink_params=struct();     % simulink model parameters
    initial_state = [];  % getInitialState returns this if non-empty instead of a random state
  end
  
  properties (Access=public)
    warning_manager;
  end    
end
