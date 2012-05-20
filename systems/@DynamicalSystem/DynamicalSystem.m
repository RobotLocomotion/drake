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

    % Returns a handle (the string) of a simulink model which implements the system
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
    function [sys1,sys2] = matchCoordinateFramesForCombination(sys1,sys2,attach_transform_to_sys1)
      % A utility method which checks if the output of sys1 is in the same
      % frame as the input to sys2, or if there is a known transform for
      % the conversion. If the two cannot be combined, then it throws an error.
      %
      % @param sys1 the first DynamicalSystem
      % @param sys2 the second DynamicalSystem
      % @param attach_transform_to_sys1 if a CoordinateTranform needs to be
      % applied, then it can be applied to the input of sys1 (true) or the output
      % of sys2.  @default false

      if (nargin<3) attach_transform_to_sys1 = false; end
      
      if (getOutputFrame(sys1)~=getInputFrame(sys2))
        tf = findTransform(getOutputFrame(sys1),getInputFrame(sys2));
        if (isempty(tf))
          error(['Input frame ,' sys2.getInputFrame().name, ' does not match output frame ', sys1.getOutputFrame().name, ' and I cannot find a CoordinateTransform to make the connection']); 
        end
        
        if (attach_transform_to_sys1)
          sys1=cascade(sys1,tf);
        else
          sys2=cascade(tf,sys2);
        end
      end
      
    end
    
    function [sys1ind,sys2ind] = stateIndicesForCombination(sys1,sys2)
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

      [sys1,sys2] = matchCoordinateFramesForCombination(sys1,sys2);
      
      mdl = ['Cascade_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
      new_system(mdl,'Model');
      set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
      
      load_system('simulink3');
      add_block('simulink3/Subsystems/Subsystem',[mdl,'/system1']);
      Simulink.SubSystem.deleteContents([mdl,'/system1']);
      Simulink.BlockDiagram.copyContentsToSubSystem(sys1.getModel(),[mdl,'/system1']);
      add_block('simulink3/Subsystems/Subsystem',[mdl,'/system2']);
      Simulink.SubSystem.deleteContents([mdl,'/system2']);
      Simulink.BlockDiagram.copyContentsToSubSystem(sys2.getModel(),[mdl,'/system2']);
      add_line(mdl,'system1/1','system2/1');

      if (getNumInputs(sys1)>0)
        add_block('simulink3/Sources/In1',[mdl,'/in']);
        add_line(mdl,'in/1','system1/1');
      end
      
      if (getNumOutputs(sys2)>0)
        add_block('simulink3/Sinks/Out1',[mdl,'/out']);
        add_line(mdl,'system2/1','out/1');
      end
      
      newsys = SimulinkModel(mdl);
      newsys = setInputFrame(newsys,getInputFrame(sys1));
      newsys = setOutputFrame(newsys,getOutputFrame(sys2));
      newsys.time_invariant_flag = sys1.time_invariant_flag && sys2.time_invariant_flag;
      newsys.simulink_params = catstruct(sys1.simulink_params,sys2.simulink_params);
    end
    function newsys = feedback(sys1,sys2)
      % Creates a new systems with sys1 and sys2 in a feedback interconnect 
      % (with sys1 on the forward path, and sys2 on the return path). 
      % the input of the new system gets added into the input of sys1
      % the output of sys1 will be the output of the new system.
      
      [sys1,sys2] = matchCoordinateFramesForCombination(sys1,sys2,false);
      [sys2,sys1] = matchCoordinateFramesForCombination(sys2,sys1,true);
      
      mdl = ['Feedback_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
      new_system(mdl,'Model');
      set_param(mdl,'SolverPrmCheckMsg','none');  % disables warning for automatic selection of default timestep
      
      load_system('simulink3');
      add_block('simulink3/Sources/In1',[mdl,'/in']);
      add_block('simulink3/Math/Sum',[mdl,'/sum']);
      add_line(mdl,'in/1','sum/1');

      add_block('simulink3/Subsystems/Subsystem',[mdl,'/system1']);
      Simulink.SubSystem.deleteContents([mdl,'/system1']);
      Simulink.BlockDiagram.copyContentsToSubSystem(sys1.getModel(),[mdl,'/system1']);
      add_block('simulink3/Subsystems/Subsystem',[mdl,'/system2']);
      Simulink.SubSystem.deleteContents([mdl,'/system2']);
      Simulink.BlockDiagram.copyContentsToSubSystem(sys2.getModel(),[mdl,'/system2']);

      add_line(mdl,'sum/1','system1/1');
      add_line(mdl,'system1/1','system2/1');
      add_line(mdl,'system2/1','sum/2');

      add_block('simulink3/Sinks/Out1',[mdl,'/out']);
      add_line(mdl,'system1/1','out/1');
      
      newsys = SimulinkModel(mdl);
      newsys = setInputFrame(newsys,getInputFrame(sys1));
      newsys = setOutputFrame(newsys,getOutputFrame(sys1));
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
      
      load_system('simulink3');
      add_block('simulink3/Subsystems/Subsystem',[mdl,'/system']);
      Simulink.SubSystem.deleteContents([mdl,'/system']);
      Simulink.BlockDiagram.copyContentsToSubSystem(sys.getModel(),[mdl,'/system']);

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
      
      newsys = SimulinkModel(mdl);
      newsys.time_invariant_flag = sys.time_invariant_flag;
      newsys.simulink_params = sys.simulink_params;  
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
        ts = [ts,Simulink.Block.getSampleTimes(inport{i}).Value'];
      end
    end
    function ts = getOutputSampleTimes(obj)
      % Returns the sample time of the output
      % See getSampleTime for more details about sample times.
      mdl = getModel(obj);
      outport = find_system(mdl,'SearchDepth',1,'BlockType','Outport');
      ts = [];
      for i=1:length(outport)
        ts = [ts,Simulink.Block.getSampleTimes(outport{i}).Value'];
      end
    end
    
    function tf = isDT(obj)
      % Returns true if the system has only one sample time [a b], with b>0
      ts = getSampleTime(obj);
      tf = (size(ts,2)==1 && ts(1)>0); % only one sample time and not continuous
    end
    function tf = isCT(obj)
      % Returns true if the system has only one sample time [a b], with b==0
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

    function f=getInputFrame(obj)
      f=obj.input_frame;
    end
    function obj=setInputFrame(obj,f)
      typecheck(f,'CoordinateFrame');
      if (f.dim ~= obj.getNumInputs()) error('frame dimension does not match number of inputs'); end
      obj.input_frame=f;
    end
    function f=getStateFrame(obj)
      f=obj.state_frame;
    end
    function obj=setStateFrame(obj,f)
      typecheck(f,'CoordinateFrame');
      if (f.dim ~= obj.getNumStates()) error('frame dimension does not match number of states'); end
      obj.state_frame=f;
    end
    function f=getOutputFrame(obj)
      f=obj.output_frame;
    end
    function obj=setOutputFrame(obj,f)
      typecheck(f,'CoordinateFrame');
      if (f.dim ~= obj.getNumOutputs()) error('frame dimension does not match number of outputs'); end
      obj.output_frame=f;
    end
    
    
    function xs = stateVectorToStructure(obj,xv,mdl)
      % Converts the vector state xv to the structure xs for simulink state
      % @param xv the state (in vector form)
      % @param mdl optional - pass in a model (default: getModel(obj))
      
      if (obj.getNumStates()<1) xs=[]; return; end
      if (nargin<3) mdl = getModel(obj); end
      
      if (isempty(obj.structured_x))
        obj.structured_x = Simulink.BlockDiagram.getInitialState(mdl);
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
      [A,B,C,D] = linmod(mdl,x0,u0,[1e-5,t0]);
      
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
      [A,B,C,D] = dlinmod(mdl,ts,x0,u0,[1e-5,t0]);
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
    
    function runLCMPlant(obj,lcmCoder,x0,options)
      % Runs the system as an LCM client which subscribes to u and publishes xhat
      % @param lcmCoder an LCMCoder object, which defines all the messages
      %  @param x0 initial conditions.  Use [] for the default initial
      %  conditions.
      % @param options see the options for the runLCM() method
      if (nargin<3) x0=[]; end
      if (nargin<4) options = struct(); end
      runLCM(obj,lcmCoder,'u','xhat',x0,options);
    end
    function runLCMControl(obj,lcmCoder,x0,options)
      % Runs the system as an LCM client which subscribes to xhat and publishes u
      % @param lcmCoder an LCMCoder object, which defines all the messages
      %  @param x0 initial conditions.  Use [] for the default initial
      %  conditions.
      % @param options see the options for the runLCM() method
      if (nargin<3) x0=[]; end
      if (nargin<4) options = struct(); end
      runLCM(obj,lcmCoder,'xhat','u',x0,options);
    end
    function runLCMEstimator(obj,lcmCoder,x0,options)
      % Runs the system as an LCM client which subscribes to y and publishes xhat
      % @param lcmCoder an LCMCoder object, which defines all the messages
      %  @param x0 initial conditions.  Use [] for the default initial
      %  conditions.
      % @param options see the options for the runLCM() method
      if (nargin<3) x0=[]; end
      if (nargin<4) options = struct(); end
      runLCM(obj,lcmCoder,'y','xhat',x0,options);
    end
    function runLCMVisualizer(obj,lcmCoder,x0,options)
      % Runs the system as an LCM client which subscribes to xhat and publishes nothing
      % @param lcmCoder an LCMCoder object, which defines all the messages
      %  @param x0 initial conditions.  Use [] for the default initial
      %  conditions.
      % @param options see the options for the runLCM() method
      if (nargin<3) x0=[]; end
      if (nargin<4) options = struct(); end
      runLCM(obj,lcmCoder,'xhat',[],x0,options);
    end

  end
  
  properties (SetAccess=private,GetAccess=private)
    time_invariant_flag = false;  % set to true if you know the system is time invariant
    simulink_params=struct();     % simulink model parameters
    structured_x;                 % simulink state structure (cached for efficiency)

    input_frame;  % named coordinate systems for input, state, and output
    state_frame;
    output_frame;
  end
end
