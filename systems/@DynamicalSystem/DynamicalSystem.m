classdef DynamicalSystem
% An interface class for a state-space dynamical system 
%  with a single (vector) input u 
%  and a single (vector) state x composed of a combination of continuous time and discrete time variables, which is also the output
                                                   
  methods (Abstract = true)
    n = getNumContStates(obj);
    n = getNumDiscStates(obj);
    n = getNumInputs(obj);
    n = getNumOutputs(obj);
    ts = getSampleTime(obj);
    mdl = getModel(obj);

    x0 = getInitialState(obj);

    xcdot = dynamics(obj,t,x,u);
    xdn = update(obj,t,x,u);
    y = output(obj,t,x,u);
  end
  
  % construction methods
  methods
    function newsys = cascade(sys1,sys2)
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
      newsys.time_invariant_flag = sys1.time_invariant_flag && sys2.time_invariant_flag;
      newsys.simulink_params = catstruct(sys1.simulink_params,sys2.simulink_params);
    end
    function newsys = feedback(sys1,sys2)
      % sys1 and sys2 in a feedback loop 
      % (with sys1 on the forward path, and sys2 on the return path).  
      % the output of sys1 will be the output of the new system.

      if (getNumOutputs(sys1)<1 || getNumOutputs(sys2)<1 || getNumOutputs(sys1)~=getNumInputs(sys2) || getNumOutputs(sys2)~=getNumInputs(sys1))
        error('these systems can''t be combined in feedback because they don''t have a compatible number of inputs / outputs');
      end
      
      mdl = ['Feedback_',datestr(now,'MMSSFFF')];  % use the class name + uid as the model name
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
      add_line(mdl,'system2/1','system1/1');

      add_block('simulink3/Sinks/Out1',[mdl,'/out']);
      add_line(mdl,'system1/1','out/1');
      
      newsys = SimulinkModel(mdl);      
    end
    function newsys = sampledData(sys,tsin,tsout)
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
      n = getNumDiscStates(obj) + getNumContStates(obj);
    end
    function ts = getInputSampleTimes(obj)
      mdl = getModel(obj);
      inport = find_system(mdl,'SearchDepth',1,'BlockType','Inport');
      ts = [];
      for i=1:length(inport)
        ts = [ts,Simulink.Block.getSampleTimes(inport{i}).Value'];
      end
    end
    function ts = getOutputSampleTimes(obj)
      mdl = getModel(obj);
      outport = find_system(mdl,'SearchDepth',1,'BlockType','Outport');
      ts = [];
      for i=1:length(outport)
        ts = [ts,Simulink.Block.getSampleTimes(outport{i}).Value'];
      end
    end
    
    function tf = isDT(obj)
      ts = getSampleTime(obj);
      tf = (size(ts,2)==1 && ts(1)>0); % only one sample time and continuous
%      tf = (getNumContStates(obj)==0 && getNumDiscStates(obj)>0);
    end
    function tf = isCT(obj)
      ts = getSampleTime(obj);
      tf = (size(ts,2)==1 && ts(1)==0); % only one sample time and continuous
%      tf = (getNumContStates(obj)>0 && getNumDiscStates(obj)==0);
    end
    
    function tf = isTI(obj)  % is this time invariant?
      tf = obj.time_invariant_flag;
    end
    function obj = setTIFlag(obj,bval)
      % set TI=true if you know the system is time-invariant.  it simplifies many things.
      obj.time_invariant_flag = bval;
    end

    function u = wrapInput(obj,u)
      i=find(obj.input_angle_flag);
      u(i) = mod(u(i)+pi,2*pi)-pi;
    end
    function y = wrapOutput(obj,y)
      i=find(obj.output_angle_flag);
      y(i) = mod(y(i)-pi,2*pi)-pi;
    end
    function obj = setAngleFlags(obj,in_angle_flag,state_angle_flag,out_angle_flag)
      if (~isempty(in_angle_flag))
        typecheck(in_angle_flag,{'logical','double'});
        sizecheck(in_angle_flag,[obj.getNumInputs() 1]);
        obj.input_angle_flag = in_angle_flag;
      end
      if (~isempty(state_angle_flag))
        typecheck(state_angle_flag,{'logical','double'});
        sizecheck(state_angle_flag,[obj.getNumStates() 1]);
        obj.state_angle_flag = state_angle_flag;
      end
      if (~isempty(out_angle_flag))
        typecheck(out_angle_flag,{'logical','double'});
        sizecheck(out_angle_flag,[obj.getNumOutputs() 1]);
        obj.output_angle_flag = out_angle_flag;
      end
    end
    
    function xs = stateVectorToStructure(obj,xv)
      if (isempty(obj.structured_x))
        obj.structured_x = Simulink.BlockDiagram.getInitialState(getModel(obj));
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
      % input is param_name,param_value,param_name,param_value
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
    
    function [A,B,C,D,xdot0,y0] = linearize(obj,t,x,u)
      % note: also worth looking into using linearize instead of linmod: http://www.mathworks.com/help/toolbox/slcontrol/ug/linearize.html
      mdl = getModel(obj);
      if (~strcmp(get_param(mdl,'SimulationStatus'),'stopped'))
        feval(mdl,[],[],[],'term');
      end
      [A,B,C,D] = linmod(mdl,x,u,[1e-5,t]);
      
      if (nargout>4) 
        xdot0 = dynamics(obj,t,x,u);
        if (length(xdot0)~=size(A,1)) 
          % linmod did something clever, like adding states to the model.  
          error('have to handle this case more carefully'); 
        end
        if (nargout>5)
          y0 = output(obj,t,x,u);
        end
      end
    end
    function [A,B,C,D,xn0,y0] = dlinearize(obj,ts,t,x,u)
      mdl = getModel(obj);
      if (~strcmp(get_param(mdl,'SimulationStatus'),'stopped'))
        feval(mdl,[],[],[],'term');
      end
      [A,B,C,D] = dlinmod(mdl,ts,x,u,[1e-5,t]);
      if (nargout>4) 
        xn0 = update(obj,t,x,u);
        if (length(xn0)~=size(A,1)) 
          % linmod did something clever, like adding states to the model.  
          error('have to handle this case more carefully'); 
        end
        if (nargout>5)
          y0 = output(obj,t,x,u);
        end
      end
    end
    
    function runLCMPlant(obj,lcmCoder,options)
      if (nargin<3) options = struct(); end
      runLCM(obj,lcmCoder,'u','xhat',options);
    end
    function runLCMControl(obj,lcmCoder,options)
      if (nargin<3) options = struct(); end
      runLCM(obj,lcmCoder,'xhat','u',options);
    end
    function runLCMEstimator(obj,lcmCoder,options)
      if (nargin<3) options = struct(); end
      runLCM(obj,lcmCoder,'y','xhat',options);
    end
    function runLCMVisualizer(obj,lcmCoder,options)
      if (nargin<3) options = struct(); end
      runLCM(obj,lcmCoder,'xhat',[],options);
    end

  end
  
  properties (SetAccess=private,GetAccess=private)
    time_invariant_flag = false;  % set to true if you know the system is time invariant
    simulink_params=struct();
    structured_x;  % save a little time by caching this structure
  end
  properties (SetAccess=private,GetAccess=public)
    input_angle_flag = [];
    state_angle_flag = [];
    output_angle_flag = [];
  end

end
