classdef (InferiorClasses = {?DrakeSystem}) HybridDrakeSystem < DrakeSystem

  % some restrictions on the mode systems:
  %  must be CT only

  properties (SetAccess=private)
    % modes:
    modes={};       % cell array of systems representing each state (or mode)
    mode_names={};

    % transitions:
    target_mode={}; % target_mode{i}(j) describes the target for the jth transition out of the ith mode
    guard={};       % guard{i}{j} is for the jth transition from the ith mode
    transition={};      % transition{i}{j} is for the jth transition from the ith mode
  end

  % construction
  methods
    function obj = HybridDrakeSystem(num_u, num_y)
      obj = obj@DrakeSystem(0,1,num_u,num_y,false,true);
      obj = setSampleTime(obj,[-1;0]);  % use inherited sample time until modes are added
    end

    function [obj,mode_num] = addMode(obj,mode_sys,name)%,add_mode_number_to_output)
      typecheck(mode_sys,'DrakeSystem');
      if isa(mode_sys,'HybridDrakeSystem') error('hybrid modes are not supported (yet)'); end
      if isa(mode_sys,'StochasticDrakeSystem') error('stochastic modes are not supported (yet)'); end
      if (getNumStates(mode_sys)>0 && ~(mode_sys.isCT() || mode_sys.isInheritedTime())) error('only CT or inherited sample time modes are allowed for now'); end
%      if nargin<4 || isempty(add_mode_number_to_output), add_mode_number_to_output = false; end
      obj.target_mode = {obj.target_mode{:},[]};
      obj.guard = {obj.guard{:},{}};
      obj.transition = {obj.transition{:},{}};

      oldStateFrame=getStateFrame(obj);
      obj = setNumContStates(obj,max(getNumContStates(obj),getNumContStates(mode_sys)));
      obj = setNumDiscStates(obj,max(getNumDiscStates(obj),1+getNumDiscStates(mode_sys)));

      if (obj.getNumInputs>0 && obj.getInputFrame ~= mode_sys.getInputFrame)
        tf = findTransform(obj.getInputFrame,mode_sys.getInputFrame);
        if (isempty(tf))
          error(['Input frame ' mode_sys.getInputFrame().name, ' does not match input frame ', obj.getInputFrame().name, ' and I cannot find a CoordinateTransform to make the connection']);
        end
        mode_sys = cascade(tf,mode_sys);
      end
      if (obj.getNumOutputs>0 && obj.getOutputFrame ~= mode_sys.getOutputFrame)
        tf = findTransform(mode_sys.getOutputFrame,obj.getOutputFrame);
        if (isempty(tf))
          error(['Output frame ' mode_sys.getOutputFrame().name, ' does not match output frame ', obj.getOutputFrame().name, ' and I cannot find a CoordinateTransform to make the connection']);
        end
        mode_sys = cascade(mode_sys,tf);
      end

      if (getNumZeroCrossings(mode_sys)>0)
        obj = setNumZeroCrossings(obj,max(getNumZeroCrossings(obj),getNumZeroCrossings(mode_sys)));
      end
%      if (getNumStateConstraints(mode_sys)>0)
%        error('need to reimplement this');  % presumably by adding the state constraints augmented by the mode
%        obj = setNumStateConstraints(obj,max(getNumStateConstraints(obj),getNumStateConstraints(mode_sys)));
%      end
%      if getNumUnilateralConstraints(mode_sys)>0
%        error('not implemented yet');
%      end

      obj = setSampleTime(obj,[getSampleTime(obj),getSampleTime(mode_sys)]);
      obj = setDirectFeedthrough(obj,isDirectFeedthrough(obj) || isDirectFeedthrough(mode_sys));
      obj = setTIFlag(obj,isTI(obj) && isTI(mode_sys));

      obj.modes = {obj.modes{:},mode_sys};
      mode_num = length(obj.modes);

      if (getStateFrame(obj)~=oldStateFrame)
        % todo: remove old transforms (from mode states to oldStateFrame)
        % if I create those

        for i=1:mode_num  % support multiple modes have the same state frame
          tf = obj.getStateFrame.findTransform(getStateFrame(obj.modes{i}));
          if isempty(tf)
            obj.getStateFrame.addTransform(HybridStateTransform(obj,obj.modes{i},i));
          elseif isa(tf,'HybridStateTransform')
            obj.getStateFrame.updateTransform(tf.addModeNumber(i));
          end
        end
      else
        % just add the transform for this mode
        tf = obj.getStateFrame.findTransform(getStateFrame(obj.modes{mode_num}));
        if isempty(tf)
          obj.getStateFrame.addTransform(HybridStateTransform(obj,obj.modes{mode_num},mode_num));
        elseif isa(tf,'HybridStateTransform')
          obj.getStateFrame.updateTransform(tf.addModeNumber(mode_num));
        end
      end

      if (nargin>2)
        typecheck(name,'char');
        obj.mode_names{mode_num}=name;
      else
        obj.mode_names{mode_num}=int2str(mode_num);
      end
    end

    function obj = addTransition(obj,from_mode_num,guard,transition,directFeedthrough,timeInvariant,to_mode_num)
      % guard is a function pointer with style:
      %    phi = guard(fsm_obj,t,mode_x,u)
      %    with phi a scalar, which indicates a transition when phi<=0.
      % transition is a function pointer with style:
      %    [to_mode_xn,to_mode_num,status] = transition(obj,from_mode_num,t,mode_x,u)
      % if to_mode_num is not supplied, or is 0, then it means that the to_mode is only given by the transition update.

      if (from_mode_num<1 || from_mode_num>length(obj.modes)) error('invalid from mode'); end
      if (nargin<7) to_mode_num=0; end
      if (isnumeric(guard))
        error('the syntax for the hybrid models has changed (sorry).  to_mode_num is now an output of the transition function instead of an argument to addTransition.  this was necessary for more complicated transitions.  type ''help @HybridDrakeSystem/addTransition'' for details.');
        % todo: zap trailing ~ from argument list when i zap this version change notification.
      end
      typecheck(guard,{'function_handle','inline'});
      if (~isempty(transition)) typecheck(transition,{'function_handle','inline'}); end
      tid = length(obj.guard{from_mode_num})+1;
      if (to_mode_num<0 || to_mode_num>length(obj.modes)) error('invalid to mode'); end
      obj.target_mode{from_mode_num}(tid) = to_mode_num;
      obj.guard{from_mode_num}{tid} = guard;
      obj.transition{from_mode_num}{tid} = transition;
      obj = setDirectFeedthrough(obj,obj.isDirectFeedthrough() || directFeedthrough);
      obj = setTIFlag(obj,obj.isTI() && timeInvariant);
      obj = setNumZeroCrossings(obj,max(getNumZeroCrossings(obj),tid+getNumZeroCrossings(obj.modes{from_mode_num})));
    end

    function guard = andGuards(obj,varargin)
      % produces a single guard that is a logical 'and' of the variable
      % number of guards input.
      if (length(varargin)<2) error('should have at least two functions to and'); end
      for i=1:length(varargin)
        typecheck(varargin{i},{'function_handle','inline'});
      end
      guard = @(obj,t,x,u)andGuardFun(obj,t,x,u,varargin{:});
      % todo: support different levels of derivatives

      function varargout = andGuardFun(obj,t,x,u,varargin)
        varargout=cell(1,nargout);
        [varargout{:}] = varargin{1}(obj,t,x,u);
        for i=2:length(varargin)
          [phi{1:nargout}]=varargin{i}(obj,t,x,u);
          if (phi{1}>varargout{1})
            varargout=phi;
          end
        end
        % original argument (with none of the complexity of gradients)
        %  for i=1:length(varargin)
        %    phi(i) = varargin{i}(obj,t,x,u);
        %  end
        %  phi = max(phi);
      end
    end

    function guard = notGuard(obj,orig_guard)
      guard = @(obj,t,x,u)notGuardFun(obj,t,x,u,orig_guard);

      function varargout = notGuardFun(obj,t,x,u,guard)
        [varargout{1:nargout}]= guard(obj,t,x,u);
        for i=1:length(varargout)  % invert guard and all gradients
          varargout{i}=-varargout{i};
        end
        % not gradient version was:
        % phi = guard(obj,t,x,u);
        % phi = -phi;
      end
    end

    function drawModeGraph(obj)
      % depends on having graphviz2mat installed (from matlabcentral)
      % todo: make that a dependency in configure?

      A = zeros(length(obj.modes));
      for i=1:length(obj.modes)
        A(i,obj.target_mode{i}) = 1;
      end
      drawGraph(A,obj.mode_names);
    end
  end

  % access methods
  methods
    function mode_sys = getMode(obj,mode_num)
      if (mode_num<1 || mode_num>length(obj.modes)) error('bad mode num'); end
      mode_sys = obj.modes{mode_num};
    end
  end

  % implementation
  methods
    function m = getInitialMode(obj)
      m=1;
    end

    function x0 = getInitialState(obj)
      if ~isempty(obj.initial_state)
        x0 = obj.initial_state;
        return;
      end
      
      m = getInitialMode(obj);
      x0 = [m; getInitialState(obj.modes{m})];
      % pad if necessary:
      x0 = [x0;repmat(0,getNumStates(obj)-length(x0),1)];
    end

    function x0 = getInitialStateWInput(obj,t,x,u)
      m = x(1); nX = getNumStates(obj.modes{m});
      if (nX>0) xm = x(1+(1:nX)); else xm=[]; end
      x0=[x(1); getInitialStateWInput(obj.modes{m},t,xm,u)];
      % pad if necessary:
      x0 = [x0;repmat(0,getNumStates(obj)-length(x0),1)];
    end


    function [xcdot,df] = dynamics(obj,t,x,u)
      m = x(1);
      if (getNumContStates(obj.modes{m}))
        nX = getNumStates(obj.modes{m});
        if (nX>0) xm = x(1+(1:nX)); else xm=[]; end
        xcdot = dynamics(obj.modes{m},t,xm,u);
      else
        xcdot=[];
      end
      % pad if necessary:
      xcdot = [xcdot;repmat(0,getNumContStates(obj)-length(xcdot),1)];
    end

    function xdn = update(obj,t,x,u)
      m = x(1);
      if (getNumDiscStates(obj.modes{m}))
        nX = getNumStates(obj.modes{m});
        if (nX>0) xm = x(1+(1:nX)); else xm=[]; end
        xdn = [m;update(obj.modes{m},t,xm,u)];
      else
        xdn=m;
      end
      % pad if necessary:
      xdn = [xdn;repmat(0,getNumDiscStates(obj)-length(xdn),1)];
    end

    function y = output(obj,t,x,u)
      m = x(1); nX = getNumStates(obj.modes{m});
      if (nX>0) xm = x(1+(1:nX)); else xm=[]; end
      y = output(obj.modes{m},t,xm,u);
    end

    function zcs = zeroCrossings(obj,t,x,u)
      m = x(1); nX = getNumStates(obj.modes{m});
      if (nX>0) xm = x(1+(1:nX)); else xm=[]; end
      zcs=ones(getNumZeroCrossings(obj),1);
      n=length(obj.guard{m});
      for i=1:n
        zcs(i) = obj.guard{m}{i}(obj,t,xm,u);
      end
      n2=getNumZeroCrossings(obj.modes{m});
      if (n2>0)
        zcs(n+(1:n2))=zeroCrossings(obj.modes{m},t,xm,u);
      end
    end
    
    function [x,success] = resolveConstraints(obj,x,v)
      m = x(1);
      nX = getNumStates(obj.modes{m});
      xm = x(1+(1:nX));
      if (nargin<3) v=[]; end
      [x(1+(1:nX)),success] = resolveConstraints(obj.modes{m},xm,v);
    end

    function [xn,status] = transitionUpdate(obj,t,x,u)
      status = 0;
      m = x(1); nX = getNumStates(obj.modes{m});
      if (nX>0) xm = x(1+(1:nX)); else xm=[]; end
      n=length(obj.guard{m});
      zcs=ones(n,1);
      for i=1:n  % just compute the guards (zeroCrossings also computes the mode zcs)
        zcs(i) = obj.guard{m}{i}(obj,t,xm,u);
      end
%      disp(obj.mode_names{m}); zcs
      active_id = find(zcs<0);
      if (isempty(active_id)) % no transition (return the original state)
        xn = x;
        return;
      end
      if (length(active_id)>1) error('multiple guards tripped at the same time.  behavior is undefined.  consider reducing the step size'); end
      nX = getNumStates(obj.modes{m});
      if (nX>0) xm = x(1+(1:nX)); else xm=[]; end
      [mode_xn,to_mode_num,status] = obj.transition{m}{active_id}(obj,m,t,xm,u);
      if (obj.target_mode{m}(active_id)>0 && obj.target_mode{m}(active_id)~=to_mode_num)
        error('transition to_mode_num differs from specified target mode');
      end

%      disp(obj.mode_names{to_mode_num});

%      to_mode_num
      xn = [to_mode_num;mode_xn];
      % pad if necessary:
      xn = [xn;repmat(0,getNumStates(obj)-length(xn),1)];
      if(status==0)
        n=length(obj.guard{to_mode_num});
        zcs2 = ones(n,1);
        for i=1:n
          zcs2(i) = obj.guard{to_mode_num}{i}(obj,t,mode_xn,u);
        end
        if (any(zcs2<0))
          active_id2 = find(zcs2<0);
          %        disp(obj);
          for j = 1:numel(active_id2)
            warning('Drake:HybridDrakeSystem:SuccessiveZeroCrossings','Transitioned from mode %s to mode %s, and immediately triggered guard number %d\n',obj.mode_names{m},obj.mode_names{to_mode_num},active_id2(j));
          end
          %        keyboard;
        end % useful for debugging successive zcs.
      end
    end

    function obj = setOutputFrame(obj,frame)
      obj = setOutputFrame@DrakeSystem(obj,frame);
      for i=1:length(obj.modes)
        obj.modes{i}=setOutputFrame(obj.modes{i},frame);
      end
    end

    function obj = setInputFrame(obj,frame)
      obj = setInputFrame@DrakeSystem(obj,frame);
      for i=1:length(obj.modes)
        obj.modes{i}=setInputFrame(obj.modes{i},frame);
      end
    end

    function sys = feedback(sys1,sys2)
      warning('feedback combinations with hybrid systems not implemented yet.  kicking out to a simulink combination.');
      sys = feedback(SimulinkModel(sys1.getModel()),sys2);
    end
    function sys = cascade(sys1,sys2)
      warning('cascade combinations with hybrid systems not implemented yet.  kicking out to a simulink combination.');
      sys = cascade(SimulinkModel(sys1.getModel()),sys2);
    end
  end

end
