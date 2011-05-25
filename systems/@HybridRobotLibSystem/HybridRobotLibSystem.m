classdef HybridRobotLibSystem < RobotLibSystem

  % some restrictions on the mode systems:
  %  must be CT only 
  
  properties (SetAccess=private)
    modes={};       % cell array of systems representing each state (or mode)
    % transitions:
    guard={};       % guard{i}{j} is for the jth transition from the ith mode
    transition={};      % transition{i}{j} is for the jth transition from the ith mode
    num_zcs = 0;    % number of zero-crossings.
    output_mode = true;
  end
  
  % construction
  methods
    function obj = HybridRobotLibSystem()
      obj = obj@RobotLibSystem(0,1,0,1,false,true);
    end
    
    function obj = setModeOutputFlag(obj,tf)
      if (obj.output_mode) obj=setNumOutputs(obj,getNumOutputs(obj)-1); end
      obj.output_mode = logical(tf);
      if (obj.output_mode) obj=setNumOutputs(obj,getNumOutputs(obj)+1); end
    end
    
    function [obj,mode_num] = addMode(obj,mode_sys)
      typecheck(mode_sys,'SmoothRobotLibSystem');
%      if (getNumStates(mode_sys)>0 && ~mode_sys.isCT()) error('only CT modes are allowed for now'); end
      obj.modes = {obj.modes{:},mode_sys};
      mode_num = length(obj.modes);
      obj.guard = {obj.guard{:},{}};
      obj.transition = {obj.transition{:},{}};
      obj = setNumContStates(obj,max(getNumContStates(obj),getNumContStates(mode_sys)));
      obj = setNumDiscStates(obj,max(getNumDiscStates(obj),1+getNumDiscStates(mode_sys)));
      obj = setNumInputs(obj,max(getNumInputs(obj),getNumInputs(mode_sys)));
      if (obj.output_mode)
        obj = setNumOutputs(obj,max(getNumOutputs(obj),1+getNumOutputs(mode_sys)));
      else
        obj = setNumOutputs(obj,max(getNumOutputs(obj),getNumOutputs(mode_sys)));
      end
      obj = setDirectFeedthrough(obj,isDirectFeedthrough(obj) || isDirectFeedthrough(mode_sys));
      obj = setTIFlag(obj,isTI(obj) && isTI(mode_sys));
    end
    
    function obj = addTransition(obj,from_mode_num,guard,transition,directFeedthrough,timeInvariant,~)
      % guard is a function pointer with style:  
      %    phi = guard(fsm_obj,t,mode_x,u)
      %    with phi a scalar, which indicates a transition when phi<=0.
      % transition is a function pointer with style:
      %    [to_mode_xn,to_mode_num,status] = transition(obj,from_mode_num,t,mode_x,u)
      
      if (from_mode_num<1 || from_mode_num>length(obj.modes)) error('invalid from mode'); end
      if (isnumeric(guard))
        error('the syntax for the hybrid models has changed (sorry).  to_mode_num is now an output of the transition function instead of an argument to addTransition.  this was necessary for more complicated transitions.  type ''help @HybridRobotLibSystem/addTransition'' for details.'); 
        % todo: zap trailing ~ from argument list when i zap this version change notification.
      end
      typecheck(guard,{'function_handle','inline'});
      if (~isempty(transition)) typecheck(transition,{'function_handle','inline'}); end
      tid = length(obj.guard{from_mode_num})+1;
      obj.guard{from_mode_num}{tid} = guard;
      obj.transition{from_mode_num}{tid} = transition;
      obj = setDirectFeedthrough(obj,obj.isDirectFeedthrough() || directFeedthrough);
      obj = setTIFlag(obj,obj.isTI() && timeInvariant);
      obj.num_zcs = max(obj.num_zcs,tid);
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
    
  end

  % access methods
  methods 
    function mode_sys = getMode(obj,mode_num)
      if (mode_num<1 || mode_num>length(systems)) error('bad mode num'); end
      mode_sys = systems{mode_num};
    end
    
    function n = getNumZeroCrossings(obj)
      n = obj.num_zcs;
    end
  end
  
  % implementation
  methods
    function m = getInitialMode(obj)
      m=1;
    end

    function x0 = getInitialState(obj)
      m = getInitialMode(obj);
      x0 = [m; getInitialState(obj.modes{m})];
      % pad if necessary:
      x0 = [x0;repmat(0,getNumStates(obj)-length(x0),1)];
    end

    function x0 = getInitialStateWInput(obj,t,x,u);
      m = x(1); 
      xm = x(1+(1:getNumStates(obj.modes{m})));
      x0=[x(1); getInitialStateWInput(obj.modes{m},t,xm,u)];
      % pad if necessary:
      x0 = [x0;repmat(0,getNumStates(obj)-length(x0),1)];
    end
    
    function xcdot = dynamics(obj,t,x,u)
      m = x(1); 
      if (getNumContStates(obj.modes{m}))
        xm = x(1+(1:getNumStates(obj.modes{m})));
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
        xm = x(1+(1:getNumStates(obj.modes{m})));
        xdn = update(obj.modes{m},t,xm,u);
      else
        xdn=[];
      end
      % pad if necessary:
      xdn = [xdn;repmat(0,getNumDiscStates(obj)-length(xdn)-1,1)];
    end
    
    function y = output(obj,t,x,u)
      m = x(1);
      xm = x(1+(1:getNumStates(obj.modes{m})));
      y = output(obj.modes{m},t,xm,u);
      if (obj.output_mode) y = [m;y]; end
      % pad if necessary:
      y = [y;repmat(0,getNumOutputs(obj)-length(y),1)];
    end
    
    function zcs = guards(obj,t,x,u)
      m = x(1);
      xm = x(1+(1:getNumStates(obj.modes{m})));
      zcs=ones(obj.num_zcs,1);
      n=length(obj.guard{m});
      for i=1:n
        zcs(i) = obj.guard{m}{i}(obj,t,xm,u);
      end
    end
    
    function [xn,status] = transitionUpdate(obj,t,x,u)
      status = 0;
      m = x(1);
      zcs = guards(obj,t,x,u);
      active_id = find(zcs<0);
      if (isempty(active_id)) % no transition (return the original state)
        xn = x;
        return;
      end
      if (length(active_id)>1) error('multiple guards tripped at the same time.  behavior is undefined.  consider reducing the step size'); end
      xm = x(1+(1:getNumStates(obj.modes{m})));
      [mode_xn,to_mode_num,status] = obj.transition{m}{active_id}(obj,m,t,xm,u);
%      to_mode_num
      xn = [to_mode_num;mode_xn];
      % pad if necessary:
      xn = [xn;repmat(0,getNumStates(obj)-length(xn),1)];
      if (any(guards(obj,t,xn,u)<0)),
        zcs2=guards(obj,t,xn,u);
        active_id2 = find(zcs2<0);
        disp(obj);
        fprintf('transitioned from mode %d to mode %d, and immediately triggered mode %d''s guard number %d\n',m,to_mode_num,to_mode_num,active_id2);
        keyboard; 
      end % useful for debugging successive zcs.
    end
  end
  
end
