classdef RobotLibSystem < DynamicalSystem

  % constructor
  methods
    function obj = RobotLibSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag)
      if (nargin>0)
        obj = setNumContStates(obj,num_xc);
        obj = setNumDiscStates(obj,num_xd);
        obj = setNumInputs(obj,num_u);
        if (nargin>=4) obj = setNumOutputs(obj,num_y); end
        if (nargin>=5) obj = setDirectFeedthrough(obj,direct_feedthrough_flag); end
        if (nargin>=6) obj = setTIFlag(obj,time_invariant_flag); end
      end
      obj.uid = datestr(now,'MMSSFFF');
    end      
  end
      
  % default methods - these should be implemented or overwritten
  % 
  methods
    function x0 = getInitialState(obj)
      x0 = zeros(obj.num_xd+obj.num_xc,1);
    end
    
    function xcdot = dynamics(obj,t,x,u)
      error('systems with continuous states must implement Derivatives');
    end
    function dxcdot = dynamicsGradients(obj,t,x,u,order)
      error('not implemented yet');
      % todo: implement numerical gradients here
    end
    
    function xdn = update(obj,t,x,u)
      error('systems with discrete states must implement Update');
    end
    function dxdn = updateGradients(obj,t,x,u,order)
      error('not implemented yet');
      % todo: implement numerical gradients here
    end
    
    function y = output(obj,t,x,u)
      error('default is intentionally not implemented');
    end
    function dy = outputGradients(obj,t,x,u,order)
      error('not implemented yet');
      % todo: implement numerical gradients here
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
    function ts = getSampleTime(obj)  
      % as described at http://www.mathworks.com/help/toolbox/simulink/sfg/f6-58760.html
      % to set multiple sample times, specify one *column* for each sample
      % time/offset pair.
      ts = [0;0];  % continuous time, no offset
    end
    function tf = isDirectFeedthrough(obj)
      tf = obj.direct_feedthrough_flag;
    end
    function obj = setDirectFeedthrough(obj,tf);
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
            'UpperLimit',['[',num2str(obj.umax),']'],'LowerLimit',['[',num2str(obj.umin),']']);
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
      obj.num_x = obj.num_xc + obj.num_xd;
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
      obj.num_u = num_u;
      
       % cut umin and umax to the right size, and pad new inputs with
      % [-inf,inf]
      if (length(obj.umin)~=1 && length(obj.umin)~=num_u)
        obj.umin = [obj.umin(1:num_u); -inf*ones(max(num_u-length(obj.umin),0),1)];
      end
      if (length(obj.umax)~=1 && length(obj.umax)~=num_u)
        obj.umax = [obj.umax(1:num_u); inf*ones(max(num_u-obj.length(obj.umax),0),1)];
      end
    end
    function obj = setInputLimits(obj,umin,umax)
      % Guards the input limits to make sure it stay consistent
      
      if (length(umin)~=1 && length(umin)~=obj.num_u) error('umin is the wrong size'); end
      if (length(umax)~=1 && length(umax)~=obj.num_u) error('umax is the wrong size'); end
      if (any(obj.umax<obj.umin)) error('umin must be less than umax'); end
      obj.umin = umin;
      obj.umax = umax;
    end
    function obj = setNumOutputs(obj,num_y)
      if (num_y<0) error('num_y must be >=0'); end
      obj.num_y = num_y;
    end
  end

  % utility methods
  methods
%     function [A,B,C,D,x0dot,y0] = linearize(obj,t,x0,u0)
%       if (~isCT(obj) || getNumDiscStates(obj)>0)  % boot if it's not the simple case
%         [A,B,C,D,x0dot,y0] = linearize@DynamicalSystem(obj,t,x0,u0);
%       end
%       
%       nX = getNumContStates(obj);
%       nU = getNumInputs(obj);
%       df = dynamicsGradients(obj,t,x0,u0);
%       A = df{1}(:,1+(1:nX));
%       B = df{1}(:,nX+1+(1:nU));
%       
%       if (nargout>2)
%         dy = outputGradients(obj,t,x0,u0);
%         C = dy{1}(:,1+(1:nX));
%         D = dy{1}(:,nX+1+(1:nU));
%         if (nargout>4)
%           x0dot = dynamics(obj,t,x0,u0);
%           if (nargout>5)
%             y0 = output(obj,t,x0,u0);
%           end
%         end
%       end
%     end

    function gradTest(obj)
      if (getNumContStates(obj))
        gradTest(@dynamicsTest,0,getInitialState(obj),getDefaultInput(obj),struct('tol',.01))
      end
      function [f,df] = dynamicsTest(t,x,u)
        f = dynamics(obj,t,x,u);
        df = dynamicsGradients(obj,t,x,u,1);
      end
      if (getNumDiscStates(obj))
        gradTest(@updateTest,0,getInitialState(obj),getDefaultInput(obj),struct('tol',.01))
      end
      function [f,df] = updateTest(t,x,u)
        f = update(obj,t,x,u);
        df = updateGradients(obj,t,x,u,1);
      end
      if (getNumOutputs(obj))
        gradTest(@outputTest,0,getInitialState(obj),getDefaultInput(obj),struct('tol',.01))
      end
      function [f,df] = outputTest(t,x,u)
        f = output(obj,t,x,u);
        df = outputGradients(obj,t,x,u,1);
      end
    end
    

  end
  
  properties (SetAccess=private, GetAccess=protected)
    num_xc=0; % number of continuous state variables
    num_xd=0; % number of dicrete(-time) state variables
    num_x=0;  % dimension of x (= num_xc + num_xd)
    num_u=0;  % dimension of u
    num_y=0;  % dimension of the output y
    uid;    % unique identifier for simulink models of this block instance
    direct_feedthrough_flag=true;  % true/false: does the output depend on u?  set false if you can!
  end
  properties (SetAccess=private, GetAccess=public)
    umin=-inf;   % constrains u>=umin (default umin=-inf)
    umax=inf;    % constrains u<=uman (default umax=inf)
  end
end
