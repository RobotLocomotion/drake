classdef LyapunovFunction 
% interface class for Lyapunov functions
  
  properties (SetAccess=protected,GetAccess=private)
    frame
    time_invariant_flag = false;
  end

  methods (Abstract=true)
    V = eval(obj,t,x);  
%    plotLevelSet();
  end
  
  methods 
    function obj=LyapunovFunction(frame,time_invariant_flag)
      typecheck(frame,'CoordinateFrame');
      obj.frame = frame;
      if (nargin>1)
        typecheck(time_invariant_flag,'logical');
        obj.time_invariant_flag = time_invariant_flag;
      end
    end
    
    function y = getLevelSet(obj,t,options)
      % returns the 1 sub-level set of the lyapunov function
      
      if (nargin<2) options=struct(); end
      if (~isfield(options,'tol')) options.tol = 1e-2; end % default tolerance of fplot
      if ~isfield(options,'num_samples') options.num_samples = 100; end
      if ~isfield(options,'x0') options.x0 = zeros(size(x)); end

      if ~isfield(options,'plotdims') options.plotdims = 1:obj.frame.dim; end
      if length(options.plotdims)~=2 
        error('not implemented yet');
      end

      if eval(obj,t,options.x0)>1  % assume star convexity
        error('x0 is not in the one sub level-set of V');
      end

      tv = 0*options.x0;
%      function r = getRadius(theta)
%        theta
%        tv(options.plotdims(1)) = sin(theta);
%        tv(options.plotdims(2)) = cos(theta);
%        r = fminbnd(@(a) (1-obj.eval(t,options.x0+a*tv))^2,eps,1e3);
%      end
      
%      [theta,r] = fplot(@getRadius,[-pi,pi],options.tol);
%      tv = zeros(obj.frame.dim,length(theta));
%      tv(options.plotdims(1),:) = sin(theta);
%      tv(options.plotdims(2),:) = cos(theta);
%      y = repmat(options.x0,1,length(theta)) + repmat(r',obj.frame.dim,1).*tv;
      
       theta = linspace(-pi,pi,options.num_samples);

       i=1;
       tv(options.plotdims(1)) = sin(theta(i));
       tv(options.plotdims(2)) = cos(theta(i));
       r = fminbnd(@(a) (1-obj.eval(t,options.x0+a*tv))^2,eps,1e3);
       y(:,i) = options.x0+r*tv;
       for i=2:length(theta)
         tv(options.plotdims(1)) = sin(theta(i));
         tv(options.plotdims(2)) = cos(theta(i));
         r = fminbnd(@(a) (1-obj.eval(t,options.x0+a*tv))^2,.5*r,2*r);
         y(:,i) = options.x0+r*tv;
       end
    end
    
    function fr=getFrame(obj)
      fr = obj.frame;
    end
    
    function obj=setFrame(obj,fr) 
      typecheck(fr,'CoordinateFrame');
      valuecheck(fr.dim,obj.frame.dim);
      obj.frame = fr;
    end
    
    function V=inFrame(obj,frame)
      if (frame==obj.getFrame)
        V = obj;
      else
        % if obj.getFrame.prefix = x and frame.prefix = y, then 
        % I have V(x) and want to return V(f(y)), where x = f(y) is the 
        % transform *from frame to obj.getFrame*
        tf = findTransform(frame,obj.getFrame,struct('throw_error_if_fail',true));
        if getNumStates(tf)>0 error('not implemented yet'); end
        Vfun = @(t,x) obj.eval(t,tf.output(t,[],x));
        V = FunctionHandleLyapunovFunction(frame, Vfun, obj.isTI() && tf.isTI());
      end
    end
    
    function obj = setTIFlag(obj,time_invariant_flag)
      obj.time_invariant_flag = time_invariant_flag;
    end
    
    function b = isTI(obj)
      b = obj.time_invariant_flag;
    end
  end
end
