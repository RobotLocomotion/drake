classdef (InferiorClasses = {?ConstantTrajectory,?PPTrajectory,?FunctionHandleTrajectory}) QuadraticLyapunovFunction < PolynomialLyapunovFunction
  % x'*S*x + x'*s1 + s2
  % S,s1,s2 can be doubles or trajectories (yielding a time-varying quadratic)
  
  properties
    S;
    s1;
    s2;
  end
  
  methods
    function obj = QuadraticLyapunovFunction(frame,S,s1,s2)
      obj = obj@PolynomialLyapunovFunction(frame,true);
      typecheck(frame,'CoordinateFrame');
      n = frame.dim;
      typecheck(S,{'double','Trajectory'});
      sizecheck(S,[n n]);
      obj.S = S;
      if isa(S,'Trajectory') obj.time_invariant_flag = false; end
      
      if (nargin>2)
        typecheck(s1,{'double','Trajectory'});
        sizecheck(s1,[n 1]);
        obj.s1 = s1;
        if isa(s1,'Trajectory') obj.time_invariant_flag = false; end
      else
        obj.s1 = zeros(n,1);
      end
      
      if (nargin>3)
        typecheck(s2,{'double','Trajectory'});
        sizecheck(s2,1);
        obj.s2 = s2;
        if isa(s2,'Trajectory') obj.time_invariant_flag = false; end
      else
        obj.s2 = 0;
      end
      
      if ~isTI(obj)
        if isa(obj.S,'double'), obj.S = ConstantTrajectory(obj.S); end
        if isa(obj.s1,'double'), obj.s1 = ConstantTrajectory(obj.s1); end
        if isa(obj.s2,'double'), obj.s2 = ConstantTrajectory(obj.s2); end
      end
    end
    
    function Vpoly = getPoly(obj,t)
      x = obj.getFrame.getPoly;
      if isTI(obj)
        Vpoly = x'*obj.S*x + x'*obj.s1 + obj.s2;
      else
        Vpoly = x'*obj.S.eval(t)*x + x'*obj.s1.eval(t) + obj.s2.eval(t);
      end
    end
    
    function pVpt = getPolyTimeDeriv(obj,t)
      if isTI(obj)
        pVpt = 0;
      else
        x = obj.getFrame.getPoly;
        pVpt = x'*obj.S.deriv(t)*x + x'*obj.s1.deriv(t) + obj.s2.deriv(t);
      end
    end
    
    function V = inFrame(obj,frame)
      if (frame==obj.getFrame)
        V = obj;
      else
        % if obj.getFrame.prefix = x and frame.prefix = y, then 
        % I have V(x) and want to return V(f(y)), where x = f(y) is the 
        % transform *from frame to obj.getFrame*
        tf = findTransform(frame,obj.getFrame,struct('throw_error_if_fail',true));
        if isa(tf,'AffineSystem') && getNumStates(tf)==0  % then I can keep it quadratic
          D=tf.D;c=tf.y0;
          V = QuadraticLyapunovFunction(frame,D'*obj.S*D,D'*(obj.S+obj.S')*c + D'*obj.s1,c'*obj.S*c + c'*obj.s1 + obj.s2);
        else
          V = inFrame@PolynomialLyapunovFunction(obj,frame);
        end
      end
    end

    function V = mtimes(a,b)
      % support simple scaling of Lyapunov functions via multiplication by
      % a (scalar) double
      if ~isa(b,'PolynomialLyapunovFunction')
        % then a must be the lyapunov function.  swap them.
        tmp=a; a=b; b=tmp;
      end
      typecheck(a,{'numeric','Trajectory'});
      sizecheck(a,1);

      V = QuadraticLyapunovFunction(b.getFrame, a*b.S, a*b.s1, a*b.s2);
    end
    
    function V = extractQuadraticLyapunovFunction(obj)
      V=obj;
    end
    
    function h=plotFunnel(obj,options)  
      if nargin<2, options=struct(); end
      if ~isTI(obj) && ~isfield(options,'ts')
        % compute natural time samples from trajectory information
        tspan = [max([obj.S.tspan(1),obj.s1.tspan(1),obj.s2.tspan(1)]),min([obj.S.tspan(2),obj.s1.tspan(2),obj.s2.tspan(2)])];
        ts = unique([reshape(obj.S.getBreaks(),[],1);reshape(obj.s1.getBreaks(),[],1);reshape(obj.s2.getBreaks(),[],1)]);
        options.ts = ts(ts>=tspan(1) & ts<=tspan(2));
      end
      
      % then pass it to the main plotFunnel
      h = plotFunnel@PolynomialLyapunovFunction(obj,options);
    end
  end
end
