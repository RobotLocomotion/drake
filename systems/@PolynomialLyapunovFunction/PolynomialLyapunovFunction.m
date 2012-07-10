classdef PolynomialLyapunovFunction < LyapunovFunction
  
  properties (SetAccess=private,GetAccess=private)
    Vpoly;
    Vpolytraj;
    p_t
  end
  
  methods
    function obj=PolynomialLyapunovFunction(frame,Vpoly)
      obj=obj@LyapunovFunction(frame,true);
      obj.p_t = msspoly('t',1);
      sizecheck(Vpoly,[1 1]);  % must be a scalar
      if (isa(Vpoly,'msspoly'))
        obj.Vpoly=Vpoly;
        if (deg(Vpoly,obj.p_t)>0) obj.time_invariant_flag = false; end
        t=0;
      else
        typecheck(Vpoly,'PolynomialTrajectory');
        obj.Vpolytraj = Vpoly;
        obj.time_invariant_flag = false;
        t=Vpoly.tspan(1);
      end
      
      if any(match(frame.poly,decomp(getPoly(obj,t)))==0)
        error('polynomial depends on variables other than t and those defined in frame');
      end
    end
    
    function V = eval(obj,t,x)  
      if isempty(obj.Vpoly)
        V = double(subs(obj.Vpolytraj.eval(t),[obj.p_t;obj.getFrame.poly],[t;x]));
      else
        V = double(subs(obj.Vpoly,[obj.p_t;obj.getFrame.poly],[t;x]));
      end
    end
    
    function Vdot = evalDeriv(obj,sys,t,x)
      error('not implemented yet');
    end
    
    function Vpoly = getPoly(obj,t)
      if (nargin<2)
        if isTI(obj), t=0;
        else error('you must specify t'); end
      end
      if isempty(obj.Vpoly)
        Vpoly = subs(obj.Vpolytraj.eval(t),obj.p_t,t);
      else
        Vpoly=subs(obj.Vpoly,obj.p_t,t);
      end
    end
    
    function Vdotpoly = getPolyDeriv(obj,sys,t)
      error('not implemented yet');
    end
    
    function V = inFrame(obj,frame)
      if (frame==obj.getFrame)
        V = obj;
      else
        tf = findTransform(frame,obj.getFrame);
        if isempty(tf) error('couldn''t find a coordinate transform from the Lyapunov frame to the requested frame'); end
        Vpoly = subss(obj.Vpoly,obj.getFrame.poly,tf.output(0,[],frame.poly));
        V = PolynomialLyapunovFunction(frame,Vpoly);
      end
    end
  end

  % todo: move over getLevelSet, getProjection, plotFunnel, ...
end
