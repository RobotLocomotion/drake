classdef StereographicPoly
  % effectively overloads msspoly with trig functions which are implemented using the
  % stereographic projection.
  %
  % Note: I would have loved to derive from msspoly, but I need to
  % guarantee that the btrig property is updated (or not) on *every* method call.


  % examples:
  % xdot = [ x(2); (u - m*g*lc*sin(x(1)) - b*x(2))/I ];
  % xdot = [ x(2); (u


  properties
    num;       % msspoly
    denom;     % msspoly
    btrig=[];  % boolean (of size obj.m x obj.n) which remembers if this element was ever used as a trig element
  end

  methods
    function obj = StereographicPoly(x,y,z)
      % same arguments as msspoly
      num = msspoly(x,y,z);
      btrig = repmat(false,obj.m,obj.n);
    end

    function p = getmsspoly(obj)
      p=obj.poly;
    end

    function y=sin(x)
      y=(1-x^2)/(1+x^2);
    end
  end

end
