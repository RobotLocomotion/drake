classdef SineSys < DrakeSystem
% Simple example of SOS region of attraction using Taylor approximation
%
% This is a simple follow-on example to the example in xcubed.m
%
% Consider the system given by
%   xdot = -sin(x)
% Simple inspection reveals that this system has a fixed point at each
%   x = n*pi for all integers n  (e.g., n=[..,-2,-1,0,1,2,...])
% The fixed points even n are stable, at odd n are unstable.
%
% In the example below we estimate the region of attraction to the fixed
% point at x=0 using the toolbox.  Note that we use a Taylor approximation
% to find a polynomial approximation of the system around the origin.
%
% The resulting plot demonstrates that the region of attraction estimate is
% tight for the approximate polynomial system, but only an approximation
% for the original nonlinear system (it is conservative here, but is not
% guaranteed to be always conservative).
%
% Note that the same example can be done without approximation using
% 'Trig-SOS' when we release that code.
%

  methods
    function obj = SineSys()
      obj = obj@DrakeSystem(1,0,0,1,false,true);
    end
    function xdot = dynamics(obj,t,x,u)
      xdot = -sin(x);
    end
    function y=output(obj,t,x,u)
      y=x;
    end
  end

  methods (Static=true)
    function run()
      checkDependency('sedumi');

      p=SineSys();
      pp=extractTrigPolySystem(p,struct('replace_output_w_new_state',true));
      V=tilyap(pp,Point(pp.getStateFrame,[sin(0);cos(0)]),eye(2));
      px = pp.getStateFrame.getPoly;

      xdot=msspoly('d',2);
      Vdot = diff(V.getPoly,px)*xdot;
      prog=mssprog;

      L1monom = monomials(px,0:2);
      [prog,a] = new(prog,length(L1monom)*length(xdot),'free');
      a=reshape(a,length(L1monom),length(xdot));
      L1 = a'*L1monom;
      L2monom = monomials(px,0:2);
      [prog,b] = new(prog,length(L2monom)*pp.getNumStateConstraints,'free');
      b=reshape(b,length(L2monom),pp.getNumStateConstraints);
      L2 = b'*L2monom;

      % x on unit circle and x not (0,-1) => Vdot<0
      [prhs,plhs]=getPolyDynamics(pp);
      pxcon = getPolyStateConstraints(pp);
      prog.sos=-Vdot+L1'*(plhs*xdot-prhs)+L2'*(pxcon);

      % run SeDuMi and check output
      [prog,sigma1] = new(prog,1,'pos');
      [prog,info] = sedumi(prog,sigma1,0);
      %  if (info.numerr>1)
      %    error('sedumi had numerical issues.');
      %  end
      if (info.pinf || info.dinf)
        error('problem looks infeasible.');
      end

      L1 = prog(L1);
      L2 = prog(L2);
    end

    function runTrigPolyVectorFields()
      p=SineSys();

      pp=extractTrigPolySystem(p,struct('replace_output_w_new_state',true));

      figure(1);
      xs=linspace(-2*pi,2*pi,101);
      plot(xs,p.dynamics(0,xs,[]));
      line([xs(1),xs(end)],[0 0],'Color','k');
      axisAnnotation('arrow',[-2,-1.5],[0 0],'Color','k','HeadWidth',100,'HeadLength',100);
      axisAnnotation('arrow',[2,1.5],[0 0],'Color','k','HeadWidth',100,'HeadLength',100);
      axisAnnotation('arrow',[4,4.75],[0 0],'Color','k','HeadWidth',100,'HeadLength',100);
      axisAnnotation('arrow',[-4,-4.75],[0 0],'Color','k','HeadWidth',100,'HeadLength',100);
      xlabel('x');
      ylabel('xdot');
      title('original system');
      xlim([xs(1),xs(end)]);

      figure(2);
      [s,c]=ndgrid(linspace(-1.5,1.5,31),linspace(-1.5,1.5,31));
      sdot=s; cdot=c;
      for i=1:numel(s)
        xdot=pp.dynamics(0,[s(i);c(i)],[]);
        sdot(i)=xdot(1); cdot(i)=xdot(2);
      end
      quiver(s,c,sdot,cdot);
      axisAnnotation('ellipse',[-1 -1 2 2],'Color','k');
      xlabel('s');
      ylabel('c');
      title('trig-poly system')
      axis equal
      axis(1.5*[-1 1 -1 1]);
    end

    function runTaylor()
      % create a new xcubed object
      p = SineSys();

      x0=Point(p.getStateFrame,0);

      % taylor expand around the origin (t0=0,x0=0,u0=[]) to order 3.
      pp = taylorApprox(p,0,x0,[],3);

      % compute region of attraction around x0=0
      % the levelset V<1 is the region of attraction
      V=regionOfAttraction(pp,x0);

      % put Lyapunov function back in the state frame
      V = V.inFrame(p.getStateFrame);

      % plot everything.
      xs = -5:.01:5;
      plot(xs,-sin(xs),xs,double(msubs(pp.getPolyDynamics,pp.getStateFrame.getPoly,xs)),xs,double(msubs(V.getPoly,V.getFrame.getPoly,xs))-1,xs,0*xs,'linewidth',2);
      axis([-5,5,-1.5,1.5])
      legend('-sin(xs)','poly approx','roa (V-1)');
    end
  end
end
