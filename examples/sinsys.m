classdef sinsys < SmoothRobotLibSystem

  methods
    function obj = sinsys()
      obj = obj@SmoothRobotLibSystem(1,0,0,1,false,true);
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
      % create a new xcubed object
      p = sinsys();
      
      % taylor expand around the origin (t0=0,x0=0,u0=[]) to order 3.
      pp = taylorApprox(p,0,0,[],3)
      
      % compute region of attraction around x0=0       
      % the levelset V<1 is the region of attraction
      V=regionOfAttraction(pp,0)

      % plot everything.
      xs = -5:.01:5;
      plot(xs,-sin(xs),xs,double(msubs(pp.p_dynamics,pp.p_x,xs)),xs,double(msubs(V,pp.p_x,xs))-1,xs,0*xs,'linewidth',2);
      axis([-5,5,-1.5,1.5])
      legend('-sin(xs)','poly approx','roa (V-1)');
    end
  end
end
