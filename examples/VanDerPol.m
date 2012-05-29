classdef VanDerPol < PolynomialSystem

  methods
    function obj = VanDerPol()
      obj = obj@PolynomialSystem(2,0,0,2,false);
    end
    function xdot = dynamics(obj,t,x,u)
      xdot = [x(2); -x(1)-x(2)*(x(1)^2-1)];
    end
    function y=output(obj,t,x,u)
      y=x;
    end
  end
  
  methods (Static=true)
    function run()
      % simulate the VDP to obtain the nominal limit cycle, then compute
      % the region of attraction to the origin for the time-reversed
      % system.

      vdp = VanDerPol();
      % simulate to obtain the nominal limit cycle
      x_limit_cycle = simulate(vdp,[0 6.69],[-0.1144;2.0578]);

      figure(1); clf
      ts=x_limit_cycle.getBreaks(); xlim=eval(x_limit_cycle,ts);
      fill(xlim(1,:),xlim(2,:),[0.8 0.8 0.2]);
      hold on;
      
      rvdp = timeReverse(vdp);
      options.max_iterations=100;
      options.converged_tol = 1e-2;
      options.method={'levelSet','bilinear'};
      V=regionOfAttraction(rvdp,zeros(2,1),options);
      
      clf; hold on
      fill(xlim(1,:),xlim(2,:),[0.8 0.8 0.2])
      plotFunnel(V,zeros(2,1));
    end
  end
end
