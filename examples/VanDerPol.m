classdef VanDerPol < PolynomialSystem

  methods
    function obj = VanDerPol()
      obj = obj@PolynomialSystem(2,0,0,2,false,true,false);
    end
    function xdot = dynamicsRHS(obj,t,x,u)
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
      fill(xlim(1,:),xlim(2,:),MITred);%[0.8 0.8 0.2]);
      hold on;
      
      rvdp = timeReverse(vdp);
      options.max_iterations=100;
      options.converged_tol = 1e-2;
      options.method={'levelSet','bilinear'};
      V=regionOfAttraction(rvdp,Point(rvdp.getStateFrame,zeros(2,1)),options);
      
      clf; hold on
      fill(xlim(1,:),xlim(2,:),MITred);%[0.8 0.8 0.2])
      plotFunnel(V);
    end
    
    function phasePortrait()
      vdp = VanDerPol();
      
      % simulate to obtain the nominal limit cycle
      x_limit_cycle = simulate(vdp,[0 6.69],[-0.1144;2.0578]);
      
      ts=x_limit_cycle.getBreaks(); xlim=eval(x_limit_cycle,ts);
      plot(xlim(1,:),xlim(2,:),'k','LineWidth',2);
      hold on;
      
%      N = 15;
      tf = .8;
%      x0=diag([5,6])*rand(2,1)-repmat([2.5;3],1,20);
 
      % obtained from running ginput once below
      x0 = [-0.697004608294931 -0.466589861751152 1.41129032258065 0.051843317972351 0.00576036866359519 0.673963133640553 -0.846774193548387 -1.77995391705069 -1.96428571428571 1.63018433179724 1.65322580645161 2.01036866359447 -1.90668202764977 0.835253456221199 -0.120967741935484;-2.2719298245614 0.657894736842106 0.640350877192983 -0.956140350877193 0.271929824561404 -0.0438596491228065 -0.412280701754386 1.30701754385965 2.23684210526316 2.62280701754386 -1.28947368421053 -2.42982456140351 -2.60526315789474 2.1140350877193 -0.429824561403509];
      N = size(x0,2);
      
      for i=1:N;
%        [a,b]=ginput(1); x0(:,i)=[a;b];
        x = simulate(vdp,[0 tf],x0(:,i));
        h=fnplt(x);
        set(h,'Color',MITred);
        xa = x.eval(tf-.02); xb=x.eval(tf);
        axisAnnotation('arrow',[xa(1),xb(1)],[xa(2),xb(2)],'Color',MITred,'HeadWidth',50,'HeadLength',50);
        drawnow;
      end
      axis([-2.5,2.5,-3,3]);
%      save vdp.mat x0;
    end
  end
end
