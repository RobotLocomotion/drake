classdef TrajectorySwingup < FiniteStateMachine
  
  methods
    function obj = TrajectorySwingup(plant)
      x0 = [0;0]; tf0 = 4; xf = [pi;0];
      utraj0 = PPTrajectory(foh(linspace(0,tf0,11),randn(1,11)));

      con.u.lb = plant.umin;
      con.u.ub = plant.umax;
      con.x0.lb = x0;
      con.x0.ub = x0;
      con.xf.lb = xf;
      con.xf.ub = xf;
      con.T.lb = 2;
      con.T.ub = 6;

      function [g,dg] = cost(t,x,u);
        xd = repmat([pi;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Q = diag([10,1]);
        R = 100;
        g = sum((Q*xerr).*xerr,1) + (R*u).*u;
        
        if (nargout>1)
          dgdt = 0;
          dgdx = 2*xerr'*Q;
          dgdu = 2*u'*R;
          dg{1} = [dgdt,dgdx,dgdu];
        end
      end
      
      function [h,dh] = finalcost(t,x)
        xd = repmat([pi;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Qf = 100*diag([10,1]);
        h = sum((Qf*xerr).*xerr,1);
        
        if (nargout>1)
          dh{1} = [0, 2*xerr'*Qf];
        end
      end
      
      options=struct();
      %options.grad_test = true;
      [utraj,xtraj,info] = dircol(plant,@cost,@finalcost,x0,utraj0,con,options);
      if (info~=1) error('failed to find a trajectory'); end

      c = tvlqr(plant,xtraj,utraj,eye(2),1,eye(2));
      obj = obj.addMode(c);
      
      lqr = PendulumLQR(plant);
      obj = obj.addMode(lqr);

      % todo: build closed-loop lqr system and do verification on it to
      % determine transition boundary.
      obj.x0 = lqr.x0;
      obj.S = lqr.S;
      obj.rho = 4;

      lqr_in = inline('(x-obj.x0)''*obj.S*(x-obj.x0) - obj.rho','obj','t','junk','x');
      lqr_out = inline('obj.rho - (x-obj.x0)''*obj.S*(x-obj.x0)','obj','t','junk','x');
      
      obj = obj.addTransition(1,2,lqr_in,[],true,true);
      obj = obj.addTransition(2,1,lqr_out,[],true,true);
      
      obj.output_mode = false;
    end
    
  end
  
  methods (Static)
    function run()
      pd = PendulumPlant;
      pv = PendulumVisualizer;
      c = TrajectorySwingup(pd);

      sys = feedback(pd,c);

      for i=1:5
        xtraj = simulate(sys,[0 6]);
        playback(pv,xtraj);
      end
    end
  end
  
  properties
    x0
    S
    rho
  end
  
end
  
