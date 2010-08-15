classdef TrajectorySwingup < Control
  
  properties
    tilqr;   % lqr stabilizer at the top
    xtraj;   % nominal swingup trajectory (in x)
    utraj;   % nominal swingup trajectory (in u)
    tvlqr;   % lqr trajectory stabilizer
  end
  
  methods 
    function obj = TrajectorySwingup(system)
      obj = obj@Control(2,1);
      if (nargin>0)
        typecheck(system,'PendulumDynamics');
        
        disp('Creating TI stabilizer at the top...')
        obj.tilqr = PendulumLQR(system);

        disp('Optimizing swingup trajectory...');
        x0 = zeros(2,1); tf0 = 2;
        utraj0 = PPTrajectory(zoh(linspace(0,tf0,51),randn(1,51)));
        options = struct('maxDT',0.05,'Tmin',2,'Tmax',6,'xf',obj.tilqr.x0);
        [obj.xtraj,obj.utraj,info] = dirtran(system,@cost,@finalcost,x0,utraj0,options);
        if (info~=1) error('dirtran failed to find a trajectory'); end
        
        disp('Creating trajectory stabilizer...');
        Q = diag([10,1]); R = 100;
        obj.tvlqr = TVLQR(system,obj.xtraj,obj.utraj,Q,R,obj.tilqr.S);
        obj.tvlqr = setParent(obj.tvlqr,obj.tilqr);
        disp('done.');
      end
      
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
    end
    
    function u = control(obj,t,x,u)
      u = obj.tvlqr.control(t,x,u);
    end

  end
    
end