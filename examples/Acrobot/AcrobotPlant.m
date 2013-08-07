classdef AcrobotPlant < Manipulator & ParameterizedSystem
  
  properties
    % parameters from Spong95 (except inertias are now relative to the
    % joints)
    % axis)
    l1 = 1; l2 = 2;  
    m1 = 1; m2 = 1;  
    g = 9.81;
    b1=.1;  b2=.1;
%    b1=0; b2=0;
    lc1 = .5; lc2 = 1; 
%    I1 = 0.083 + m1*lc1^2;  I2 = 0.33 + m2*lc2^2;  
    I1=[]; I2 = [];  % set in constructor
    
    xG
    uG
  end
  
  methods
    function obj = AcrobotPlant
      obj = obj@Manipulator(2,1);
      obj = setInputLimits(obj,-10,10);
      obj.I1 = 0.083 + obj.m1*obj.lc1^2;
      obj.I2 = 0.33 + obj.m2*obj.lc2^2;

      obj = setOutputFrame(obj,obj.getStateFrame);
      
      obj.xG = Point(obj.getStateFrame,[pi;0;0;0]);
      obj.uG = Point(obj.getInputFrame,0);
      
      obj = setParamFrame(obj,CoordinateFrame('AcrobotParams',10,'p',...
        { 'l1','l2','m1','m2','b1','b2','lc1','lc2','I1','I2' }));
    end

    function findLumpedParameters(obj)
      q=msspoly('q',obj.num_q);
      s=msspoly('s',obj.num_q);
      c=msspoly('c',obj.num_q);
      qt=TrigPoly(q,s,c);
      qd=msspoly('qd',obj.num_q);
      p=obj.getParamFrame.poly;
      pobj = setParams(obj,p);
      [H,C,B] = manipulatorDynamics(pobj,qt,qd);
      
      [a,b]=decomp(getmsspoly([H(:);C(:);B(:)]),[q;s;c;qd]);
      coeff = msspoly(ones(size(b,1),1));
      for i=1:size(b,1)
        % would use prod(a'.^b(i,:)) if msspoly implemented prod
        for k=find(b(i,:))
          coeff(i) = coeff(i).*(a(k)^b(i,k));
        end        
      end
      keyboard;
    end
    
    
    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      % keep it readable:
      m1=obj.m1; m2=obj.m2; l1=obj.l1; g=obj.g; lc1=obj.lc1; lc2=obj.lc2; I1=obj.I1; I2=obj.I2; b1=obj.b1; b2=obj.b2;
      m2l1lc2 = m2*l1*lc2;  % occurs often!

      c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));
      
      h12 = I2 + m2l1lc2*c(2);
      H = [ I1 + I2 + m2*l1^2 + 2*m2l1lc2*c(2), h12; h12, I2 ];
      
      C = [ -2*m2l1lc2*s(2)*qd(2), -m2l1lc2*s(2)*qd(2); m2l1lc2*s(2)*qd(1), 0 ];
      G = g*[ m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12); m2*lc2*s12 ];
            
      % accumate total C and add a damping term:
      C = C*qd + G + [b1;b2].*qd;

      B = [0; 1];
    end
    
    % todo: also implement sodynamics here so that I can keep the
    % vectorized version?
    
    function [f,df,d2f,d3f] = dynamics(obj,t,x,u)
      f = dynamics@Manipulator(obj,t,x,u);
      if (nargout>1)
        [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function x = getInitialState(obj)
      x = .1*randn(4,1);
    end
    
    function [c,V]=balanceLQR(obj)
      Q = diag([10,10,1,1]); R = 1;
      if (nargout<2)
        c = tilqr(obj,obj.xG,obj.uG,Q,R);
      else
        if any(~isinf([obj.umin;obj.umax]))
          error('currently, you must disable input limits to estimate the ROA');
        end
        [c,V] = tilqr(obj,obj.xG,obj.uG,Q,R);
        pp = feedback(obj.taylorApprox(0,obj.xG,obj.uG,3),c);
        options.method='levelSet';
        V=regionOfAttraction(pp,V,options);
      end
    end
    
    function [utraj,xtraj]=swingUpTrajectory(obj)
      x0 = zeros(4,1); tf0 = 4; xf = double(obj.xG);

      con.u.lb = obj.umin;
      con.u.ub = obj.umax;
      con.x0.lb = x0;
      con.x0.ub = x0;
      con.xf.lb = xf;
      con.xf.ub = xf;
      con.T.lb = 2;
      con.T.ub = 6;

      options.method='dircol';
      %options.grad_test = true;
      info=0;
      while (info~=1)
        utraj0 = PPTrajectory(foh(linspace(0,tf0,21),randn(1,21)));
        tic
        [utraj,xtraj,info] = trajectoryOptimization(obj,@cost,@finalcost,x0,utraj0,con,options);
        toc
      end

      function [g,dg] = cost(t,x,u);
        R = 1;
        g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
        return;
        
        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Q = diag([10,10,1,1]);
        R = 100;
        g = sum((Q*xerr).*xerr + (R*u).*u,1);
        
        if (nargout>1)
          dgdt = 0;
          dgdx = 2*xerr'*Q;
          dgdu = 2*u'*R;
          dg = [dgdt,dgdx,dgdu];
        end
      end
      
      function [h,dh] = finalcost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
        return;
        
        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Qf = 100*diag([10,10,1,1]);
        h = sum((Qf*xerr).*xerr,1);
        
        if (nargout>1)
          dh = [0, 2*xerr'*Qf];
        end
      end  

    end

    
  end
  
end
