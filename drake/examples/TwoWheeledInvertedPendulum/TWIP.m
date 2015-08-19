classdef TWIP < DrakeSystem 
  
  properties
    m1 = 0.579; %mass of body
    m2 = 0.030; %mass of wheel(s)
    R = 0.040; %radius of wheel
    L = 0.105; %distance from wheel axis to CoM of body
    I1 = 0.008122; %moment of inertia of body
    I2 = 2*(30/1000)*(40/1000)^2; %moment of inertia of wheel
    g = 9.81; 
    V = 8.3; %supply voltage
    
    %motor constants
    Kt = 0.305; %torque constant
    Ke = 0.472; %back-emf constant
    Rm = 6.83; %winding resistance
    cfm = 0.000727; %motor friction coefficient
    cfw = 0.001; %wheel friction coefficient
    
    xG = zeros(4,1);
    uG = 0;
  end
  
  methods
    function obj = TWIP
      obj = obj@DrakeSystem(4,0,1,2);
      obj = setInputLimits(obj,-100, 100);
      obj = setInputFrame(obj,CoordinateFrame('TWIPInput',1,'u',{'tau'}));
      obj = setStateFrame(obj,CoordinateFrame('TWIPState',4,'x',{'alpha','beta','alphadot','betadot'}));
      obj = setOutputFrame(obj,CoordinateFrame('TWIPOutput',2,'y',{'gyroscope', 'encoder'}));
      obj = setParamFrame(obj,CoordinateFrame('TWIPParams',12,'p',...
        { 'm1','m2','R','L','I1','I2', 'Kt', 'Ke', 'Rm', 'cfm', 'cfw', 'V'}));
      obj = setParamLimits(obj,zeros(obj.getParamFrame.dim,1));
      obj = obj.setTIFlag(true);
    end
    
    function [f,df,d2f,d3f] = dynamics(obj,t,x,u)
      m1=obj.m1; m2=obj.m2; R=obj.R; g=obj.g; L=obj.L; I1=obj.I1; I2=obj.I2; 
      Kt = obj.Kt; Ke = obj.Ke; Rm = obj.Rm; cfm = obj.cfm; cfw = obj.cfw; V = obj.V;
      q = x(1:2);
      qd = x(3:4);
      
      alpha = q(1);
      alphadot = qd(1);
      
      H = [m1*L^2 + I1, m1*L*R*cos(alpha); m1*L*R*cos(alpha), I2+m1*R^2+m2*R^2];
      C = [0, 0; -m1*L*R*sin(alpha)*alphadot, 0];
      G = [-m1*g*L*sin(alpha);0];
      A = [-cfm-(2*Ke*Kt)/Rm,cfm+(2*Ke*Kt)/Rm;cfm+(2*Ke*Kt)/Rm,-cfm-cfw-(2*Ke*Kt)/Rm];
      C = (C-A)*qd + G;
      B = [(Kt*V)/(50*Rm);-(Kt*V)/(50*Rm)];
      f = [qd;H\(B*u-C)];
      
      if (nargout>1)
        [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function y = output(obj,t,x,u)
        C = 180/pi*[0,0,1,0;1,-1,0,0];
        y = C*x;
    end
    
    function x = getInitialState(obj)
      x = 0.1*randn(4,1);
    end
    
    function n = getNumPositions(obj)
      n = 2;
    end
    
    function n = getNumVelocities(obj)
      n = 2;
    end
    
    function [c,V]=balanceLQR(obj)
      Q = diag([10,1,1,1]); R = 1;
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
      x0 = [pi;0;0;0];
      xf = double(obj.xG);
      tf0 = 10;
      
      N = 50;
      prog = DircolTrajectoryOptimization(obj,N,[5 15]);
      prog = prog.addStateConstraint(ConstantConstraint(x0),1);
      prog = prog.addStateConstraint(ConstantConstraint(xf),N);
      prog = prog.addRunningCost(@cost);
      prog = prog.addFinalCost(@finalCost);
      
      traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
      
      for attempts=1:10
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
        toc
        if info==1, break; end
      end

      function [g,dg] = cost(dt,x,u)
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
          dgddt = 0;
          dgdx = 2*xerr'*Q;
          dgdu = 2*u'*R;
          dg = [dgddt,dgdx,dgdu];
        end
      end
      
      function [h,dh] = finalCost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
        return;
      end      

    end

    
  end
  
end
