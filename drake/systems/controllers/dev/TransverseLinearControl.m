classdef TransverseLinearControl < DrakeSystem
% Orbital stabilization via the transverse linearization

% irm@mit.edu

methods 
    function obj=TransverseLinearControl(x0,u0,K,TransSurf,S,Sdot, Qtraj, Rtraj, plant)
      obj = obj@DrakeSystem(1,0,x0.dim,u0.dim,false,false);
      obj.x0 = x0;
      obj.u0 = u0;
      obj.plant = plant;
      obj.K = K;
      obj.Qtraj = Qtraj;
      obj.Rtraj = Rtraj;
      obj.TransSurf = TransSurf;
      if (nargin>4)
        obj.S = S;
        if (nargin>5)
          obj.Sdot = Sdot;
        end
      end
      t0 = obj.x0.tspan(1);
      tf = obj.x0.tspan(end);
      obj.tspan = [t0 tf];
      obj.breaks = linspace(t0,tf,1e3);
      obj.X = obj.x0.eval(obj.breaks);
      obj.zs = obj.TransSurf.z.eval(obj.breaks);
      
      %These should be added back in when running on the real robot since
      %evaluating K takes a long time
      
      obj.K_preEval = flipToPP(obj.K);
%      obj.K_preEval = DTTrajectory(obj.K.eval(obj.breaks), obj.breaks(2)-obj.breaks(1));
%      obj.K_preEval = obj.K_preEval.shiftTime(t0);

      obj = setInputFrame(obj,getStateFrame(plant));
      obj = setOutputFrame(obj,getInputFrame(plant));
    end
    
    function tau0 = getInitialState(obj)
      tau0 = obj.x0.tspan(1);
    end
    
    function tau0 = getInitialStateWInput(obj,~,tau0,x)
      [tau,~,flag]=computeTau(obj,x);
      if (flag==0), tau0 = tau; end
    end
    
    function taudot = dynamics(obj,t,tau,x)
        u = obj.output(t,tau,x);

        if(tau < obj.tspan(1))
            tau = obj.tspan(1);
        elseif(tau > obj.tspan(2))
            tau = obj.tspan(2);
        end

        xstar = obj.x0.eval(tau);
        ustar = obj.u0.eval(tau);
        ft = obj.plant.dynamics(t,x,u);
        fstar = obj.plant.dynamics(tau,xstar,ustar);
        z = obj.TransSurf.z.eval(tau);
        dz =  obj.TransSurf.zdot.eval(tau);


        taudot1 = z'*ft/(z'*fstar-dz'*(x-xstar));
        correction = z'*(x-xstar);
        taudot = taudot1+5*correction;
    end
    
    function ts = getSampleTime(obj)
      % make sure that this static function uses an inherited sample time
      ts = [-1;0];  % inherited sample time
    end
    
    function [u] = output(obj,t,tau,x)
      % implements the actual control function
      %      x = wrap(obj,obj.x0,x);
      if(tau>obj.tspan(1) && tau < obj.tspan(2))
        xnom = obj.x0.eval(tau);
        %scope('cg','xnom',t,xnom,struct('scope_id',3,'num_points',500, 'linespec', 'r'));
      end
      
      if(tau < obj.tspan(1))
        tau = obj.tspan(1);
      elseif(tau > obj.tspan(2))
          tau = obj.tspan(2);
      end
      
      %scope('cg','x',t,x,struct('scope_id',3,'num_points',500));
      
      %tau = computeTau(obj,x);
      if (isempty(tau)) 
          u=zeros(obj.u0.dim,1); 
          xperp = repmat(-1, 3, 1);
          return; 
      end  % this should go away when computeTau is working better
     
      [Pi, ~] = obj.TransSurf.getPi(tau);
      xperp = Pi*(x-obj.x0.eval(tau));
      u = obj.u0.eval(tau)-obj.K_preEval.eval(tau)'*xperp;
      
      %scope('cg','tau',t,tau,struct('scope_id',1,'num_points',500));
      
      %scope('cg','xperp',t,xperp,struct('scope_id',2,'num_points',500));
      
%      cost = xperp'*obj.S.eval(tau)*xperp;
      %scope('cg','xPerpSxPerp',t,cost,struct('scope_id',4,'num_points',500));
      
      %plotData(obj,x, tau, u,t)
    end
    
    
    function [tau,zeroval,exitflag] = computeTau(obj,x)
      % tau = computeTau(x0traj,ztraj,x,do_plot)
      %
      % Computes the nearest time index tau with respect to transversal
      % surfaces defined by z(tau).
      %
      % Inputs:
      %   - x0traj: the state trajectory (trajectory object)
      %   - ztraj: the definition of transversal surfaces (trajectory object)
      %   - the current state to query
      % Output:
      %   - tau: a time index of xtraj with x in the surface
      %     defined by by ztraj(tau)
      %   - zeroval: ztraj(tau)'*x, should be zero if successful
      %   - exitflag: exit flag returned from Matlab fzero.
      % irm@mit.edu

      % Find index of xtraj with Euclidean distance closest to x as starting
      % guess
      [r index_euclid] = min(sqrt(sum((obj.X-repmat(x,1,size(obj.X,2))).^2,1)));
      
      % find tau with z(tau)'x = 0
      
      xdiff = repmat(x,1,size(obj.X,2))-obj.X;
      zdotx = dot(obj.zs,xdiff);
      min_zperp = min(abs(zdotx));
      indices_z = find(abs(zdotx)<10*min_zperp);
      if (isempty(indices_z)) 
          tau=[]; 
          zeroval=nan; 
          exitflag=1; 
          return; 
      end
      
      [r, subindex] = min(abs(index_euclid-indices_z));
      i = indices_z(subindex);
      
      tau = obj.breaks(i);
%       
%       f = @(t) ztraj.eval(t)'*(x-x0traj.eval(t));
%       
%       zeroval = f(tau);
      zeroval = 0;  % for now
      exitflag = 0; %Does nothing yet, should report success
    end      

  function plotData(obj,x, tau, u,t)
    x0 = obj.x0.eval(tau); u0 = obj.u0.eval(tau);
    Q = obj.Qtraj.eval(tau); Ri = inv(obj.Rtraj.eval(tau));
    nX = length(x0); nU = length(u0);
    tmp=obj.plant;
    [ft,df] = geval(@tmp.dynamics,tau, x0, u0);
    A = df(:,1+(1:nX));
    B = df(:,nX+1+(1:nU));
    
    zt = obj.TransSurf.z.eval(tau);
    zdott = obj.TransSurf.zdot.eval(tau);
    
    [Pi, Pidot] = obj.TransSurf.getPi(tau);
    
    % derivatives of tau dynamics wrt x_perp and u
        
    dtau_dxperp = (zt'*A*Pi'+zdott'*Pi')/(zt'*ft);
    dtau_du = zt'*B/(zt'*ft);
        
    % transverse A, B and Q
    Atr = Pidot*Pi' + Pi*A*Pi' - Pi*ft*dtau_dxperp;
    Btr = Pi*B-Pi*ft*dtau_du;
        
    Qtr = Pi*Q*Pi';
    xperp = Pi*(x-x0);
    
    transverseAngle = 180/pi*acos(zt'*ft/norm(ft)/norm(zt));
    scope('cg','transAngle',t,transverseAngle,struct('scope_id',9,'num_points',500));
    scope('cg','Zperp',t,zt'*(x-x0),struct('scope_id',8,'num_points',500));
    transGrad = Atr*xperp + Btr*(u-u0);
    Grad = A*x + B*(u);
    %scope('cg','Grad',t,Grad,struct('scope_id',6,'num_points',1));
    %scope('cg','transGrad',t,transGrad,struct('scope_id',5,'num_points',1));
  end
    
  end
  
  properties 
    x0=[];
    u0=[];
    K = [];
    S = [];
    Qtraj;
    Rtraj;
    Sdot = [];
    TransSurf = [];
    tspan = [];
    x0s = [];
    breaks = [];
    zs = [];
    X = [];
    K_preEval = [];
    plant;
end



end
