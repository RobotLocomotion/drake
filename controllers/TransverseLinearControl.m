classdef TransverseLinearControl < SmoothRobotLibSystem
% Orbital stabilization via the transverse linearization

% irm@mit.edu

methods 
    function obj=TransverseLinearControl(x0,u0,K,TransSurf,S,Sdot)
      obj = obj@SmoothRobotLibSystem(0,0,x0.dim,u0.dim,true,true);
      obj.x0 = x0;
      obj.u0 = u0;
      obj.K = K;
      obj.TransSurf = TransSurf;
      if (nargin>4)
        obj.S = S;
        if (nargin>5)
          obj.Sdot = Sdot;
        end
      end
    end
    
    function ts = getSampleTime(obj)
      % make sure that this static function uses an inherited sample time
      ts = [-1;0];  % inherited sample time
    end
    
    function u = output(obj,t,junk,x)
      % implements the actual control function
      %      x = wrap(obj,obj.x0,x);

      tau = computeTau(obj,x);
      if (isempty(tau)) u=zeros(obj.u0.dim,1); return; end  % this should go away when computeTau is working better
     
      [Pi, ~] = obj.TransSurf.getPi(tau);
      xperp = Pi*(x-obj.x0.eval(tau));
      u = obj.u0.eval(tau)-obj.K.eval(tau)*xperp;
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
      %
      % irm@mit.edu
      
      x0traj = obj.x0;
      ztraj = obj.TransSurf.z;
      
      t0 = x0traj.tspan(1);
      tf = x0traj.tspan(end);
      ts = linspace(t0,tf,1e3);
      X = x0traj.eval(ts);
      
      % Find index of xtraj with Euclidean distance closest to x as starting
      % guess
      [r index_euclid] = min(sqrt(sum((X-repmat(x,1,size(X,2))).^2,1)));
      
      % find tau with z(tau)'x = 0
      
      zs = ztraj.eval(ts);
      x0s = x0traj.eval(ts);
      xdiff = repmat(x,1,size(x0s,2))-x0s;
      zdotx = dot(zs,xdiff);
      indices_z = find(zdotx.^2<1e-3);
      if (isempty(indices_z)) tau=[]; zeroval=nan; exitflag=1; return; end
      
      [r, subindex] = min(abs(index_euclid-indices_z));
      i = indices_z(subindex);
      
      tau = ts(i);
      
      f = @(t) ztraj.eval(t)'*(x-x0traj.eval(t));
      
      zeroval = f(tau);
      exitflag = 0; %Does nothing yet, should report success
    end      

    
  end
  
  properties 
    x0=[];
    u0=[];
    K = [];
    S = [];
    Sdot = [];
    TransSurf = [];
end



end