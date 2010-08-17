classdef TVLQRControl < LQRControl

% Time-varying Linear-quadratic Regulator Policy
% 
% More info here...
%
% Type <a href="matlab: help ltv_lqr_policy>ltv_lqr_policy">help ltv_lqr_policy>ltv_lqr_policy</a> for help with the constructor.

  
  properties
    % these should all be trajectory objects
    x0=[]; u0=[]; K=[]; S=[]; Sdot=[]; rho=[];  
    Q=[]; R=[]; Qf=[];
    tspan;
    dynamics;
  end
  
  properties (SetAccess=protected,GetAccess=public)
    parent=[];  % since these are handle classes, this is a pointer to
                % another lqr_policy class, which should be executed after 
                % the final time of this controller
  end
  
  methods
    function obj = TVLQRControl(dynamics,x0traj,u0traj,Q,R,Qf)
      obj = obj@LQRControl(size(Q,1),size(R,1));
      if (nargin>0)
        obj.dynamics = dynamics;
        obj.x0 = x0traj;
        obj.u0 = u0traj;
        
        obj.tspan = x0traj.getBreaks();
        obj.tspan = obj.tspan([1,end]);
        
        if (~isa(Q,'Trajectory'))
          Q = PPTrajectory(zoh(obj.tspan([1,end]),repmat(Q,[1 1 2])));
        end
        if (~isa(R,'Trajectory'))
          R = PPTrajectory(zoh(obj.tspan([1,end]),repmat(R,[1 1 2])));
        end
      
        obj.Q = Q;
        obj.R = R;
        obj.Qf = Qf;
        obj = obj.design();
      end
    end
    
    function [obj,K,S,Sdot] = design(obj)
      obj.S = matrixODE(obj.dynamics.ode_solver,@(t,S)Sdynamics(t,S,obj.dynamics,obj.Q,obj.R,obj.x0,obj.u0),obj.tspan(end:-1:1),obj.Qf,obj.dynamics.ode_options);
      obj.K = FunctionHandleTrajectory(@(t)Ksoln(t,obj.dynamics,obj.S,obj.R,obj.x0,obj.u0),[obj.x0.dim obj.u0.dim],obj.S.getBreaks());
      obj.Sdot = FunctionHandleTrajectory(@(t)Sdynamics(t,obj.S.eval(t),obj.dynamics,obj.Q,obj.R,obj.x0,obj.u0),[obj.x0.dim obj.x0.dim],obj.S.getBreaks());

      if (nargout>1)
        K = obj.K;
        S = obj.S;
        Sdot = obj.Sdot;
      end
      
      function Sdot = Sdynamics(t,S,dynamics,Qtraj,Rtraj,xtraj,utraj)
        x0 = xtraj.eval(t); u0 = utraj.eval(t);
        Q = Qtraj.eval(t); Ri = inv(Rtraj.eval(t));
        nX = length(x0); nU = length(u0);
        df = dynamics.dynamicsGradients(t,x0,u0);
        A = df{1}(:,1+(1:nX));
        B = df{1}(:,nX+1+(1:nU));
        Sdot = -(Q - S*B*Ri*B'*S + S*A + A'*S);
      end
      
      function K = Ksoln(t,dynamics,Straj,Rtraj,xtraj,utraj)
        S = Straj.eval(t); Ri = inv(Rtraj.eval(t)); x0=xtraj.eval(t); u0 = utraj.eval(t);
        nX = length(x0); nU = length(u0);
        df = dynamics.dynamicsGradients(t,x0,u0);
        B = df{1}(:,nX+1+(1:nU));
        K = Ri*B'*S;
      end
    end
    
    function obj = setParent(obj,p)
      if (~isa(p,'LQRControl')) error('parents must be an lqr_policy class'); end
      obj.parent = p;
    end
    
    function u = control(obj,t,x)
      if (t<obj.tspan(1)) error('out of my time range'); end
      if (t>obj.tspan(end) && ~isempty(obj.parent))
        u =control(obj.parent,t,x);
      else  
        x0 = obj.x0.eval(t);
        u0 = obj.u0.eval(t);
        K = obj.K.eval(t);

        x = wrap(obj,x0,x);
        u = u0 - K*(x-x0);
      end
    end

    function J = costToGo(obj,t,x)
      if (length(t)>1) error('don''t handle the multiple time case yet (but should be simple)'); end

      x0 = obj.x0.eval(t);
      S = obj.S.eval(t);
      
      x = wrap(obj,x0,x);
      xbar = x-repmat(x0,1,size(x,2));
      J = sum((S*xbar).*xbar,1);  
    end

    function [obj,rhotraj] = verify(obj,rhof,options)
    %rhotraj = tvlqr_verify(@(t,x,u,order)PLANTFUN(obj.dynamics,t, ...
    %                                                x,u,order),obj.x0,obj.u0,obj.K,obj.S,obj.Sdot,rhof,options);
      rhotraj = invset_tvlq(obj.dynamics,obj.x0,obj.u0,obj.K,obj.S,rhof,options);

      obj.rho = rhotraj;
    end
    
    function [bVerified,verified_time,confidence] = isVerified(obj,x,time_to_check)
      %   IsVerified
      %   Check if the input is inside the ltv "funnel".  
      %   This function is vectorized (can take in x=[x1,x2,...] very
      %   efficiently)
      %
      if (isempty(obj.rho))  % need to run verify() first!
        bVerified = false;
        verified_time = 0; 
        confidence = -inf;
        return;
      end
      
      function c=evalFunnel(t,x)
        J = costToGo(obj,t,x);
        rho=obj.rho.eval(t);
        c = rho - J;
      end

      if (nargin>4 && ~isempty(tcheck)) % just evaluate it at a single time.
        confidence=evalFunnel(time_to_check,x)>0;
        bVerified = confidence>0;
        verified_time=time_to_check;
      else % search over time (approximates it by checking all knot points, then running an optimization around the best knot point)
        breaks = obj.x0.getBreaks();
        for i=1:length(breaks)
          c(i,:) = evalFunnel(breaks(i),x);
        end
        % to do: add search between knot points
        [confidence,j] = max(c,[],1);
        bVerified=confidence>0;
        verified_time=breaks(j);
      end

    end

    function plotNominal(obj,options)
      nX=length(obj.x0.eval(obj.tspan(1)));
      if (nX~=2) error('not handled yet'); end
        % todo: handle plotDims, etc.
      breaks=obj.x0.getBreaks();
      pts = obj.x0.eval(breaks);
      panels = [];
      if (isempty(obj.bWrap) || isempty(obj.xWrap))
        plot(pts(1,:),pts(2,:),'b.-','LineWidth',1,'MarkerSize',5);
      else
        for i=1:nX %find(obj.bWrap)
          if (obj.bWrap(i))
            panel = unique(floor((pts(i,:)-obj.xWrap(i,1))/(obj.xWrap(i,2)-obj.xWrap(i,1))));
            % plot entire trajectory over, shifted appropriately for each unique panel
            % (the easiest way to get the lines across the
            % boundaries correct)
            panels = [repmat(panels,1,length(panel)); reshape(repmat(panel,max(length(panels),1),1),1,[])];
          else
            panels = [panels;zeros(1,max(1,size(panels,2)))];
          end
        end
        for i=1:size(panels,2)
          shift = panels(:,i).*(obj.xWrap(:,2)-obj.xWrap(:,1));
          shift(isnan(shift)) = 0;
          ppts = pts - repmat(shift,1,size(pts,2));
          plot(ppts(1,:),ppts(2,:),'b.-','LineWidth',1,'MarkerSize',5);
        end
      end        
    end
    
    function plotFunnel(obj,options)
      % Plot Funnel (2D)
      %  Options:
      %    Nsamples
      %    color
      if (isempty(obj.rho))
        error('You must run verify() first'); 
      end
      
      nX=length(obj.x0.eval(obj.tspan(1)));
      if (nargin<2) options = struct(); end
      if (~isfield(options,'Nsamples')) options.Nsamples=30; end
      if (~isfield(options,'color')) options.color = .7*[1 1 1]; end
      % todo: handle plotDims, etc.

      panels = [];
      breaks = obj.x0.getBreaks();
      pts = obj.x0.eval(breaks);
      for i=1:nX %find(obj.bWrap)
        if (~isempty(obj.bWrap) && obj.bWrap(i))
          panel = unique(floor((pts(i,:)-obj.xWrap(i,1))/(obj.xWrap(i,2)-obj.xWrap(i,1))));
          % plot entire trajectory over, shifted appropriately for each unique panel
          % (the easiest way to get the lines across the
          % boundaries correct)
          panel = (min(panel)-1):(max(panel)+1); % add extra on the boundary to be conservative (funnel might wrap even if nominal trajectory does not.
          panels = [repmat(panels,1,length(panel)); reshape(repmat(panel,max(length(panels),1),1),1,[])];
        else
          panels = [panels;zeros(1,size(panels,2))];
        end
      end
      
      theta = linspace(0,2*pi,options.Nsamples);
      xt = [sin(theta);cos(theta)];
      
      for p=1:size(panels,2)
        shift = panels(:,p).*(obj.xWrap(:,2)-obj.xWrap(:,1));
        shift(isnan(shift)) = 0;
      
        for i=(length(breaks)-1):-1:1
          t0=breaks(i);
          t1=breaks(i+1)-eps;
          
          x0=obj.x0.eval(t0) - shift;
          x1=obj.x0.eval(t1) - shift;
          S0=obj.S.eval(t0);
          S1=obj.S.eval(t1);
          rho0=obj.rho.eval(t0);
          rho1=obj.rho.eval(t1);
          
          X0 = sqrt(rho0)*S0^(-.5)*xt+repmat(x0,1,options.Nsamples);
          X1 = sqrt(rho1)*S1^(-.5)*xt+repmat(x1,1,options.Nsamples);
          if (i==length(breaks)-1)
            plot3(X1(1,:),X1(2,:),repmat(.1,1,options.Nsamples),'k','Linewidth',2);
          end
          
          X = [X0,X1(:,end:-1:1)];
          
          k = convhull(X(1,:),X(2,:));
          fill3(X(1,k),X(2,k),repmat(0,1,length(k)),options.color,'LineStyle','none');
          plot3(X(1,k),X(2,k),repmat(-.1,1,length(k)),'k','Linewidth',5);
        end
        plot3(X0(1,:),X0(2,:),repmat(.1,1,options.Nsamples),'k','Linewidth',2);
      end
    end
    
    function plotFunnel3(obj,options)
      if (isempty(obj.rho))
        error('You must run verify() first'); 
      end
      nX=length(obj.x0.eval(obj.tspan(1)));
      if (nX~=2) error('only handles two dimensions so far'); end
      % todo: handle options like plotDims for nX>2
      if (nargin<2) options = struct(); end
      if (~isfield(options,'Nsamples')) options.Nsamples=30; end
      if (~isfield(options,'color')) options.color = .7*[1 1 1]; end
      
%      cc = .95*rand(1,3);
      breaks = obj.x0.getBreaks();
      tf = breaks(end);
      N = options.Nsamples;
      theta = linspace(0,2*pi,options.Nsamples);
      xt = [sin(theta);cos(theta)];
      
      for i=1:length(breaks-1)
        t0=breaks(i);
        t1=breaks(i+1)-eps;
        
        x0=obj.x0.eval(t0) - shift;
        x1=obj.x0.eval(t1) - shift;
        S0=obj.S.eval(t0);
        S1=obj.S.eval(t1);
        rho0=obj.rho.eval(t0);
        rho1=obj.rho.eval(t1);

        X0 = sqrt(rho0)*S0^(-.5)*xt+repmat(x0,1,N);
        X1 = sqrt(rho1)*S1^(-.5)*xt+repmat(x1,1,N);
    
        h=fill3([X0(1,1:(N-1));X0(1,2:N);X1(1,2:N);X1(1,1:(N-1));X0(1,1:(N-1))],[X0(2,1:(N-1));X0(2,2:N);X1(2,2:N);X1(2,1:(N-1));X0(2,1:(N-1))],tf-[t0;t0;t1;t1;t0],options.color);
        set(h,'LineStyle','none','FaceLighting','phong');
      end
      light('Position',[1 0 0],'Style','infinite');
      light('Position',[-1 0 0],'Style','infinite');
    end    
  end
end
