classdef TILQRControl < LQRControl
% Time-invariant LQR control (Continuous Time)
%
% See also: TVLQRControl, TIDLQRControl
  
  properties
    plant=[]; 
    x0=[]; 
    u0=[]; 
    K=[]; 
    S=[]; 
    rho=[]; 
  end
  
  methods 
    function obj=TILQRControl(plant,x0,u0,Q,R)
      % Construct and design the LQR controller
      obj = obj@LQRControl(length(x0),length(u0));
      if (nargin>0)
        obj.plant = plant;
        obj.x0 = x0;
        obj.u0 = u0;
        obj = obj.design(Q,R);
      end
    end

    function [obj,K,S] = design(obj,Q,R)
      % extracts A and B from dynamics and call matlab's lqr
      nX = length(obj.x0);
      nU = length(obj.u0);

      xdot = obj.plant.dynamics(0,obj.x0,obj.u0);
      if (any(abs(xdot)>1e-6)) error('you can only do tilqr around a fixed point'); end 
      df = obj.plant.dynamicsGradients(0,obj.x0,obj.u0);

      A = full(df{1}(:,1+(1:nX)));
      B = full(df{1}(:,nX+1+(1:nU)));
      
      [K,S] = lqr(A,B,Q,R);
      obj.K = K;
      obj.S = S;
    end
    
    function u = control(obj,t,x)
      % implements the actual LQR controller
      x = wrap(obj,obj.x0,x);
      xbar = x-obj.x0;
      u = obj.u0-obj.K*xbar;
    end
    
    function du = controlGradients(obj,t,x,order)
      % Computes Taylor expansion of the control function
      if (nargin<4) order=1; end
      du{1} = [zeros(1,size(K,1)),-K];
      if (order>1) error('not implemented yet');  end % it's all zeros... just need to fill in the struct 
    end
    
    function J = costToGo(obj,t,x)
      % Evaluates x'*S*x using the Riccati soln
      x = wrap(obj,obj.x0,x);
      xbar = x-repmat(obj.x0,1,size(x,2));
      J = sum((obj.S*xbar).*xbar,1);
    end

    function [obj,rho] = verify(obj,options)
      % Estimates the region of attraction of the controller
      if (nargin<3) options = struct(); end
      if (obj.control_dt>0) error('not implemented yet'); end
      %      rho =
      %      tilqr_verify(obj.plant,obj.x0,obj.u0,obj.K,obj.S,options);
      disp('estimating region of attraction...');
      rho = roaTILQ(obj.plant,obj.x0,obj.u0,obj.K,obj.S,options);
      disp('done estimating roa.');
      obj.rho = rho;
    end
    
    function [bVerified,verified_time,confidence] = isVerified(obj,x,time_to_check)
      % Check if the current state is inside the estimated region of attraction
      if (isempty(obj.rho))  % need to run verify() first
        bVerified = false;
        verified_time = 0; 
        confidence = -inf;
        return;
      end
      confidence = obj.rho - costToGo(obj,0,x);
      bVerified = confidence>=0;
      if (nargout>1) verified_time = repmat(0,size(confidence)); end
    end
    
    function plotNominal(obj,options)
      % plots the goal
      nX=length(obj.x0);
      if (nargin<2) options=struct(); end
      if (~isfield(options,'plotDims')) options.plotDims = [1,2]; end
      
      pd=options.plotDims; N=size(pd,1);
      if (N==1)
        plot(obj.x0(pd(1)),obj.x0(pd(2)),'x','MarkerSize',2);
        xlabel(['x(',num2str(pd(1)),')']);
        ylabel(['x(',num2str(pd(2)),')']);
      else
        rows=ceil(sqrt(N));
        cols=ceil(N/rows);
        for i=1:N
          subplot(rows,cols,i);
          plot(obj.x0(pd(i,1)),obj.x0(pd(i,2)),'x','MarkerSize',2);
          xlabel(['x(',num2str(pd(i,1)),')']);
          ylabel(['x(',num2str(pd(i,2)),')']);
        end
      end
    end
    
    function plotFunnel(obj,options)
      % plots the funnel (in 2D)
      % Plot Funnel (2D)
      %  Options:
      %    Nsamples
      %    color
      if (isempty(obj.rho))
        error('You must run verify() first'); 
      end
      
      if (nargin<2) options = struct(); end
      if (~isfield(options,'Nsamples')) options.Nsamples=30; end
      if (~isfield(options,'color')) options.color = .7*[1 1 1]; end
      % todo: handle plotDims, etc.
      
      T = obj.S^(-.5);
      theta = linspace(0,2*pi,options.Nsamples);
      xt = [sin(theta);cos(theta)];
      r = sqrt(obj.rho);
      X = r*T*xt+repmat(obj.x0,1,options.Nsamples);
      
      h=fill(X(1,:),X(2,:),options.color,'LineWidth',2);
    end
    
    function plotSat(obj,domain,options)
      % plots the input saturation constraints in state space
      
      % todo: handle plotDims, etc.

      domain=reshape(domain,[],1);  % make sure it's a column vector
      for i=1:length(obj.u0)
        line(domain,-(obj.umax-obj.u0(i)+obj.K(i,1)*(domain-obj.x0(1)))/obj.K(i,2)+obj.x0(2),'LineWidth',2,'LineStyle','--','Color',[0 0 0]);
        line(domain,-(obj.umin-obj.u0(i)+obj.K(i,1)*(domain-obj.x0(1)))/obj.K(i,2)+obj.x0(2),'LineWidth',2,'LineStyle','--','Color',[0 0 0]);
      end
    end
        
    function plotFunnel3(obj,options)
      % plots x'*Sx on the z-axis, to visualize the Lyapunov function. 
      if (isempty(obj.rho))
        error('You must run verify() first'); 
      end
      
      if (nargin<2) options = struct(); end
      if (~isfield(options,'Nsamples')) options.Nsamples=30; end
      if (~isfield(options,'Nrhosamples')) options.Nrhosamples=10; end
      if (~isfield(options,'Zscale')) options.Zscale=1; end
      % todo: handle color, plotDims, etc.
      
      T = obj.S^(-.5);
      theta = linspace(0,2*pi,options.Nsamples);
      xt = [sin(theta);cos(theta)];
      rho = linspace(0,obj.rho,options.Nrhosamples);
      r = sqrt(rho);
      x = T*xt;
      X = r'*x(1,:) + obj.x0(1);
      Y = r'*x(2,:) + obj.x0(2);
      Z = repmat(rho'/options.Zscale,1,options.Nsamples);
      
      h=surf(X,Y,Z);
      set(h,'LineStyle','none','FaceLighting','phong')
      light('Position',[1 0 0],'Style','infinite');
      light('Position',[-1 0 0],'Style','infinite');
    end  
  end
end