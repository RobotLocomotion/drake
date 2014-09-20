classdef DoubleIntegrator < LinearSystem
  
  methods
    function obj = DoubleIntegrator
      obj = obj@LinearSystem([0,1;0,0],[0;1],[],[],eye(2),[]);
      obj = setInputLimits(obj,-1,1);
      obj = setOutputFrame(obj,getStateFrame(obj));
    end
    
    function g = mintime_cost(obj,x,u)
      g = ((x(1,:) ~= 0) | (x(2,:) ~= 0));
    end
    
    function g = lqr_cost(obj,x,u)
      Q=eye(2); R = 10;
      g = x'*Q*x + u'*R*u;
    end
    
    function v = constructVisualizer(obj)
      function draw(t,x)
        blockx = [-1, -1, 1, 1, -1];
        blocky = [0, 0.5, 0.5, 0, 0];
        
        % draw the mass
        brickcolor=[.75 .6 .5];
        fill(blockx+repmat(x(1),1,5),blocky,brickcolor);

        faintline=[.6 .8 .65]*1.1;
        plot(min(blockx)+[0 0],[-5 5],'k:','Color',faintline);
        plot(max(blockx)+[0 0],[-5 5],'k:','Color',faintline);

        % draw the ground
        line([-5, 5], [0, 0],'Color',[.3 .5 1],'LineWidth',1);
        axis equal;
        axis([-5 5 -1 2]);
        %grid on
      end
      
      v = FunctionHandleVisualizer(getOutputFrame(obj),@draw);
    end
  end
  
  methods (Static = true)
    function runValueIteration
      p = DoubleIntegrator;
      options.gamma = .9999;  % discount factor
      options.dt = .01;
      
      costfun = @mintime_cost;
%      costfun = @lqr_cost;
      xbins = {[-3:.2:3],[-4:.2:4]};
      mdp = MarkovDecisionProcess.discretizeSystem(p,costfun,xbins,linspace(-1,1,9),options);
      
      function drawfun(J,PI)
        figure(2); clf;
        n1=length(xbins{1});
        n2=length(xbins{2});
        subplot(2,1,1);imagesc(xbins{1},xbins{2},reshape(PI,n1,n2)');
        axis xy;
        xlabel('q');
        ylabel('qdot');
        title('u(x)');
        subplot(2,1,2);imagesc(xbins{1},xbins{2},reshape(J,n1,n2)');
        axis xy;
        xlabel('q');
        ylabel('qdot');
        title('J(x)');
        drawnow;
        
%        pause;
      end
      valueIteration(mdp,0.0001,@drawfun);
      
      % add colorbars (it's expensive to do it inside the loop)
      figure(2); 
      subplot(2,1,1); colorbar;
      subplot(2,1,2); colorbar;
    end
    
  end
  
end