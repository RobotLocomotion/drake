classdef GridWorld < DrakeSystem
  % defines a continuous state, action dynamics (discrete time)
  % for the grid world, which can easily be discretized and solved
  % as an MDP
  
  properties
    goal         % x,y position of the goal
    board        % 2x1 vector with number of cells in the grid [x cells; y cells]
  end
  
  methods
    function obj = GridWorld()
      obj = obj@DrakeSystem(0,2,1,2,false,true);
      obj.board = [20;20];
      obj.goal = [2;8];
      obj = setOutputFrame(obj,obj.getStateFrame);
    end
    
    function x0 = getInitialState(obj)
      x0 = [randi(obj.board(1)); randi(obj.board(2))];
    end
    
    function in_obstacle = obstacle(obj,x)
      x = round(x);
      in_obstacle = false;  % no obstacles yet

      in_obstacle = x(1,:)>=6 & x(1,:)<=8 & x(2,:)>=4 & x(2,:)<=7;
    end

    function xn = update(obj,t,x,u)
      u = mod(round(u),5);
      
      xn=round(x);
      xn(2,u==1) = xn(2,u==1)+1;  % up
      xn(1,u==2) = xn(1,u==2)+1;  % right
      xn(2,u==3) = xn(2,u==3)-1;  % down
      xn(1,u==4) = xn(1,u==4)-1;  % left
      
      % implement board limits
      lim = (xn(1,:) > obj.board(1) | xn(1,:) < 1 | xn(2,:) > obj.board(2) | xn(2,:) < 1);
      xn(lim) = x(lim);
    end
    
    function y = output(obj,t,x,~)
      y = x;
    end
    
    function g = mintime_cost(obj,x,u)
      x = round(x);
      if (sum(x==obj.goal)>1)
        g = 0;
      else
        g = 1;
      end
      
      if (obj.obstacle(x))
        g = 10;
      end
    end
    
    function g = quadratic_cost(obj,x,u)
      g = (x-obj.goal)'*eye(2)*(x-obj.goal);
      
      if (obj.obstacle(x))
        g = obj.board'*obj.board;  % make it larger than any quadratic cost could be on this board
      end
    end    
    
    function v = constructVisualizer(obj)
      [X,Y] = ndgrid(1:obj.board(1),1:obj.board(2));
      X = [X(:)';Y(:)'];
      is_obstacle = obj.obstacle(X);
      
      function draw(t,y)
        plot(obj.goal(1),obj.goal(2),'g*',y(1),y(2),'bp','MarkerSize',10,'LineWidth',3);
        for i=1.5:1:obj.board(1)-0.5
          line(repmat(i,2,1),[0.5,obj.board(2)+0.5],'Color',[0 0 0]);
        end
        for i=1.5:1:obj.board(2)-0.5
          line([0.5,obj.board(1)+0.5],repmat(i,2,1),'Color',[0 0 0]);
        end
        plot(X(1,is_obstacle),X(2,is_obstacle),'rx','MarkerSize',20,'LineWidth',4);
        
        axis([0.5 obj.board(1)+0.5 0.5 obj.board(2)+0.5]);
        title(['n = ' num2str(t)])
      end
      
      v = FunctionHandleVisualizer(obj.getOutputFrame,@draw);
    end
  end

  methods (Static=true)
    function drawBoard
      p = GridWorld;
      v = p.constructVisualizer();
      x0 = getInitialState(p)
      y = p.output(0,x0,0);
      v.draw(0,y);
    end
    
    function runValueIteration
      p = GridWorld;
      options.gamma = 1;
%      costfun = @mintime_cost;
      costfun = @quadratic_cost;
      mdp = MarkovDecisionProcess.discretizeSystem(p,costfun,{1:p.board(1),1:p.board(2)},{0:4},options);

      x = 1:p.board(1); y = 1:p.board(2);
      [X,Y] = ndgrid(x,y);
      
      mdp.valueIteration(.01,@drawfun);
      
      function drawfun(J,PI)
        figure(2); clf; hold on;
        imagesc(1:p.board(1),1:p.board(2),reshape(J,p.board(1),p.board(2))');
        colorbar;
        axis equal; axis xy; axis tight;
        
        % draw the policy (note PI is in action index, not the numerical
        % value of the action)
        U = 1*(PI==3) -1*(PI==5);  
        V = 1*(PI==2) -1*(PI==4);
        quiver(X(:),Y(:),U,V,'k');
        
        % pause;  % useful for stepping through the value iteration updates
      end
    end
  end
  
end