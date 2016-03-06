classdef PlanePlantOldModel < SmoothRobotLibSystem
% Defines the dynamics for the powered plane.
  
  properties
    m = 0.082;
    g = 9.81; % m/s^2
  end
  
  methods
    function obj = PlanePlantOldModel()
      obj = obj@SmoothRobotLibSystem(4,0,1,4);
      %ulimit = 13; % max servo velocity
      %obj = setInputLimits(obj,-ulimit,ulimit);
    end
    
    function [xdot df,d2f,d3f] = dynamics(obj,t,x,u)
        % x = [ xpos ypos yaw roll ]
        
        % from Michael's plane dynamics
        max_roll = pi/6;
        R_turn = 1; % minimum turning radius (at max roll), = m/c1/sinroll =2*m/c1
        v_nom = sqrt(R_turn*sin(max_roll)*9.8); %for horizontal flight, =mg/c1
        roll_tau = 0.3;
        
        roll_0 = u; % input is the roll rate

        v = v_nom * sqrt( sec(x(4)) );
        dyaw = v * sin(x(4)) / sin(max_roll)/R_turn;

        xdot = [-v*sin(x(3)); v*cos(x(3)); dyaw; -1/roll_tau*(x(4)-roll_0)];
        
        if (nargout>1)
            [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
        end
        
    end
    
    function [y,dy] = output(obj,t,x,u)
      y = x;
      
      if (nargout>1)
        %dy=[zeros(obj.num_y,1),eye(obj.num_y),zeros(obj.num_y,obj.num_u)];
        dy = 0;
        disp('Warning: output dy not implemented yet.')
      end
    end
    
    function x = getInitialState(obj)
      x = [0 0 0 0]';
    end
    
  end  
  
end
