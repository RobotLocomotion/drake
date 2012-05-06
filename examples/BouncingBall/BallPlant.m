classdef BallPlant < HybridRobotLibSystem
  
  methods
    function obj = BallPlant()
      sys = BallFlightPhasePlant(); % create flight mode system
      [obj,flight_mode] = addMode(obj,sys);  % add the single mode
      
      g1=inline('x(1)-obj.r','obj','t','x','u');  % q-r<=0
      g2=inline('x(2)','obj','t','x','u'); % qdot<=0
      obj = addTransition(obj, ...
        flight_mode, ...           % from mode
        andGuards(obj,g1,g2), ...  % q-r<=0 & qdot<=0
        @collisionDynamics, ...    % transition method
        false, ...                 % not direct feedthrough
        true);                     % time invariant

      obj=setModeOutputFlag(obj,false); % don't attach mode to the output
    end

    function [xn,m,status] = collisionDynamics(obj,m,t,x,u)
      xn = [x(1); -obj.cor*x(2)];      % qdot = -cor*qdot
      
      if (xn(2)<0.01) status = 1; % stop simulating if ball has stopped
      else status = 0; end
    end          
    
  end
  
  properties
    r = 1;  % radius of the ball
    cor = .8;  % coefficient of restitution
  end
end
