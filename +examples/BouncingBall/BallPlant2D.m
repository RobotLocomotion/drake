classdef BallPlant2D < HybridDrakeSystem
  
  methods
    function obj = BallPlant2D()
      obj = obj@HybridDrakeSystem(...
        0, ...  % number of inputs
        2);     % number of outputs
      
      % create flight mode system
      sys = BallFlightPhasePlant2D();
      obj = setInputFrame(obj,sys.getInputFrame);
      obj = setOutputFrame(obj,sys.getOutputFrame);
      [obj,flight_mode] = addMode(obj,sys);  % add the single mode
      
      g1=inline('x(2)-obj.r','obj','t','x','u');  % q-r<=0
      g2=inline('x(4)','obj','t','x','u'); % qdot<=0
      obj = addTransition(obj, ...
        flight_mode, ...           % from mode
        andGuards(obj,g1,g2), ...  % q-r<=0 & qdot<=0
        @collisionDynamics, ...    % transition method
        false, ...                 % not direct feedthrough
        true);                     % time invariant
    end

    function [xn,m,status] = collisionDynamics(obj,m,t,x,u)
      xn = [x(1:3); -obj.cor*x(4)];      % qdot = -cor*qdot
      
      if (xn(4)<0.01) status = 1; % stop simulating if ball has stopped
      else status = 0; end
    end          
    
  end
  
  properties
    r = 1;  % radius of the ball
    cor = .8;  % coefficient of restitution
  end
  
  methods (Static=true)
    function run
      b=BallPlant2D;
      v=BallVisualizer2D(b);
      x=b.simulate([0 5],[1;-5;5;2;0]);
      v.playback(x);
    end
  end
end
