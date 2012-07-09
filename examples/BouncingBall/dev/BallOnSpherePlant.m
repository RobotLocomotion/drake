classdef BallOnSpherePlant < HybridDrakeSystem
  % a nice example for Zeno stability from Lamperski and Ames, 2012.  
  
  methods
    function obj = BallOnSpherePlant()
      obj = obj@HybridDrakeSystem(...
        0, ...  % number of inputs
        2);     % number of outputs
      
      % create flight mode system
      sys = BallFlightPhasePlant2D();
      obj = setInputFrame(obj,sys.getInputFrame);
      obj = setOutputFrame(obj,sys.getOutputFrame);
      
      [obj,flight_mode] = addMode(obj,sys);  % add the single mode
      
      g1=inline('norm(x(1:2))-1','obj','t','x','u');  
      g2=inline('x(1:2)''*x(3:4)','obj','t','x','u'); 
      
      g3=inline('x(2)+2','obj','t','x','u');
      
      obj = addTransition(obj, ...
        flight_mode, ...           % from mode
        andGuards(obj,g1,g2), ...  % q-r<=0 & qdot<=0
        @collisionDynamics, ...    % transition method
        false, ...                 % not direct feedthrough
        true);                     % time invariant

      obj = addTransition(obj, ...
        flight_mode, ... 
        g3, ...
        @stopIfFalling, ...
        false, ...
        true);
    end

    function [xn,m,status] = collisionDynamics(obj,m,t,x,u)
      xn = [x(1:2); x(3:4) - (1+obj.e)*(x(1:2)'*x(3:4))*x(1:2)];
      
      if (xn(2)<1e-3) status = 1; % stop simulating if ball has stopped
      else status = 0; end
    end          
    
    function [xn,m,status] = stopIfFalling(obj,m,t,x,u)
      xn=x;
      status=1;
    end
    
  end
  
  properties
    e = .5;  % coefficient of restitution
  end
  
  methods (Static=true)
    function run
      b=BallOnSpherePlant;
      v=BallOnSphereVisualizer(b);

      zeno = false;
      try 
        x=b.simulate([0 5],[1;.190;1.2;0;0]);   % not zeno
      catch ex
        if strcmp(ex.identifier,'Simulink:Engine:SolverConsecutiveZCNum')
          zeno=true;
        else
          rethrow(ex);
        end
      end
      if (zeno) error('hit zeno for initial conditions that should not zeno'); end
      v.playback(x);
      
      zeno = false;
      try
        x=b.simulate([0 5],[1;.191;1.2;0;0]);  % zeno
      catch ex
        if strcmp(ex.identifier,'Simulink:Engine:SolverConsecutiveZCNum')
          zeno=true;
        else
          rethrow(ex);
        end
      end
      if (~zeno) error('should have zeno''d from these initial conditions, but did not'); end
    end
  end
  
end
