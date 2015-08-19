classdef StateMachineControl < HybridDrakeSystem
% This controller is my reconstruction of Raibert's 3 part
% controller.  A few gaps have been filled in, but I believe that
% it is very close to the original. -- Russ
  
  properties
    k_fp = 153.0;  b_fp = 14.0;   % foot placement
    k_att = 153.0; b_att = 14.0;  % attitude
    k_xdot = 0.01;                % forward speed
    thrust = 0.035;                % monotonically related to hopping height
    retract = -0.1;               % retracts leg during stance phase

    r_s0 = 1.0;      % rest length of the leg spring (todo: should match dynamics)
    l_1 = 0.5;       % distance from the foot to the com of the leg
    l_2 = 0.4;       % distance from the hip to the com of the body
  end
  
  methods
    function obj = StateMachineControl(plant,desired_speed)
      typecheck(plant,'HopperPlant');
      obj = obj@HybridDrakeSystem(plant.getNumOutputs,plant.getNumInputs);
      obj = setInputFrame(obj,plant.getOutputFrame);
      obj = setOutputFrame(obj,plant.getInputFrame);
      
      if (nargin<2) desired_speed = .5; end
      T_s = .425;
      
      %% add individual control modes
      function u2=bodyAttitudeControl(~,~,x)
        u2 = -obj.k_att*(x(4)-x(3)/2) - obj.b_att*x(9);
      end
      function u=flightControl(~,~,x)
        % Control foot placement
        x_dot = x(6) + x(10)*sin(x(3)) + x(5)*x(8)*cos(x(3)) + x(9)*obj.l_2*cos(x(4));
        theta_d = -asin((x_dot*T_s/2 + obj.k_xdot*(x_dot - desired_speed))/x(5));
        u(2) = obj.k_fp*(x(3)-theta_d) + obj.b_fp*(x(8));
        u(1) = obj.retract;
      end
      
      c = FunctionHandleSystem(0,0,10,2,true,true,[],[],@(t,x,u) flightControl(t,x,u));
      c = setInputFrame(c,obj.getInputFrame); c = setOutputFrame(c,obj.getOutputFrame);
      [obj,FLIGHT] = addMode(obj,c,'FLIGHT');

      c0 = ConstOrPassthroughSystem([0;0],10);
      c0 = setInputFrame(c0,obj.getInputFrame); c0 = setOutputFrame(c0,obj.getOutputFrame);
      [obj,LOADING] = addMode(obj,c0,'LOADING');
      
      c = FunctionHandleSystem(0,0,10,2,true,true,[],[],@(t,x,u) [0; bodyAttitudeControl(t,x,u)]); 
      c = setInputFrame(c,obj.getInputFrame); c = setOutputFrame(c,obj.getOutputFrame);
      [obj,COMPRESSION] = addMode(obj,c,'COMPRESSION');
      
      c = FunctionHandleSystem(0,0,10,2,true,true,[],[],@(t,x,u) [obj.thrust; bodyAttitudeControl(t,x,u)]);
      c = setInputFrame(c,obj.getInputFrame); c = setOutputFrame(c,obj.getOutputFrame);
      [obj,THRUST] = addMode(obj,c,'THRUST');
      
      [obj,UNLOADING] = addMode(obj,c0,'UNLOADING');
      [obj,ESCAPE] = addMode(obj,c0,'ESCAPE');


      %% add transitions
      function [mode_xn,to_mode_num,status]=simpleTransition(obj,~,~,~,x,to_mode)
        mode_xn = x;
        to_mode_num = to_mode;
        status = 0;
      end
      
%      obj = addTransition(obj,LOADING,@(obj,t,~,x) x(10), ...
%        @(obj,m,t,x,u)simpleTransition(obj,m,t,x,u,COMPRESSION), ...
%        false,true,COMPRESSION);
      
      obj = addTransition(obj,COMPRESSION,@(obj,t,~,x) -x(10), ...
        @(obj,m,t,x,u)simpleTransition(obj,m,t,x,u,THRUST), ...
        false,true,THRUST);
      
      obj = addTransition(obj,THRUST,@(obj,t,~,x) obj.r_s0 - x(5), ...
        @(obj,m,t,x,u)simpleTransition(obj,m,t,x,u,UNLOADING), ...
        false,true,UNLOADING);
      
% todo: handle T_s
% todo: add ESCAPE back in (requires a state remembering the time)
      
%      obj = addTransition(obj,UNLOADING,@(obj,t,~,x) -x(2), ...
%        @(obj,m,t,x,u)simpleTransition(obj,m,t,x,u,ESCAPE), ...
%        false,true,ESCAPE);

%      obj = addTransition(obj,ESCAPE,@(obj,t,~,x) -x(2), ...
%        @(obj,m,t,x,u)simpleTransition(obj,m,t,x,u,FLIGHT), ...
%        false,true,FLIGHT);
      
      obj = addTransition(obj,UNLOADING,@(obj,t,~,x) -x(2), ...
        @(obj,m,t,x,u)simpleTransition(obj,m,t,x,u,FLIGHT), ...
        false,true,FLIGHT);
      
      obj = addTransition(obj,FLIGHT,@(obj,t,~,x) x(2), ...
        @(obj,m,t,x,u)simpleTransition(obj,m,t,x,u,COMPRESSION), ...
        false,true,COMPRESSION);
    end
  end
  
  methods (Static=true)
    function run
      h=HopperPlant();
      c=StateMachineControl(h);
      v=HopperVisualizer(h);
%      sys = cascade(feedback(h,c),v);
%      simulate(sys,[0 inf]);
      y=simulate(feedback(h,c),[0 3]);
      playback(v,y);
    end
  end
  
end
