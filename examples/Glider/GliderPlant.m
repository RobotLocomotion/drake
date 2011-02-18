classdef GliderPlant < RobotLibSystem
% Defines the dynamics for the perching glider.  Translated from Rick Cory's code in 
% https://svn.csail.mit.edu/russt/robots/perchingGlider/ 
  
  properties  % took parameters from Rick's "R1 = no dihedral" model
    S_w = 0.0385; % wing + fus + tail.
    S_e = 0.0147;
    l_w = 0;
    l_e = 0.022;
    l_h = 0.27;
    I = 0.003;
    m = 0.082;
    g = 9.81; % m/s^2
    rho = 1.292; % kg/m^3
%    xd = [0,0,pi/4,0,0,-.5,-0.5]'% the goal
    phi_lo_limit = -0.9473; % lower elevator angle limit
    phi_up_limit = 0.4463; % upper elevator angle limit
%    talon_b = [0 -.043]'; % talon coordinates, about cg in body coords (meters).
  end
  
  methods
    function obj = GliderPlant()
      obj = obj@RobotLibSystem(7,0,1,7);
      ulimit = 13; % max servo velocity
      obj = setInputLimits(obj,-ulimit,ulimit);
    end
    
    function xdot = dynamics(obj,t,x,u)
      q=x(1:4); qdot=x(5:7); 
      m=obj.m; g=obj.g; rho=obj.rho; S_w=obj.S_w;
      S_e=obj.S_e; I=obj.I; l_h=obj.l_h; l_w=obj.l_w; l_e=obj.l_e;

      V_w = [qdot(1)-l_w*sin(q(3))*qdot(3); qdot(2)+l_w*cos(q(3))*qdot(3)]; % wing velocity
      V_e = [qdot(1)+l_h*sin(q(3))*qdot(3)+l_e*sin(q(4)+q(3))*(u(1)+qdot(3));
        qdot(2)-l_h*cos(q(3))*qdot(3)-l_e*cos(q(4)+q(3))*(u(1)+qdot(3))]; % elev. velocity
      V = [V_w V_e];
      
      alpha = q(3) - atan(V(2,:)./V(1,:)); % aoa
      alpha(2) = alpha(2) + q(4); % add elv. angle component
      
      V_norm = sqrt(dot(V,V,1)); % velocity norm
      n_L = [-V(2,:); V(1,:)]./repmat(V_norm,2,1); % unit Lift vector
      n_D = -V./repmat(V_norm,2,1); % unit Drag vector
      
      S = [S_w S_e]; % surface areas
      Cl = 2*sin(alpha).*cos(alpha); Cd = 2*sin(alpha).^2;
      L = repmat(0.5*rho*V_norm.^2.*S.*Cl,2,1).*n_L;
      D = repmat(0.5*rho*V_norm.^2.*S.*Cd,2,1).*n_D;
      F = L + D; % total aerodynamic force
      F_grav = [0;-m*g]; % gravity
      
      r_w = [l_w*cos(q(3)); % wing moment arm
        l_w*sin(q(3))];
      r_e = [-l_h*cos(q(3))-l_e*cos(q(4)+q(3)); % elevator moment arm
        -l_h*sin(q(3))-l_e*sin(q(4)+q(3))];
      r = [r_w r_e]; %  combined moment vector
      M = r(1,:).*F(2,:) - r(2,:).*F(1,:); % moments
      
      xdot(1:3,1) = qdot;
      xdot(4,1) = u(1);
      xdot(5:6,1) = (sum(F,2) + F_grav)/m;
      xdot(7,:) = sum(M)/I;
      
      % todo: enforce elevator constraints
      % phi_lo_limit < x(4) < phi_up_limit
    end
    
    function df = dynamicsGradients(obj,t,x,u,order)
      % Implement the Taylor expansion of the second-order gradients
      if (nargin>4 && order>1) df = dynamicsGradients@Plant(obj,t,x,u,order); return; end

      df = glider_grads(obj,t,x,u);
    end
    
    function y = output(obj,t,x,u)
      y = x;
    end
    
    function dy = outputGradients(obj,t,x,u,order)
      error('implement this (it''s trivial)'); 
    end
    
    function x = getInitialState(obj)
      x = [-3.5 .1 0 0 7.0 0 0]';
    end
    
  end  
  
end