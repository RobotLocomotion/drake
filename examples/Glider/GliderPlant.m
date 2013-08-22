classdef GliderPlant < DrakeSystem
% Defines the dynamics for the perching glider.  Translated from Rick Cory's code in 
% https://svn.csail.mit.edu/russt/robots/perchingGlider/ 
  
  % state:  
  %  x(1) - x position
  %  x(2) - z position
  %  x(3) - pitch (theta)
  %  x(4) - elevator (phi)
  %  x(5) - x velocity
  %  x(6) - z velocity
  %  x(7) - pitch velocity (thetadot)
  % input:
  %  u(1) - elevator velocity (phidot)

  properties  % took parameters from Rick's "R1 = no dihedral" model
    S_w = 0.0385; % wing + fus + tail.
    S_e = 0.0147;
    l_w = 0;
    l_e = 0.022;
    l_h = 0.27;
    I = 0.003;
    m = 0.082;
    g = 9.81; % m/s^2
    %Air density for 20 degC dry air, at sea level
    rho = 1.204; % kg/m^3
    xd = [0,0,pi/4,0,0,-.5,-0.5]';% the goal
    phi_lo_limit = -0.9473; % lower elevator angle limit
    phi_up_limit = 0.4463; % upper elevator angle limit
%    talon_b = [0 -.043]'; % talon coordinates, about cg in body coords (meters).
  end
  
  methods
    function obj = GliderPlant()
      obj = obj@DrakeSystem(7,0,1,7);
      ulimit = 13; % max servo velocity
      obj = setInputLimits(obj,-ulimit,ulimit);
      obj = setOutputFrame(obj,getStateFrame(obj));
    end
    
    function [xdot,df] = dynamics(obj,t,x,u)
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
      
      xdot = [qdot; u(1); (sum(F,2) + F_grav)/m; sum(M)/I; ];
      
      % todo: enforce elevator constraints
      % phi_lo_limit < x(4) < phi_up_limit
      
      if (nargout>1)
        [df]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function [y,dy] = output(obj,t,x,u)
      y = x;
      if (nargout>1)
        dy=[zeros(obj.num_y,1),eye(obj.num_y),zeros(obj.num_y,obj.num_u)];
      end
    end
    
    function x = getInitialState(obj)
      x = [-3.2 .4 0 0 7.0 0 0]';
    end
    
  end  
  
end
