classdef Test
  properties
    h = 1e-3;
    k = 1;
    b = 0.01;
    g = 10;
    L = 1.2;
    R = .1;
  end
  
  methods
    
    function xn = update(obj,t,x,u)
      q = x(1:3);
      qd = x(4:6);
      
      A = zeros(7);
      b = zeros(6,1);
      
      %qn = q + h*qdn
      A(1:3,4:6) = obj.h*eye(3);
      b(1:3) = -q;
      
      %qdn = qd + h*dynamics + J'*lambda
      I = .005;
      H = diag([1;1;I]);
      
      [l,dl] = getLength(obj,q);
      A(6,3) = -obj.h*obj.k/I;
      A(6,6) = -obj.h*obj.b/I;
      b(5) = obj.h*obj.g;
      A(4:6,7) = H\(obj.h*dl');
      
      A(1:6,1:6) = A(1:6,1:6) - eye(6);
      
      b(4:6) = b(4:6) - qd;
      
      % l + h*ldot = L
      A(7,4:6) = obj.h*dl;
      b(7) = obj.L - l;
      
      z = A\b;
      xn = z(1:6);
    end
    
    function xn = updateLCP(obj,t,x,u)
      % z = [lxp1;lxm1;lxp2;lxm2;gamma1;gamma2]
      
      %A*[qn;qdn;lz1;lz2] = B*z + b;
      A = zeros(8);
      B = zeros(8,6);
      b = zeros(8,1);
      
      %qn = q + h*qdn
      A(1:3,4:6) = obj.h*eye(3);
      b(1:3) = -q;
      
      %qdn = qd + h*dynamics + J'*lambda
      I = .005;
      H = diag([1;1;I]);
      
      [n,D] = cableJacobians(obj,q);
      A(6,3) = -obj.h*obj.k;
      A(6,6) = -obj.h*obj.b;
      b(5) = obj.h*obj.g;
      
      A(4:6,7:8) = obj.h*n';
      
      A(1:3,1:3) = A(1:3,1:3) - eye(3);
      A(4:6,4:6) = A(4:6,4:6) - H;          
      
      b(4:6) = b(4:6) - qd;
      
      B(4:6,1:2) = obj.h*D';
      B(4:6,3:4) = -obj.h*D';
      
      % l + h*ldot = L
      [l,dl] = getLength(obj,q);
      A(7,4:6) = obj.h*dl;
      b(7) = obj.L - l;
      
    end
    
    function [n,D] = cableJacobians(obj,q)
      x_b = q(1);
      y_b = q(2);
      theta = q(3);
      
      n = [-y_b x_b 0;
           y_b (1-x_b) 0];
      D = [x_b/sqrt(x_b^2 + y_b^2) y_b/sqrt(x_b^2 + y_b^2) 0;
           (1-x_b)/sqrt((1-x_b)^2 + y_b^2) -y_b/sqrt((1-x_b)^2 + y_b^2) obj.R]l
    end
    
    function [l,dl] = getLength(obj,q)
      x_b = q(1);
      y_b = q(2);
      theta = q(3);
      
      l = sqrt(x_b^2 + y_b^2) + sqrt((1-x_b)^2 + y_b^2) + obj.R*theta;
      
      dl = [x_b/sqrt(x_b^2 + y_b^2)  - (1-x_b)/sqrt((1-x_b)^2 + y_b^2);
         y_b/sqrt(x_b^2 + y_b^2) + y_b/sqrt((1-x_b)^2 + y_b^2);
         obj.R]';
    end
    
    function [x,N] = test(obj)
      N = 1e4;
      x0 = [.4;sqrt(.6^2-.5^2);0;0;0;0];
      [l,dl] = obj.getLength(x0(1:3));
      x0(3) = -(l-obj.L)/obj.R;
      
      x = zeros(6,N);
      x(:,1) = x0;
      for k=2:N,
        x(:,k) = obj.update(0,x(:,k-1),[]);
      end
    end
    
  end
  
  
  
end