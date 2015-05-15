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
      q = x(1:3);
      qd = x(4:6);
      x_b = q(1);
      y_b = q(2);
      theta = q(3);   
      
      % z = [lxp;lxm;gamma]
      % y = [qn;qdn;lz];
      %A*y = B*z + c;
      A = zeros(7);
      B = zeros(7,3);
      c = zeros(7,1);
      
      %qn = q + h*qdn
      A(1:3,4:6) = -obj.h*eye(3);
      A(1:3,1:3) = eye(3);
      c(1:3) = q;
      
      %qdn = qd + h*dynamics + J'*lambda
      I = .005;
      H = diag([1;1;I]);
      
      A(6,3) = obj.h*obj.k;
      A(6,6) = obj.h*obj.b;
      c(5) = -obj.h*obj.g;
            
      A(4:6,4:6) = A(4:6,4:6) + H;                
      c(4:6) = c(4:6) + H*qd;
      
      
      % l + h*ldot = L
      [l,dl] = getLength(obj,q);
      A(7,4:6) = obj.h*dl;
      c(7) = obj.L - l;         
      
      
      vec_1 = [-x_b;-y_b];
      vec_1 = vec_1/norm(vec_1);
      
      vec_2 = [1-x_b;-y_b];
      vec_2 = vec_2/norm(vec_2);
      
      % get angle
      wrap_angle = acos(vec_1'*vec_2)/2;
      
      %force basis
      basis = [(vec_1 + vec_2)'/2; (vec_1(2)+vec_2(2))/2, -(vec_1(1)+vec_2(1))/2];
      basis(1,:) = basis(1,:)/norm(basis(1,:));
      basis(2,:) = basis(2,:)/norm(basis(2,:));
      A(4:6,7) = [basis(1,:)'*cos(wrap_angle)/wrap_angle; -obj.R*1/(2*wrap_angle)];
      
      B(4:6,1) = [-basis(1,:)'*cos(wrap_angle) - basis(2,:)'*sin(wrap_angle); 0];
      B(4:6,2) = -B(4:6,1);
      
      invAB = A\B;
      invAc = A\c;
      
      if invAc(7) < 0
        invAc(7) = -invAc(7);
        invAB(7,:) = -invAB(7,:);
      end
      
      % complementarity constraints
      M = zeros(3);
      w = zeros(3,1);
      
      % psi + gamma \perp lxp
      M(1,:) = -sin(wrap_angle)/wrap_angle*basis(2,:)*invAB(4:5,:);
      M(1,3) = M(1,3) +1;
      M(2,1:2) = -M(1,1:2);
      M(2,3) = M(2,3) + 1;
      w(1) = -sin(wrap_angle)/wrap_angle*basis(2,:)*invAc(4:5,:);
      w(2) = -w(1);
      % psi - gamma \perp lxm
      
      % mu*lz - lxp - lxm \perp gamma
      mu = 1;
      M(3,:) = [-1 -1 0] + mu*invAB(7,:);
      w(3) = mu*invAc(7);
      
      sol = pathlcp(M,w);
        
      y = invAB*sol + invAc;
      xn = y(1:6)
    end
    
    function [n,D] = cableJacobians(obj,q)
      x_b = q(1);
      y_b = q(2);
      theta = q(3);
      
      
      
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
      N = 1e3;
      x0 = [-.4;sqrt(.6^2-.5^2);0;0;0;0];
      [l,dl] = obj.getLength(x0(1:3));
      x0(3) = -(l-obj.L)/obj.R;
      
      x = zeros(6,N);
      x(:,1) = x0;
      for k=2:N,
        x(:,k) = obj.updateLCP(0,x(:,k-1),[]);
      end
%       for k=2:N,
%         x(:,k) = obj.update(0,x(:,k-1),[]);
%       end
    end
    
  end
  
  
  
end