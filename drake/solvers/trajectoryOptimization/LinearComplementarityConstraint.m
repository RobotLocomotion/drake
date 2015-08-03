classdef LinearComplementarityConstraint < CompositeConstraint
  % LinearComplementarityConstraint
  % A constraint of the form z >= 0, Wz + Mx + q >= 0, <z,Wz + q> = 0
  %  for given W,q
  % Constraints are applied to the stacked vector [x;z;gamma]
  %   wherever there are slack variables gamma
  %
  % mode 1: (default)
  %         z >= 0 (bb),
  %         W*z + M*x + q >= 0 (lin),
  %         <z,W*z+M*x+q)> = 0 (nl) (elementwise)
  %
  % mode 3: (Fischer-Burmeister)
  %         z + W*z+M*x+q - sqrt(z^2 + (W*z+M*x+q)^2) (nl) (elementwise)
  methods
    
    function obj = LinearComplementarityConstraint(W,q,M,mode,slack)
      zdim = size(W,2);
      
      if nargin < 3
        M = zeros(zdim,0);
      end
      
      if nargin < 4
        mode = 1;
      end
      
      if nargin < 5
        slack = 0;
      end
      
      xdim = size(M,2);
      
      assert(zdim == size(W,1));
      assert(zdim == size(M,1));
      
      constraints = {};
      n = 0;
      switch mode
        case 1
          bcon = BoundingBoxConstraint([-inf(xdim,1);zeros(zdim,1)],inf(zdim+xdim,1));
          lincon = LinearConstraint(-q,inf(zdim,1),[M W]);
          nlcon = FunctionHandleConstraint(zeros(zdim,1),zeros(zdim,1)+slack,xdim+zdim,@prodfun);
          constraints = {bcon;lincon;nlcon};
        case 3
          constraints = FunctionHandleConstraint(zeros(zdim,1),zeros(zdim,1),zdim,@fbfun);
      end
      function [f,df] = prodfun(y)
        x = y(1:xdim);
        z = y(xdim+1:end);
        
        g = W*z + M*x + q;
        dg = [M W];
        
        f = z.*g;
        df = diag(z)*dg + [zeros(zdim,xdim) diag(g)];
      end

      function [f,df] = fbfun(y)
        x = y(1:xdim);
        z = y(xdim+1:end);
        
        g = W*z + M*x + q;
        dg = [M W];
        
        f = z + g  - sqrt(z.^2 + g.^2);
        df = [zeros(zdim,xdim) eye(zdim)] + dg - diag(1./sqrt(z.^2 + g.^2 + 1e-6)) * ([zeros(zdim,xdim) diag(z)] + diag(g)*dg);
      end
      
      obj = obj@CompositeConstraint(constraints, n);
    end
  end
end