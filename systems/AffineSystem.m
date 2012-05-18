classdef AffineSystem < PolynomialSystem
% Implements 
%   xcdot = Ac*x + Bc*u + xcdot0
%   xdn = Ad*x + Bd*u + xdn0
%   y = C*x + D*u + y0

  methods
    function obj = AffineSystem(Ac,Bc,xcdot0,Ad,Bd,xdn0,C,D,y0)
      obj = obj@PolynomialSystem(0,0,0,0,false,[],[],[]);

      num_xc = max([size(Ac,1),size(Bc,1),size(xcdot0,1)]);
      num_xd = max([size(Ad,1),size(Bd,1),size(xdn0,1)]);
      num_u = max([size(Bc,2),size(Bd,2),size(D,2)]);
      num_y = max([size(C,1),size(D,1),size(y0,1)]);
      
      obj = setNumContStates(obj,num_xc);
      obj = setNumDiscStates(obj,num_xd);
      obj = setNumInputs(obj,num_u);
      obj = setNumOutputs(obj,num_y);
      
      if (isempty(Ac)) 
        obj.Ac = sparse(num_xc,num_xc+num_xd);
      else
        sizecheck(Ac,[num_xc,num_xc+num_xd]);
        obj.Ac = Ac;
      end
      if (isempty(Bc)) 
        obj.Bc = sparse(num_xc,num_u);
      else
        sizecheck(Bc,[num_xc,num_u]); 
        obj.Bc = Bc;
      end
      if (isempty(xcdot0)) 
        obj.xcdot0 = sparse(num_xc,1);
      else
        sizecheck(xcdot0,[num_xc,1]); 
        obj.xcdot0 = xcdot0;
      end
      if (isempty(Ad))
        obj.Ad = sparse(num_xd,num_xc+num_xd);
      else
        sizecheck(Ad,[num_xd,num_xc+num_xd]); 
        obj.Ad = Ad;
      end
      if (isempty(Bd)) 
        obj.Bd = sparse(num_xd,num_u);
      else
        sizecheck(Bd,[num_xd,num_u]);
        obj.Bd = Bd;
      end
      if (isempty(xdn0)) 
        obj.xdn0 = sparse(num_xd,1);
      else
        sizecheck(xdn0,[num_xd,1]); 
        obj.xdn0 = xdn0;
      end
      if (isempty(C)) 
        obj.C = sparse(num_y,num_xc+num_xd);
      else
        sizecheck(C,[num_y,num_xc+num_xd]); 
        obj.C = C;
      end
      if (isempty(D)) 
        obj.D = sparse(num_y,num_u);
      else
        sizecheck(D,[num_y,num_u]); 
        obj.D = D;
        obj = setDirectFeedthrough(obj,true);
      end
      if (isempty(y0)) 
        obj.y0 = sparse(num_y,1);
      else
        sizecheck(y0,[num_y,1]); 
        obj.y0 = y0;
      end
      
      obj = pullEmptyPolysFromMethods(obj);
    end

    function [xcdot,df] = dynamics(obj,t,x,u)
      xcdot=obj.Ac*x;
      if (obj.num_u) xcdot=xcdot+obj.Bc*u; end
      if ~isempty(obj.xcdot0) xcdot=xcdot+obj.xcdot0; end
      if (nargout>1)
        df = [0*xcdot, obj.Ac, obj.Bc];
      end
    end
    
    function [xdn,df] = update(obj,t,x,u)
      xdn=obj.Ad*x;
      if (obj.num_u) xdn=xdn+obj.Bd*u; end
      if ~isempty(obj.xdn0) xdn=xdn+obj.xdn0; end
      if (nargout>1)
        df = [0*xdn,obj.Ad,obj.Bd];
      end
    end
    
    function [y,dy] = output(obj,t,x,u)
      y=zeros(obj.num_y,1);
      if (obj.num_x) y=y+obj.C*x; end
      if (obj.num_u) y=y+obj.D*u; end
      if ~isempty(obj.y0) y=y+obj.y0; end
      if (nargout>1)
        dy = [0*y,obj.C,obj.D];
      end
    end
  end
  
  properties
    Ac
    Bc
    xcdot0
    Ad
    Bd
    xdn0
    C
    D
    y0
  end

end

