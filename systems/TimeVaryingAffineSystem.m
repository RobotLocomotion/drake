classdef TimeVaryingAffineSystem < TimeVaryingPolynomialSystem
% Implements 
%   xcdot = Ac(t)*x + Bc(t)*u + xcdot0(t)
%   xdn = Ad(t)*x + Bd(t)*u + xdn0(t)
%   y = C(t)*x + D(t)*u + y0(t)

  methods
    function obj = TimeVaryingAffineSystem(Ac,Bc,xcdot0,Ad,Bd,xdn0,C,D,y0)
      obj = obj@TimeVaryingPolynomialSystem(0,0,0,0,false);

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
        if isnumeric(Ac) Ac = ConstantTrajectory(Ac); end
        typecheck(Ac,'Trajectory');
        sizecheck(Ac,[num_xc,num_xc+num_xd]);
        obj.Ac = Ac;
      end
      if (isempty(Bc)) 
        obj.Bc = sparse(num_xc,num_u);
      else
        if isnumeric(Bc) Bc = ConstantTrajectory(Bc); end
        typecheck(Bc,'Trajectory');
        sizecheck(Bc,[num_xc,num_u]); 
        obj.Bc = Bc;
      end
      if (isempty(xcdot0)) 
        obj.xcdot0 = sparse(num_xc,1);
      else
        if isnumeric(xdot0) xcdot0 = ConstantTrajectory(xcdot0); end
        typecheck(xcdot0,'Trajectory');
        sizecheck(xcdot0,[num_xc,1]); 
        obj.xcdot0 = xcdot0;
      end
      if (isempty(Ad))
        obj.Ad = sparse(num_xd,num_xc+num_xd);
      else
        if isnumeric(Ad) Ad = ConstantTrajectory(Ad); end
        typecheck(Ad,'Trajectory');
        sizecheck(Ad,[num_xd,num_xc+num_xd]); 
        obj.Ad = Ad;
      end
      if (isempty(Bd)) 
        obj.Bd = sparse(num_xd,num_u);
      else
        if isnumeric(Bd) Bd = ConstantTrajectory(Bd); end
        typecheck(Bd,'Trajectory');
        sizecheck(Bd,[num_xd,num_u]);
        obj.Bd = Bd;
      end
      if (isempty(xdn0)) 
        obj.xdn0 = sparse(num_xd,1);
      else
        if isnumeric(xdn0) xdn0 = ConstantTrajectory(xdn0); end
        typecheck(xdn0,'Trajectory');
        sizecheck(xdn0,[num_xd,1]); 
        obj.xdn0 = xdn0;
      end
      if (isempty(C)) 
        obj.C = sparse(num_y,num_xc+num_xd);
      else
        if isnumeric(C) C = ConstantTrajectory(C); end
        typecheck(C,'Trajectory');
        sizecheck(C,[num_y,num_xc+num_xd]); 
        obj.C = C;
      end
      if (isempty(D)) 
        obj.D = sparse(num_y,num_u);
        obj = setDirectFeedthrough(obj,false);
      else
        if isnumeric(D) D = ConstantTrajectory(D); end
        typecheck(D,'Trajectory');
        sizecheck(D,[num_y,num_u]); 
        obj.D = D;
        obj = setDirectFeedthrough(obj,true);
      end
      if (isempty(y0)) 
        obj.y0 = sparse(num_y,1);
      else
        if isnumeric(y0) y0 = ConstantTrajectory(y0); end
        typecheck(y0,'Trajectory');
        sizecheck(y0,[num_y,1]); 
        obj.y0 = y0;
      end
    end

    function xcdot = dynamics(obj,t,x,u)
      xcdot=obj.Ac.eval(t)*x;
      if (obj.num_u) xcdot=xcdot+obj.Bc.eval(t)*u; end
      if ~isempty(obj.xcdot0) xcdot=xcdot+obj.xcdot0.eval(t); end
    end
    
    function xdn = update(obj,t,x,u)
      xdn=obj.Ad.eval(t)*x;
      if (obj.num_u) xdn=xdn+obj.Bd.eval(t)*u; end
      if ~isempty(obj.xdn0) xdn=xdn+obj.xdn0.eval(t); end
    end
    
    function y = output(obj,t,x,u)
      y=zeros(obj.num_y,1);
      if (obj.num_x) y=y+obj.C.eval(t)*x; end
      if (obj.num_u) y=y+obj.D.eval(t)*u; end
      if ~isempty(obj.y0) y=y+obj.y0.eval(t); end
    end

    function sys = extractLTVSystem(obj)
      if any([obj.xcdot0;obj.xdn0;obj.y0]~=0) 
        warning('Drake:AffineSystem:extractLTVSystem','extract linear terms but affine terms are not all zero');
      end
      
      sys = TimeVaryingLinearSystem(obj.Ac,obj.Bc,obj.Ad,obj.Bd,obj.C,obj.D);

      sys = setInputFrame(sys,obj.getInputFrame());
      sys = setStateFrame(sys,obj.getStateFrame());
      sys = setOutputFrame(sys,obj.getOutputFrame());
      
      sys = setSampleTime(sys,obj.getSampleTime);
    end
  end
  
  % todo: implement feedback and cascade
  
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

