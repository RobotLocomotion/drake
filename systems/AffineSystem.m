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
    
    function sys = feedback(sys1,sys2)
      % try to keep feedback between polynomial systems polynomial.  else,
      % kick out to DrakeSystem
      %
      
      [sys1,sys2] = matchCoordinateFramesForCombination(sys1,sys2,false);
      [sys2,sys1] = matchCoordinateFramesForCombination(sys2,sys1,true);

      if ~isa(sys2,'AffineSystem') || any(~isinf([sys1.umin;sys1.umax;sys2.umin;sys2.umax]))
        sys = feedback@PolynomailSystem(sys1,sys2)
      end
        
      if (sys1.isDirectFeedthrough() && sys2.isDirectFeedthrough())
        error('Drake:AffineSystem:AlgebraicLoop','algebraic loop');
      end
      
      if (getNumZeroCrossings(sys1)>0 || getNumZeroCrossings(sys2)>0)
        error('affine systems aren''t supposed to have zero crossings'); 
      end
      if (getNumStateConstraints(sys1)>0 || getNumStateConstraints(sys2)>0)
        error('affine systems aren''t supposed to have state constraints'); 
      end
      
      %x1dot = (A1+B1D2C1)x1 + B1C2x2 + B1u + (B1D2y01 + B1y02 + xdot01)
      %x2dot = B2C1x1 + (A2+B2D1C2)x2 + B2D1u + (B2D1y02 + B2y01 + xdot02)
      %y = C1x1 + D1C2x2 + D1u + (D1y02 + y01)
      
      [sys1ind,sys2ind] = stateIndicesForCombination(sys1,sys2);

      Ac(:,sys1ind) = [sys1.Ac + sys1.Bc*sys2.D*sys1.C; sys2.Bc*sys1.C];
      Ac(:,sys2ind) = [sys1.Bc*sys2.C; sys2.Ac + sys2.Bc*sys1.D*sys2.C];
      Bc = [sys1.Bc; sys2.Bc*sys1.D];
      xcdot0 = [sys1.Bc*sys2.D*sys1.y0 + sys1.Bc*sys2.y0 + sys1.xcdot0; sys2.Bc*sys1.D*sys2.y0 + sys2.Bc*sys1.y0 + sys2.xcdot0];
      
      Ad(:,sys1ind) = [sys1.Ad + sys1.Bd*sys2.D*sys1.C; sys2.Bd*sys1.C];
      Ad(:,sys2ind) = [sys1.Bd*sys2.C; sys2.Ad + sys2.Bd*sys1.D*sys2.C];
      Bd = [sys1.Bd; sys2.Bd*sys1.D];
      xdn0 = [sys1.Bd*sys2.D*sys1.y0 + sys1.Bd*sys2.y0 + sys1.xdn0; sys2.Bd*sys1.D*sys2.y0 + sys2.Bd*sys1.y0 + sys2.xdn0];
      
      C(:,sys1ind) = sys1.C;
      C(:,sys2ind) = sys1.D*sys2.C;
      D = sys1.D;
      y0 = sys1.D*sys2.y0 + sys1.y0;
      
      sys = AffineSystem(Ac,Bc,xcdot0,Ad,Bd,xdn0,C,D,y0);
      
      sys = setInputFrame(sys,sys1.getInputFrame());
      sys = setOutputFrame(sys,sys1.getOutputFrame());
      if (sys1.getNumStates==0) 
        sys = setStateFrame(sys,sys2.getStateFrame());
      elseif (sys2.getNumStates==0)
        sys = setStateFrame(sys,sys1.getStateFrame());
      % otherwise keep the new state frame generated by the drakesystem constructor
      end

      try 
        sys = setSampleTime(sys,[sys1.getSampleTime(),sys2.getSampleTime()]);  % todo: if this errors, then kick out to drakesystem?
      catch ex
        if (strcmp(ex.identifier, 'Drake:DrakeSystem:UnsupportedSampleTime'))
          warning('Drake:AffineSystem:UnsupportedSampleTime','Aborting affine feedback because of incompatible sample times'); 
          sys = feedback@PolynomialSystem(sys1,sys2);
        else
          rethrow(ex)
        end
      end
    end
    
    function sys = cascade(sys1,sys2)
      % try to keep cascade between polynomial systems polynomial.  else,
      % kick out to DrakeSystem
      %

      [sys1,sys2] = matchCoordinateFramesForCombination(sys1,sys2);

      if ~isa(sys2,'PolynomialSystem') || any(~isinf([sys2.umin;sys2.umax]))
        sys = cascade@DrakeSystem(sys1,sys2)
      end
        
      if (getNumZeroCrossings(sys1)>0 || getNumZeroCrossings(sys2)>0)
        error('polynomialsystems aren''t supposed to have zero crossings'); 
      end
      
      % x1dot = A1x1 + B1u + xcdot01
      % x2dot = A2x2 + B2(C1x1+D1u+y01) + xcdot02
      % y = C2x2 + D2(C1x1+D1u + y01) + y02
      
      [sys1ind,sys2ind] = stateIndicesForCombination(sys1,sys2);

      Ac(:,sys1ind) = [sys1.Ac; sys2.Bc*sys1.C]; 
      Ac(:,sys2ind) = [zeros(getNumContStates(sys1),getNumStates(sys2)); sys2.Ac];
      Bc = [sys1.Bc; sys2.Bc*sys1.D];
      xcdot0 = [sys1.xcdot0; sys2.Bc*sys1.y0 + sys2.xcdot0];
      
      Ad(:,sys1ind) = [sys1.Ad; sys2.Bd*sys1.C]; 
      Ad(:,sys2ind) = [zeros(getNumDiscStates(sys1),getNumStates(sys2)); sys2.Ad];
      Bd = [sys1.Bd; sys2.Bd*sys1.D];
      xdn0 = [sys1.xdn0; sys2.Bd*sys1.y0 + sys2.xdn0];

      C(:,sys1ind) = sys2.D*sys1.C;
      C(:,sys2ind) = sys2.C;
      D = sys2.D*sys1.D;
      y0 = sys2.D*sys1.y0 + sys2.y0;
      
      sys = AffineSystem(Ac,Bc,xcdot0,Ad,Bd,xdn0,C,D,y0);

      sys = setInputFrame(sys,sys1.getInputFrame());
      sys = setOutputFrame(sys,sys2.getOutputFrame());
      
      try 
        sys = setSampleTime(sys,[sys1.getSampleTime(),sys2.getSampleTime()]);  % todo: if this errors, then kick out to drakesystem?
      catch ex
        if (strcmp(ex.identifier, 'Drake:DrakeSystem:UnsupportedSampleTime'))
          warning('Drake:PolynomialSystem:UnsupportedSampleTime','Aborting polynomial cascade because of incompatible sample times'); 
          sys = cascade@DrakeSystem(sys1,sys2);
        else
          rethrow(ex)
        end
      end
    end
    
    function sys = extractLTISystem(obj)
      if any([obj.xcdot0;obj.xdn0;obj.y0]~=0) 
        warning('Drake:AffineSystem:extractLTISystem','extract linear terms but affine terms are not all zero');
      end
      
      sys = LTISystem(obj.Ac,obj.Bc,obj.Ad,obj.Bd,obj.C,obj.D);

      sys = setInputFrame(sys,obj.getInputFrame());
      sys = setStateFrame(sys,obj.getStateFrame());
      sys = setOutputFrame(sys,obj.getOutputFrame());
      
      sys = setSampleTime(sys,obj.getSampleTime);
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

