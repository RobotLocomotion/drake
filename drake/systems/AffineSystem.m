classdef AffineSystem < PolynomialSystem
% Implements
%   xcdot = Ac*x + Bc*u + xcdot0
%   xdn = Ad*x + Bd*u + xdn0
%   y = C*x + D*u + y0

  methods
    function obj = AffineSystem(Ac,Bc,xcdot0,Ad,Bd,xdn0,C,D,y0)
      num_xc = max([size(Ac,1),size(Bc,1),size(xcdot0,1)]);
      num_xd = max([size(Ad,1),size(Bd,1),size(xdn0,1)]);
      num_u = max([size(Bc,2),size(Bd,2),size(D,2)]);
      num_y = max([size(C,1),size(D,1),size(y0,1)]);
      tiflag = isnumeric(Ac) && isnumeric(Bc) && isnumeric(xcdot0) ...
        && isnumeric(Ad) && isnumeric(Bd) && isnumeric(xdn0) ...
        && isnumeric(C) && isnumeric(D) && isnumeric(y0);
      % note: currently taylovar's will cause the system to be marked as time-varying

      obj = obj@PolynomialSystem(num_xc,num_xd,num_u,num_y,false,tiflag,false);

      if (isempty(Ac))
        Ac = sparse(num_xc,num_xc+num_xd);
      else
        sizecheck(Ac,[num_xc,num_xc+num_xd]);
      end
      if ~tiflag
        if isnumeric(Ac) Ac = ConstantTrajectory(Ac); end
        typecheck(Ac,'Trajectory');
      end
      obj.Ac = Ac;

      if (isempty(Bc))
        Bc = sparse(num_xc,num_u);
      else
        sizecheck(Bc,[num_xc,num_u]);
      end
      if ~tiflag
        if isnumeric(Bc) Bc = ConstantTrajectory(Bc); end
        typecheck(Bc,'Trajectory');
      end
      obj.Bc = Bc;

      if (isempty(xcdot0))
        xcdot0 = sparse(num_xc,1);
      else
        sizecheck(xcdot0,[num_xc,1]);
      end
      if ~tiflag
        if isnumeric(xcdot0) xcdot0 = ConstantTrajectory(xcdot0); end
        typecheck(xcdot0,'Trajectory');
      end
      obj.xcdot0 = xcdot0;

      if (isempty(Ad))
        Ad = sparse(num_xd,num_xc+num_xd);
      else
        sizecheck(Ad,[num_xd,num_xc+num_xd]);
      end
      if ~tiflag
        if isnumeric(Ad) Ad = ConstantTrajectory(Ad); end
        typecheck(Ad,'Trajectory');
      end
      obj.Ad = Ad;

      if (isempty(Bd))
        Bd = sparse(num_xd,num_u);
      else
        sizecheck(Bd,[num_xd,num_u]);
      end
      if ~tiflag
        if isnumeric(Bd) Bd = ConstantTrajectory(Bd); end
        typecheck(Bd,'Trajectory');
      end
      obj.Bd = Bd;

      if (isempty(xdn0))
        xdn0 = sparse(num_xd,1);
      else
        sizecheck(xdn0,[num_xd,1]);
      end
      if ~tiflag
        if isnumeric(xdn0) xdn0 = ConstantTrajectory(xdn0); end
        typecheck(xdn0,'Trajectory');
      end
      obj.xdn0 = xdn0;

      if (isempty(C))
        C = sparse(num_y,num_xc+num_xd);
      else
        sizecheck(C,[num_y,num_xc+num_xd]);
      end
      if ~tiflag
        if isnumeric(C) C = ConstantTrajectory(C); end
        typecheck(C,'Trajectory');
      end
      obj.C = C;

      if (isempty(D))
        D = sparse(num_y,num_u);
      else
        sizecheck(D,[num_y,num_u]);
      end
      if ~tiflag
        if isnumeric(D) D = ConstantTrajectory(D); end
        typecheck(D,'Trajectory');
      end
      obj.D = D;
      if (~tiflag || any(D(:)))  % only if D is non-zero
        obj = setDirectFeedthrough(obj,true);
      end

      if (isempty(y0))
        y0 = sparse(num_y,1);
      else
        sizecheck(y0,[num_y,1]);
      end
      if ~tiflag
        if isnumeric(y0) y0 = ConstantTrajectory(y0); end
        typecheck(y0,'Trajectory');
      end
      obj.y0 = y0;
    end

    function [xcdot,df] = dynamicsRHS(obj,t,x,u)
      if (isTI(obj))
        xcdot=obj.Ac*x;
        if (obj.num_u) xcdot=xcdot+obj.Bc*u; end
        if ~isempty(obj.xcdot0) xcdot=xcdot+obj.xcdot0; end
        if (nargout>1)
          df = [0*xcdot, obj.Ac, obj.Bc];
        end
      else
        xcdot=obj.Ac.eval(t)*x;
        if (obj.num_u) xcdot=xcdot+obj.Bc.eval(t)*u; end
        if ~isempty(obj.xcdot0) xcdot=xcdot+obj.xcdot0.eval(t); end
        if (nargout>1)
            df = [deriv(obj.Ac,t)*x+deriv(obj.Bc,t)*u obj.Ac.eval(t) obj.Bc.eval(t)];
            if ~isempty(obj.xcdot0)
                df(:,1) = df(:,1)+deriv(obj.xcdot0,t);
            end
        end
      end
    end

    function [xdn,df] = update(obj,t,x,u)
      if (isTI(obj))
        xdn=obj.Ad*x;
        if (obj.num_u) xdn=xdn+obj.Bd*u; end
        if ~isempty(obj.xdn0) xdn=xdn+obj.xdn0; end
        if (nargout>1)
          df = [0*xdn,obj.Ad,obj.Bd];
        end
      else
        xdn=obj.Ad.eval(t)*x;
        if (obj.num_u) xdn=xdn+obj.Bd.eval(t)*u; end
        if ~isempty(obj.xdn0) xdn=xdn+obj.xdn0.eval(t); end
      end
    end

    function [y,dy] = output(obj,t,x,u)
      if (isTI(obj))
        y=zeros(obj.num_y,1);
        if (obj.num_x) y=y+obj.C*x; end
        if (obj.num_u) y=y+obj.D*u; end
        if ~isempty(obj.y0) y=y+obj.y0; end
        if (nargout>1)
          dy = [0*y,obj.C,obj.D];
        end
      else
        y=zeros(obj.num_y,1);
        if (obj.num_x) y=y+eval(obj.C,t)*x; end
        if (obj.num_u) y=y+eval(obj.D,t)*u; end
        if ~isempty(obj.y0) y=y+eval(obj.y0,t); end
        if (nargout>1)
          dy = zeros(obj.num_y,1+obj.num_x+obj.num_u);
          if (obj.num_x) dy(:,1:1+obj.num_x)=dy(:,1:1+obj.num_x) + [deriv(obj.C,t)*x, eval(obj.C,t)]; end
          if (obj.num_u) dy(:,[1,2+obj.num_x:end])=dy(:,[1,2+obj.num_x:end])+[deriv(obj.D,t)*u,eval(obj.D,t)]; end
          if ~isempty(obj.y0) dy(:,1)=dy(:,1)+deriv(obj.y0,t); end
        end
      end
    end

    function sys = feedback(sys1,sys2)
      % try to keep feedback between affine systems affine.  else,
      % kick out to DrakeSystem
      %

      sys2 = sys2.inInputFrame(sys1.getOutputFrame);
      sys2 = sys2.inOutputFrame(sys1.getInputFrame);

      if ~isa(sys2,'AffineSystem') || any(~isinf([sys1.umin;sys1.umax;sys2.umin;sys2.umax]))
        sys = feedback@PolynomialSystem(sys1,sys2);
        return;
      end

      if (sys1.isDirectFeedthrough() && sys2.isDirectFeedthrough())
        error('Drake:AffineSystem:AlgebraicLoop','algebraic loop');
      end

      if (getNumZeroCrossings(sys1)>0 || getNumZeroCrossings(sys2)>0)
        error('affine systems aren''t supposed to have zero crossings');
      end
      if (getNumStateConstraints(sys1)>0 || getNumStateConstraints(sys2)>0 || getNumUnilateralConstraints(sys1)>0 || getNumUnilateralConstraints(sys2)>0)
        error('affine systems aren''t supposed to have state constraints');
      end

      %x1dot = (A1+B1D2C1)x1 + B1C2x2 + B1u + (B1D2y01 + B1y02 + xdot01)
      %x2dot = B2C1x1 + (A2+B2D1C2)x2 + B2D1u + (B2D1y02 + B2y01 + xdot02)
      %y = C1x1 + D1C2x2 + D1u + (D1y02 + y01)

      [sys1ind,sys2ind] = stateIndicesForCombination(sys1,sys2);

      if isempty(sys1ind) && isempty(sys2ind)
        Ac=[]; Ad=[]; C=[];
      end
      if ~isempty(sys1ind)
        Ac(:,sys1ind) = [sys1.Ac + sys1.Bc*sys2.D*sys1.C; sys2.Bc*sys1.C];
        Ad(:,sys1ind) = [sys1.Ad + sys1.Bd*sys2.D*sys1.C; sys2.Bd*sys1.C];
        C(:,sys1ind) = sys1.C;
      end
      if ~isempty(sys2ind)
        Ac(:,sys2ind) = [sys1.Bc*sys2.C; sys2.Ac + sys2.Bc*sys1.D*sys2.C];
        Ad(:,sys2ind) = [sys1.Bd*sys2.C; sys2.Ad + sys2.Bd*sys1.D*sys2.C];
        C(:,sys2ind) = sys1.D*sys2.C;
      end

      Bc = [sys1.Bc; sys2.Bc*sys1.D];
      xcdot0 = [sys1.Bc*sys2.D*sys1.y0 + sys1.Bc*sys2.y0 + sys1.xcdot0; sys2.Bc*sys1.D*sys2.y0 + sys2.Bc*sys1.y0 + sys2.xcdot0];

      Bd = [sys1.Bd; sys2.Bd*sys1.D];
      xdn0 = [sys1.Bd*sys2.D*sys1.y0 + sys1.Bd*sys2.y0 + sys1.xdn0; sys2.Bd*sys1.D*sys2.y0 + sys2.Bd*sys1.y0 + sys2.xdn0];

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
      % try to keep cascade between affine systems affine.  else,
      % kick out to DrakeSystem
      %

      sys2 = sys2.inInputFrame(sys1.getOutputFrame);

      if ~isa(sys2,'AffineSystem') || any(~isinf([sys2.umin;sys2.umax]))
        sys = cascade@DrakeSystem(sys1,sys2);
        return;
      end

      if (getNumZeroCrossings(sys1)>0 || getNumZeroCrossings(sys2)>0)
        error('polynomialsystems aren''t supposed to have zero crossings');
      end

      % x1dot = A1x1 + B1u + xcdot01
      % x2dot = A2x2 + B2(C1x1+D1u+y01) + xcdot02
      % y = C2x2 + D2(C1x1+D1u + y01) + y02

      [sys1ind,sys2ind] = stateIndicesForCombination(sys1,sys2);

      if isempty([sys1ind;sys2ind])
        Ac=[]; Ad=[]; C=[];
      end
      if ~isempty(sys1ind)
        Ac(:,sys1ind) = [sys1.Ac; sys2.Bc*sys1.C];
        Ad(:,sys1ind) = [sys1.Ad; sys2.Bd*sys1.C];
        C(:,sys1ind) = sys2.D*sys1.C;
      end
      if ~isempty(sys2ind)
        Ac(:,sys2ind) = [zeros(getNumContStates(sys1),getNumStates(sys2)); sys2.Ac];
        Ad(:,sys2ind) = [zeros(getNumDiscStates(sys1),getNumStates(sys2)); sys2.Ad];
        C(:,sys2ind) = sys2.C;
      end

      Bc = [sys1.Bc; sys2.Bc*sys1.D];
      xcdot0 = [sys1.xcdot0; sys2.Bc*sys1.y0 + sys2.xcdot0];

      Bd = [sys1.Bd; sys2.Bd*sys1.D];
      xdn0 = [sys1.xdn0; sys2.Bd*sys1.y0 + sys2.xdn0];

      D = sys2.D*sys1.D;
      y0 = sys2.D*sys1.y0 + sys2.y0;

      sys = AffineSystem(Ac,Bc,xcdot0,Ad,Bd,xdn0,C,D,y0);

      sys = setInputFrame(sys,sys1.getInputFrame());
      sys = setOutputFrame(sys,sys2.getOutputFrame());

      try
        sys = setSampleTime(sys,[sys1.getSampleTime(),sys2.getSampleTime()]);  % todo: if this errors, then kick out to drakesystem?
      catch ex
        if (strcmp(ex.identifier, 'Drake:DrakeSystem:UnsupportedSampleTime'))
          warning('Drake:AffineSystem:UnsupportedSampleTime','Aborting polynomial cascade because of incompatible sample times');
          sys = cascade@DrakeSystem(sys1,sys2);
        else
          rethrow(ex)
        end
      end
    end

    function sys = parallel(sys1,sys2)
      % try to keep parallel combinations between affine systems affine.  else,
      % kick out to DrakeSystem
      %

      if ~isa(sys2,'AffineSystem')
        sys = parallel@DrakeSystem(sys1,sys2);
        return;
      end

      % x1dot = A1x1 + B1u + xcdot01
      % x2dot = A2x2 + B2(C1x1+D1u+y01) + xcdot02
      % y = C2x2 + D2(C1x1+D1u + y01) + y02

      [sys1ind,sys2ind] = stateIndicesForCombination(sys1,sys2);

      if isempty([sys1ind;sys2ind])
        Ac=[]; Ad=[]; Bc=[]; Bd=[]; C=[];
      end
      if ~isempty(sys1ind)
        Ac(:,sys1ind) = [sys1.Ac; zeros(getNumContStates(sys2),getNumStates(sys1))];
        Ad(:,sys1ind) = [sys1.Ad; zeros(getNumDiscStates(sys2),getNumStates(sys1))];
        C(:,sys1ind) = [sys1.C;zeros(getNumOutputs(sys2),getNumStates(sys1))];
      end
      if ~isempty(sys2ind)
        Ac(:,sys2ind) = [zeros(getNumContStates(sys1),getNumStates(sys2)); sys2.Ac];
        Ad(:,sys2ind) = [zeros(getNumDiscStates(sys1),getNumStates(sys2)); sys2.Ad];
        C(:,sys2ind) = [zeros(getNumOutputs(sys1),getNumStates(sys2)); sys2.C];
      end

      Bc = blkdiag(sys1.Bc,sys2.Bc);
      xcdot0 = [sys1.xcdot0; sys2.xcdot0];

      Bd = blkdiag(sys1.Bd,sys2.Bd);
      xdn0 = [sys1.xdn0; sys2.xdn0];

      D = blkdiag(sys1.D,sys2.D);
      y0 = [sys1.y0; sys2.y0];

      sys = AffineSystem(Ac,Bc,xcdot0,Ad,Bd,xdn0,C,D,y0);

      sys = setInputFrame(sys,MultiCoordinateFrame.constructFrame({sys1.getInputFrame(),sys2.getInputFrame()}));
      sys = setOutputFrame(sys,MultiCoordinateFrame.constructFrame({sys1.getOutputFrame(),sys2.getOutputFrame()}));

      try
        sys = setSampleTime(sys,[sys1.getSampleTime(),sys2.getSampleTime()]);  % todo: if this errors, then kick out to drakesystem?
      catch ex
        if (strcmp(ex.identifier, 'Drake:DrakeSystem:UnsupportedSampleTime'))
          warning('Drake:AffineSystem:UnsupportedSampleTime','Aborting polynomial cascade because of incompatible sample times');
          sys = parallel@DrakeSystem(sys1,sys2);
        else
          rethrow(ex)
        end
      end
    end

    function sys = extractLinearSystem(obj)
      if any([obj.xcdot0;obj.xdn0;obj.y0]~=0)
        warning('Drake:AffineSystem:extractLinearSystem','affine terms are not all zero');
      end

      sys = LinearSystem(obj.Ac,obj.Bc,obj.Ad,obj.Bd,obj.C,obj.D);

      sys = setInputFrame(sys,obj.getInputFrame());
      sys = setStateFrame(sys,obj.getStateFrame());
      sys = setOutputFrame(sys,obj.getOutputFrame());

      sys = setSampleTime(sys,obj.getSampleTime);
    end
    
    
    function [prog,x_inds,u_inds] = dirtranModelPredictiveControl(obj,N)
      % sets up a Quadratic Program with the dynamics constraints
      % x[n+1] = Ad*x[n] + Bd*u[n] + xdn0;
      if ~isDT(obj)
        error('not implemented yet');
      end
      
      % decision variables:
      % x[0],x[1],...,x[N], u[0],...,u[N-1]
      num_x = obj.num_xd;
      x_inds = reshape(1:num_x*(N+1),num_x,N+1);
      u_inds = numel(x_inds) + reshape(1:obj.num_u*N,obj.num_u,N);

      num_vars = numel(x_inds)+numel(u_inds);
      
      Aeq = zeros(num_x*N,num_vars);
      beq = zeros(num_x*N,1);
      
      for i=1:N % todo: vectorize this
        c_inds = num_x*(i-1)+(1:num_x);

        % x[n+1]
        Aeq(c_inds,x_inds(:,i+1)) = eye(num_x);
        % -Ad*x[n]
        Aeq(c_inds,x_inds(:,i)) = -obj.Ad;
        % -Bd*u[n]
        Aeq(c_inds,u_inds(:,i)) = -obj.Bd;
        
        % xdn0
        beq(c_inds) = obj.xdn0;
      end
      
      prog = QuadraticProgram(zeros(num_vars),zeros(num_vars,1),[],[],Aeq,beq);
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
