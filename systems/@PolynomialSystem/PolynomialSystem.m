classdef PolynomialSystem < RobotLibSystem %TimeVaryingPolynomialSystem 
  % A dynamical system described by rational polynomial dynamics, and
  % polynomial outputs.  

  properties (SetAccess=private)
    p_dynamics % msspoly representations of dynamics, update,and output
    p_update
    p_output
    
    p_state_constraints
    p_mass_matrix % msspoly representation for e(x), as in e(x)xdot=f(x).  if empty then the default is e(x)=I.  must be invertible for all x.
  end

  methods
%    function x0=getInitialState(obj)
%      x0=zeros(getNumStates(obj),1);
%    end
    
    function obj = PolynomialSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,p_dynamics,p_update,p_output)
      % initialize PolynomialSystem
      %
      % There are effectively four ways to initialize a poly system
      %
      % Method 1:
      %   Call this constructor by handing in the msspoly objects for
      %   dynamics, update, and output.  Those polys will be used as
      %   expected.
      %   
      % Method 2: 
      %   Call this constructor without handing in the msspoly objects.
      %   The constructor will try to automatically construct the relevant
      %   polynomials by calling the dynamics, update, and output methods.
      %   You *must* overload those methods in your subclass for this to
      %   work (otherwise you will recieve an error).
      %
      % Method 3:
      %   Call this constructor handing in [] in the place of the msspoly
      %   objects.  Then, separately, manually call the method
      %   'pullEmptyPolysFromMethods'.  This seemingly bizarre calling
      %   method is necessary because method 2 does not work if the
      %   dynamics, update, or output methods of your subclass rely on
      %   being initialized (e.g. in the subclass constructor). 
      %
      % Method 4:
      %   Call this constructor handing in function handles, taking 
      %   input arguments (t,x,u), in the place of the msspoly objects.  
      %   The constructor will try to automatically construct the 
      %   relevant polynomials by calling the function handle methods.
      
      obj = obj@RobotLibSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,true);

      checkDependency('spot_enabled');
      
      % Now create the msspoly versions of the dynamics,update,and output:
      p_t=msspoly('t',1);
      p_x=obj.getStateFrame.poly;
      p_u=obj.getInputFrame.poly;
      
      % these will error if the system is not polynomial (should I catch
      % and rethrow the error with more information?)
      if (nargin>5 && num_xc>0)
        if isempty(p_dynamics)  % support method 3
        elseif isa(p_dynamics,'function_handle') % support method 4
          obj.p_dynamics = p_dynamics(p_t,p_x,p_u);
        else % method 1
          typecheck(p_dynamics,'msspoly');
          if any([p_dynamics.m,p_dynamics.n] ~= [num_xc,1]) error('p_dynamics does not match num_xc'); end
          if ~isempty(setdiff(decomp(p_dynamics),[p_x;p_u]))
            error('p_dynamics depends on variables that do not match my free variables representing t, x, and u.'); 
          end
          obj.p_dynamics=p_dynamics;
        end
      elseif (num_xc>0) % method 2
        obj.p_dynamics=obj.dynamics(p_t,p_x,p_u);
      end
      if ~isempty(obj.p_dynamics) && deg(obj.p_dynamics,p_t)>0
        error('polynomial dynamics must be time invariant');
      end
      
      if (nargin>6 && num_xd>0)
        if isempty(p_update) % support method 3
        elseif isa(p_update,'function_handle') % method 4
          obj.p_update = p_update(p_t,p_x,p_u);
        else % method 1
          typecheck(p_update,'msspoly');
          if any([p_update.m,p_update.n] ~= [num_xd,1]) error('p_update does not match num_xd'); end
          if ~isempty(setdiff(decomp(p_update),[p_t;p_x;p_u]))
            error('p_update depends on variables that do not match my free variables representing t, x, and u.'); 
          end
          obj.p_update = p_update;
        end
      elseif (num_xd>0) % method 2
        obj.p_update=obj.update(p_t,p_x,p_u);
      end
      if ~isempty(obj.p_update) && deg(obj.p_update,p_t)>0
        error('polynomial update must be time invariant');
      end
      
      if (nargin>7 && num_y>0)
        if isempty(p_output) % support method 3
        elseif isa(p_output,'function_handle') % method 4
          obj.p_output = p_output(p_t,p_x,p_u);
        else % method 1
          typecheck(p_output,'msspoly');
          if any([p_output.m,p_output.n] ~= [num_y,1]) error('p_output does not match num_y'); end
          if ~isempty(setdiff(decomp(p_output),[p_t;p_x;p_u]))
            error('p_output depends on variables that do not match my free variables representing t, x, and u.'); 
          end
          obj.p_output=p_output;
        end
      elseif (num_y>0) % method 2
        obj.p_output=obj.output(p_t,p_x,p_u);
      end
      if ~isempty(obj.p_output) && deg(obj.p_output,p_t)>0
        error('polynomial output must be time invariant');
      end
    end
    
    function obj = addStateConstraints(obj,p_state_constraints)
      typecheck(p_state_constraints,'msspoly');
      p_x=obj.getStateFrame.poly;
      if ~isempty(setdiff(decomp(p_state_constraints),p_x))
%      if (any(match(obj.p_x,decomp(p_state_constraints))==0))
        error('state constraints must depend only on x');
      end
      if (~iscolumn(p_state_constraints))
        error('state constraints must be a column vector');
      end
      obj.p_state_constraints = [obj.p_state_constraints; p_state_constraints];
      obj = obj.setNumStateConstraints(length(obj.p_state_constraints));
    end
    
    function obj = setMassMatrix(obj,p_mass_matrix)
      if (isempty(p_mass_matrix)) % setting it back to empty is AOK
        obj.p_mass_matrix=[];
      else
        typecheck(p_mass_matrix,'msspoly');
        sizecheck(p_mass_matrix,[length(obj.p_dynamics),obj.num_xc]);
        p_x=obj.getStateFrame.poly;
        if ~isempty(setdiff(decomp(p_state_constraints),p_x))
          error('mass matrix must depend only on x');
        end
        obj.p_mass_matrix = p_mass_matrix;
      end
    end
    
    function obj = pullEmptyPolysFromMethods(obj)
      p_t=msspoly('t',1);
      p_x=obj.getStateFrame.poly;
      p_u=obj.getInputFrame.poly;
      
      if (isempty(obj.p_dynamics) && obj.num_xc>0)
        obj.p_dynamics=obj.dynamics(p_t,p_x,p_u);
        if deg(obj.p_dynamics,p_t)>0
          error('polynomial dynamics must be time invariant');
        end
      end
      if (isempty(obj.p_update) && obj.num_xd>0)
        obj.p_update=obj.update(p_t,p_x,p_u);
        if deg(obj.p_update,p_t)>0
          error('polynomial update must be time invariant');
        end
      end
      if (isempty(obj.p_output) && obj.num_y>0)
        obj.p_output=obj.output(p_t,p_x,p_u);
        if deg(obj.p_output,p_t)>0
          error('polynomial output must be time invariant');
        end
      end
    end
    
    % Implement default methods using msspoly vars explicitly
    function [xcdot,df] = dynamics(obj,t,x,u)
      % should only get here if constructor specified p_dynamics
      if (isempty(obj.p_dynamics)) error('p_dynamics is not defined.  how did you get here?'); end
      p_x=obj.getStateFrame.poly; 
      p_u=obj.getInputFrame.poly; 
      xcdot = double(subs(obj.p_dynamics,[p_x;p_u],[x;u]));
      if (nargout>1)
        df = [zeros(obj.num_xc,1),double(subs(diff(obj.p_dynamics,[p_t;p_x;p_u]),[p_x;p_u],[x;u]))];
      end
      if (~isempty(obj.p_mass_matrix))
        e = double(subs(obj.p_mass_matrix,p_x,x));
        xcdot = e\xcdot;
        if (nargout>1) 
          % e(x)xdot = f(x) => dexdotdx + e(x)*dxdotdx = dfdx
          %  where the columns of dexdotdx are dedxi*xdot
          nX = obj.num_xc; cellxcdot=cell(1,nX); [cellxcdot{:}]=deal(xcdot); 
          dexdotdx = reshape(double(subs(diff(obj.p_mass_matrix(:),p_x),p_x,x)),nX,[])*blkdiag(cellxcdot{:}); 
          df = e\(df - [zeros(nX,1),dexdotdx,zeros(nX,obj.num_u)]);
        end
      end
    end
    function [xdn,df] = update(obj,t,x,u)
      if (isempty(obj.p_update)) error('p_update is not defined.  how did you get here?'); end
      p_x=obj.getStateFrame.poly;
      p_u=obj.getInputFrame.poly;
      xdn = double(subs(obj.p_update,[p_x;p_u],[x;u]));
      if (nargout>1)
        df = [zeros(obj.num_xd,1),double(subs(diff(obj.p_update,[p_x;p_u]),[p_x;p_u],[x;u]))];
      end
    end
    function [y,dy] = output(obj,t,x,u)
      if (isempty(obj.p_output)) error('p_output is not defined.  how did you get here?'); end
      p_x=obj.getStateFrame.poly;
      if (nargin<4 || obj.num_u<1) p_u=[]; u=[]; else p_u=obj.getInputFrame.poly; end  % ok for systems without direct feedthrough
      y = double(subs(obj.p_output,[p_x;p_u],[x;u]));
      if (nargout>1)
        dy = [zeros(obj.num_y,1),double(subs(diff(obj.p_output,[p_x;p_u]),[p_x;obj.p_u],[x;u]))];
      end
    end
    
    function obj = setNumZeroCrossings(obj,num_zcs)
      if (num_zcs)
        error('PolynomialSystems are not allowed to have zero crossings');
      end
    end
  end
  
  methods  % for constructing and manipulating polynomial systems
    function polysys = timeReverse(obj)
      if (obj.num_xd>0) error('only for CT systems'); end
      polysys = PolynomialSystem(obj.num_xc,obj.num_xd,obj.num_u,obj.num_u,obj.direct_feedthrough_flag,obj.time_invariant_flag,...
        -obj.p_dynamics,[],obj.p_output);
      if (obj.num_xcon)
        polysys = polysys.addStateConstraints(obj.p_state_constraints);
      end
      if (~isempty(obj.p_mass_matrix))
        polysys = polysys.setMassMatrix(obj.p_mass_matrix);
      end
    end
    
    function sys = feedback(sys1,sys2)
      % try to keep feedback between polynomial systems polynomial.  else,
      % kick out to RobotLibSystem
      %
      
      [sys1,sys2] = matchCoordinateFramesForCombination(sys1,sys2,false);
      [sys2,sys1] = matchCoordinateFramesForCombination(sys2,sys1,true);

      if ~isa(sys2,'PolynomialSystem') || any(~isinf([sys1.umin;sys1.umax;sys2.umin;sys2.umax]))
        sys = feedback@RobotLibSystem(sys1,sys2)
      end
        
      if (sys1.isDirectFeedthrough() && sys2.isDirectFeedthrough())
        error('RobotLib:PolynomalSystem:AlgebraicLoop','algebraic loop');
      end
      
      if (getNumZeroCrossings(sys1)>0 || getNumZeroCrossings(sys2)>0)
        error('polynomialsystems aren''t supposed to have zero crossings'); 
      end
      
      p_x = msspoly('x',sys1.getNumStates()+sys2.getNumStates());  % note: this assumes that the default state frame in the polynomial constructor below uses the prefix 'x'
      p_u = getInputFrame(sys1).poly;  % note: this assumes that the default input frame in the poly constructor uses the same prefix as sys1.getInputFrame
      
      ind=0;
      n=sys1.getNumDiscStates();
      p_x1 = p_x(ind+(1:n)');
      ind=ind+n;
      n=sys2.getNumDiscStates();
      p_x2 = p_x(ind+(1:n)');
      ind=ind+n;
      
      n=sys1.getNumContStates();
      p_x1 = [p_x1; p_x(ind+(1:n)')];
      ind=ind+n;
      n=sys2.getNumContStates();
      p_x2 = [p_x2; p_x(ind+(1:n)')];
      
      if (~sys1.isDirectFeedthrough()) % do sys1 first
        p_y1 = subs(sys1.p_output,sys1.getStateframe.poly,p_x1); % doesn't need u
        p_y2 = subss(sys2.p_output,[sys2.getStateFrame.poly;sys2.p_u],[p_x2;p_y1]);
      else % do sys2 first
        p_y2 = subs(sys2.p_output,sys2.getStateFrame.poly,p_x2); % doesn't need u
        p_y1 = subss(sys1.p_output,[sys1.getStateFrame.poly;p_u],[p_x1;p_y2+p_u]);
      end
      
      p_dynamics = []; p_update=[]; p_output=[]; 
      if (sys1.getNumContStates()>0)
        p_dynamics=[p_dynamics;subss(sys1.p_dynamics,[sys1.getStateFrame.poly;p_u],[p_x1;p_y2+p_u])];
      end
      if (sys2.getNumContStates()>0)
        p_dynamics=[p_dynamics;subss(sys2.p_dynamics,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1])];
      end
      if (sys1.getNumDiscStates()>0)
        p_update = [p_update;subss(sys1.p_update,[sys1.getStateFrame.poly;p_u],[p_x1;p_y2+p_u])];
      end
      if (sys2.getNumDiscStates()>0)
        p_update = [p_update; subss(sys2.p_update,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1])];
      end
      if (sys1.getNumOutputs()>0)
        p_output = p_y1;
      end
      
      sys = PolynomialSystem(sys1.getNumContStates()+sys2.getNumContStates(),...
        sys1.getNumDiscStates()+sys2.getNumDiscStates(),...
        sys1.getNumInputs(), sys1.getNumOutputs(), sys1.isDirectFeedthrough, ...
        p_dynamics,p_update,p_output);
      
      % handle state constraints
      p_state_constraints=[];
      if (sys1.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(sys1.p_state_constraints,sys1.getStateFrame.poly,p_x1)];
      end
      if (sys2.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(sys2.p_state_constraints,sys2.getStateFrame.poly,p_x2)];
      end
      sys = addStateConstraints(sys,p_state_constraints);
      
      try 
        sys = setSampleTime(sys,[sys1.getSampleTime(),sys2.getSampleTime()]);  % todo: if this errors, then kick out to robotlibsystem?
      catch ex
        if (strcmp(ex.identifier, 'RobotLib:RobotLibSystem:UnsupportedSampleTime'))
          warning('RobotLib:PolynomialSystem:UnsupportedSampleTime','Aborting polynomial feedback because of incompatible sample times'); 
          sys = feedback@RobotLibSystem(sys1,sys2);
        else
          rethrow(ex)
        end
      end
      
      sys = setInputFrame(sys,sys1.getInputFrame());
      sys = setOutputFrame(sys,sys1.getOutputFrame());
      
      % set mass matrix
      if isempty(sys1.p_mass_matrix)
        if isempty(sys2.p_mass_matrix)
          p_mass_matrix=[];
        else
          p_mass_matrix=blkdiag(eye(getNumContStates(sys1)),subss(sys2.p_mass_matrix,sys2.getStateFrame.poly,p_x2));
        end
      else
        if isempty(sys2.p_mass_matrix)
          p_mass_matrix=blkdiag(subss(sys1.p_mass_matrix,sys1.getStateFrame.poly,p_x1),eye(getNumContStates(sys2)));
        else
          p_mass_matrix=blkdiag(subss(sys1.p_mass_matrix,sys1.getStateFrame.poly,p_x1),subss(sys2.p_mass_matrix,sys2.getStateFrame.poly,p_x2));
        end
      end
      sys = setMassMatrix(sys,p_mass_matrix);
    end
    
    function sys = cascade(sys1,sys2)
      % try to keep cascade between polynomial systems polynomial.  else,
      % kick out to RobotLibSystem
      %

      [sys1,sys2] = matchCoordinateFramesForCombination(sys1,sys2);

      if ~isa(sys2,'PolynomialSystem') || any(~isinf([sys2.umin;sys2.umax]))
        sys = cascade@RobotLibSystem(sys1,sys2)
      end
        
      if (getNumZeroCrossings(sys1)>0 || getNumZeroCrossings(sys2)>0)
        error('polynomialsystems aren''t supposed to have zero crossings'); 
      end
      
      p_x = msspoly('x',sys1.getNumStates()+sys2.getNumStates());  % note: this assumes that the default state frame in the polynomial constructor below uses the prefix 'x'
      p_u = sys1.getInputFrame.poly;  % note: this assumes that the default input frame in the poly constructor uses the same prefix as sys1.getInputFrame
      
      ind=0;
      n=sys1.getNumDiscStates();
      p_x1 = p_x(ind+(1:n)');
      ind=ind+n;
      n=sys2.getNumDiscStates();
      p_x2 = p_x(ind+(1:n)');
      ind=ind+n;
      
      n=sys1.getNumContStates();
      p_x1 = [p_x1; p_x(ind+(1:n)')];
      ind=ind+n;
      n=sys2.getNumContStates();
      p_x2 = [p_x2; p_x(ind+(1:n)')];
      
      if (sys1.getNumStates()>0)
        p_y1 = subss(sys1.p_output,sys1.getStateFrame.poly,p_x1);
      else
        p_y1 = sys1.p_output;
      end

      p_dynamics = []; p_update=[]; p_output=[]; 
      if (sys1.getNumContStates()>0)
        p_dynamics=[p_dynamics;subss(sys1.p_dynamics,sys1.getStateFrame.poly,p_x1)];
      end
      if (sys2.getNumContStates()>0)
        p_dynamics=[p_dynamics;subss(sys2.p_dynamics,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1])];
      end
      if (sys1.getNumDiscStates()>0)
        p_update = [p_update;subss(sys1.p_update,sys1.getStateFrame.poly,p_x1)];
      end
      if (sys2.getNumDiscStates()>0)
        p_update = [p_update; subss(sys2.p_update,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1])];
      end
      if (sys1.getNumOutputs()>0)
        p_output = subss(sys2.p_output,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1]);
      end
      
      sys = PolynomialSystem(sys1.getNumContStates()+sys2.getNumContStates(),...
        sys1.getNumDiscStates()+sys2.getNumDiscStates(),...
        sys1.getNumInputs(), sys2.getNumOutputs(), sys1.isDirectFeedthrough & sys2.isDirectFeedthrough, ...
        p_dynamics,p_update,p_output);
      
      % handle state constraints
      p_state_constraints=[];
      if (sys1.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(sys1.p_state_constraints,sys1.getStateFrame.poly,p_x1)];
      end
      if (sys2.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(sys2.p_state_constraints,sys2.getStateFrame.poly,p_x2)];
      end
      sys = addStateConstraints(sys,p_state_constraints);
      
      try 
        sys = setSampleTime(sys,[sys1.getSampleTime(),sys2.getSampleTime()]);  % todo: if this errors, then kick out to robotlibsystem?
      catch ex
        if (strcmp(ex.identifier, 'RobotLib:RobotLibSystem:UnsupportedSampleTime'))
          warning('RobotLib:PolynomialSystem:UnsupportedSampleTime','Aborting polynomial cascade because of incompatible sample times'); 
          sys = cascade@RobotLibSystem(sys1,sys2);
        else
          rethrow(ex)
        end
      end
      
      sys = setInputFrame(sys,sys1.getInputFrame());
      sys = setOutputFrame(sys,sys1.getOutputFrame());
      
      % set mass matrix
      if isempty(sys1.p_mass_matrix)
        if isempty(sys2.p_mass_matrix)
          p_mass_matrix=[];
        else
          p_mass_matrix=blkdiag(eye(getNumContStates(sys1)),subss(sys2.p_mass_matrix,sys2.getStateFrame.poly,p_x2));
        end
      else
        if isempty(sys2.p_mass_matrix)
          p_mass_matrix=blkdiag(subss(sys1.p_mass_matrix,sys1.getStateFrame.poly,p_x1),eye(getNumContStates(sys2)));
        else
          p_mass_matrix=blkdiag(subss(sys1.p_mass_matrix,sys1.getStateFrame.poly,p_x1),subss(sys2.p_mass_matrix,sys2.getStateFrame.poly,p_x2));
        end
      end
      sys = setMassMatrix(sys,p_mass_matrix);
    end
  end
    
end
