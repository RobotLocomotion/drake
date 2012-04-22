classdef PolynomialSystem < SmoothRobotLibSystem 
  % A dynamical system described by rational polynomial dynamics, and
  % polynomial outputs.

  properties (SetAccess=private)
    p_t
    p_x
    p_u

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
    
    function obj = PolynomialSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag,p_dynamics,p_update,p_output)
      % initialize PolynomialSystem
      %
      % There are effectively three ways to initialize a poly system
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
      
      obj = obj@SmoothRobotLibSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag);

      checkDependency('spot_enabled');
      
      % Now create the msspoly versions of the dynamics,update,and output:
      obj.p_t=msspoly('t',1);
      if (num_xc+num_xd>0)
        obj.p_x=msspoly('x',num_xc+num_xd);
      end
      if (num_u>0)
        obj.p_u=msspoly('u',num_u);
      end
      
      % these will error if the system is not polynomial (should I catch
      % and rethrow the error with more information?)
      if (nargin>6 && num_xc>0)
        if ~isempty(p_dynamics)  % support method 3
          typecheck(p_dynamics,'msspoly');
          if any([p_dynamics.m,p_dynamics.n] ~= [num_xc,1]) error('p_dynamics does not match num_xc'); end
          obj.p_dynamics=p_dynamics;
        end
      elseif (num_xc>0)
        obj.p_dynamics=obj.dynamics(obj.p_t,obj.p_x,obj.p_u);
      end
      
      if (nargin>7 && num_xd>0)
        if ~isempty(p_update) % support method 3
          typecheck(p_update,'msspoly');
          if any([p_update.m,p_update.n] ~= [num_xd,1]) error('p_update does not match num_xd'); end
          obj.p_update = p_update;
        end
      elseif (num_xd>0)
        obj.p_update=obj.update(obj.p_t,obj.p_x,obj.p_u);
      end
      
      if (nargin>8 && num_y>0)
        if ~isempty(p_output) % support method 3
          typecheck(p_output,'msspoly');
          if any([p_output.m,p_output.n] ~= [num_y,1]) error('p_output does not match num_y'); end
          obj.p_output=p_output;
        end
      elseif (num_y>0)
        obj.p_output=obj.output(obj.p_t,obj.p_x,obj.p_u);
      end
    end
    
    function obj = addStateConstraints(obj,p_state_constraints)
      typecheck(p_state_constraints,'msspoly');
      if (any(match(obj.p_x,decomp(p_state_constraints))==0))
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
        obj.p_mass_matrix = p_mass_matrix;
      end
    end
    
    function obj = pullEmptyPolysFromMethods(obj)
      if (isempty(obj.p_dynamics) && obj.num_xc>0)
        obj.p_dynamics=obj.dynamics(obj.p_t,obj.p_x,obj.p_u);
      end
      if (isempty(obj.p_update) && obj.num_xd>0)
        obj.p_update=obj.update(obj.p_t,obj.p_x,obj.p_u);
      end
      if (isempty(obj.p_output) && obj.num_y>0)
        obj.p_output=obj.output(obj.p_t,obj.p_x,obj.p_u);
      end
    end
    
    % Implement default methods using msspoly vars explicitly
    function [xcdot,df] = dynamics(obj,t,x,u)
      % should only get here if constructor specified p_dynamics
      if (isempty(obj.p_dynamics)) error('p_dynamics is not defined.  how did you get here?'); end
      xcdot = double(subs(obj.p_dynamics,[obj.p_t;obj.p_x;obj.p_u],[t;x;u]));
      if (nargout>1)
        df = double(subs(diff(obj.p_dynamics,[obj.p_t;obj.p_x;obj.p_u]),[obj.p_t;obj.p_x;obj.p_u],[t;x;u]));
      end
      if (~isempty(obj.p_mass_matrix))
        e = double(subs(obj.p_mass_matrix,obj.p_x,x));
        xcdot = e\xcdot;
        if (nargout>1) 
          % e(x)xdot = f(x) => dexdotdx + e(x)*dxdotdx = dfdx
          %  where the columns of dexdotdx are dedxi*xdot
          nX = obj.num_xc; cellxcdot=cell(1,nX); [cellxcdot{:}]=deal(xcdot); 
          dexdotdx = reshape(double(subs(diff(obj.p_mass_matrix(:),obj.p_x),obj.p_x,x)),nX,[])*blkdiag(cellxcdot{:}); 
          df = e\(df - [zeros(nX,1),dexdotdx,zeros(nX,obj.num_u)]);
        end
      end
    end
    function [xdn,df] = update(obj,t,x,u)
      if (isempty(obj.p_update)) error('p_update is not defined.  how did you get here?'); end
      xdn = double(subs(obj.p_update,[obj.p_t;obj.p_x;obj.p_u],[t;x;u]));
      if (nargout>1)
        df = double(subs(diff(obj.p_update,[obj.p_t;obj.p_x;obj.p_u]),[obj.p_t;obj.p_x;obj.p_u],[t;x;u]));
      end
    end
    function [y,dy] = output(obj,t,x,u)
      if (isempty(obj.p_output)) error('p_output is not defined.  how did you get here?'); end
      if (nargin<4) p_u=[]; u=[]; else p_u=obj.p_u; end  % ok for systems without direct feedthrough
      y = double(subs(obj.p_output,[obj.p_t;obj.p_x;p_u],[t;x;u]));
      if (nargout>1)
        dy = double(subs(diff(obj.p_output,[obj.p_t;obj.p_x;p_u]),[obj.p_t;obj.p_x;obj.p_u],[t;x;u]));
      end
    end
    
    % todo: implement gradients (it's trivial)
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
    
    function sys = feedback(sys1,sys2,options)
      % try to keep feedback between polynomial systems polynomial.  else,
      % kick out to SmoothRobotLibSystem
      %
      % @option try_to_be_robotlibsystem set to false if you want to return
      % a simulink model. @default true
      %

      if (nargin<3) options=struct(); end
      if (~isfield(options,'try_to_be_robotlibsystem')) options.try_to_be_robotlibsystem=true; end
      
      if (sys1.num_xcon>0) error('not implemented yet. should be easy.'); end
      if (~isempty(sys1.p_mass_matrix)) error('not implemented yet, but should be easy (though not supported for every case)'); end
      
      sys=[];

      function y=doubleSafe(x)
        y=double(x);
        if (~isa(y,'double')) error('double failed'); end
      end

      function sys = makesys(satflags)
        p_dynamics = []; p_update=[]; p_output=[];

        sat_y1 = p_y1; 
        sat_y2 = p_y2; 
        if (nargin>0) 
          n = length(umin1);
          ind = find(satflags(1:n)<0);
          sat_y2(ind) = umin1(ind);  % note: umin2,etc is only defined inside the sat loop below.  yuck.
          ind = find(satflags(1:n)>0);
          sat_y2(ind) = umax1(ind);
        
          ind = find(satflags(n+1:end)<0);
          sat_y1(ind) = umin2(ind);
          ind = find(satflags(n+1:end)>0);
          sat_y1(ind) = umax2(ind);
        end
        
        if (sys1.getNumContStates()>0)
          p_dynamics=[p_dynamics;subss(sys1.p_dynamics,[sys1.p_x;sys1.p_u],[p_x1;sat_y2])];
        end
        if (sys2.getNumContStates()>0)
          p_dynamics=[p_dynamics;subss(sys2.p_dynamics,[sys2.p_x;sys2.p_u],[p_x2;sat_y1])];
        end
        if (sys1.getNumDiscStates()>0)
          p_update = [p_update;subss(sys1.p_update,[sys1.p_x;sys1.p_u],[p_x1;sat_y2])];
        end
        if (sys2.getNumDiscStates()>0)
          p_update = [p_update; subss(sys2.p_update,[sys2.p_x;sys2.p_u],[p_x2;sat_y1])];
        end
        if (sys1.getNumOutputs()>0)
          p_output = p_y1;
        end
        
        sys = PolynomialSystem(sys1.getNumContStates()+sys2.getNumContStates(),...
          sys1.getNumDiscStates()+sys2.getNumDiscStates(),...
          0, sys1.getNumOutputs(), false, sys1.isTI() & sys2.isTI(),...
          p_dynamics,p_update,p_output);
      end
        
        
      if (options.try_to_be_robotlibsystem && isa(sys2,'PolynomialSystem'))

        if (sys2.num_xcon>0) error('not implemented yet. should be easy.'); end
        if (~isempty(sys2.p_mass_matrix)) error('not implemented yet, but should be easy (though not supported for every case)'); end

        
        p_t = msspoly('t',1);

        p_x = msspoly('x',sys1.getNumStates()+sys2.getNumStates());
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
          p_y1 = subs(sys1.p_output,sys1.p_x,p_x1); % doesn't need u
          p_y2 = subss(sys2.p_output,[sys2.p_x;sys2.p_u],[p_x2;p_y1]);
        else % do sys2 first
          p_y2 = subs(sys2.p_output,sys2.p_x,p_x2); % doesn't need u
          p_y1 = subss(sys1.p_output,[sys1.p_x;sys1.p_u],[p_x1;p_y2]);
        end
        
        if (any(~isinf([sys1.umin;sys1.umax;sys2.umin;sys2.umax])))
          % if possible, create a polytopic polynomial system, else fail

          A=[]; b=[]; subsys={};
          
          umin1=sys1.umin; umax1=sys1.umax;
          sys1=setInputLimits(sys1,-inf,inf);
          
          umin2=sys2.umin; umax2=sys2.umax;
          sys2=setInputLimits(sys2,-inf,inf);
          
          subsys={feedback(sys1,sys2)};
          N = sys1.getNumInputs() + sys2.getNumInputs();
          satflags={zeros(N,1)};
          
          if (any(~isinf([umin1; umax1])))
            % found saturation.  check if the saturation is a linear
            % (or affine) function of the state
            
            if (deg(p_y2,p_t)>0 || deg(p_y2,p_x)>1)
              error('RobotLib:PolytopicRobotLibSystem:NotSupported','output of system 2 must be affine in x and independent of t and u');
            end
            y0=doubleSafe(subs(p_y2,p_x,double(0*p_x)));
            dydx=doubleSafe(diff(p_y2,p_x));
            
            % y0+dydx*x<=umin
            inds=find(umin1~=-inf);
            A = [A;-dydx(inds,:)];
            b = [b;y0(inds)-umin1(inds)];
            for i=inds
              for j=1:length(subsys)
                sf=satflags{j};
                sf(i)=-1;
                satflags={satflags{:},sf};
                subsys={subsys{:},makesys(sf)};
              end
            end
            
            % y0+dydx*x>=umax
            inds=find(umax1~=inf);
            A = [A;dydx(inds,:)];
            b = [b;umax1(inds)-y0(inds)];
            for i=inds
              for j=1:length(subsys)
                sf=satflags{j};
                sf(i)=1;
                satflags={satflags{:},sf};
                subsys={subsys{:},makesys(sf)};
              end
            end
            
          end
          
          if (any(~isinf([umin2; umax2])))
            if (deg(p_y1,p_t)>0 || deg(p_y1,p_x)>1)
              error('RobotLib:PolytopicRobotLibSystem:NotSupported','output of system 2 must be affine in x and independent of t and u');
            end
            y0=doubleSafe(subs(p_y1,p_x,double(0*p_x)));
            dydx=doubleSafe(diff(p_y1,p_x));
            
            % y0+dydx*x<=umin
            inds=find(umin2~=-inf);
            A = [A;-dydx(inds,:)];
            b = [b;y0(inds)-umin2(inds)];
            for i=inds
              for j=1:length(subsys)
                sf=satflags{j};
                sf(i+length(umin1))=-1;
                satflags={satflags{:},sf};
                subsys={subsys{:},makesys(sf)};
              end
            end
            
            % y0+dydx*x>=umax
            inds=find(umax2~=inf);
            A = [A;dydx(inds,:)];
            b = [b;umax2(inds)-y0(inds)];
            for i=inds
              for j=1:length(subsys)
                sf=satflags{j};
                sf(i+length(umin1))=1;
                satflags={satflags{:},sf};
                subsys={subsys{:},makesys(sf)};
              end
            end
          end
          
          sys = PolytopicPolynomialSystem(A,b,subsys);
          
        else
          % handle simple case
          sys = makesys();
        end
      end
      
      if (isempty(sys))
        sys = feedback@SmoothRobotLibSystem(sys1,sys2,options);
      end
    end
  end
    
end
