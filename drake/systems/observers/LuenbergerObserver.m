classdef LuenbergerObserver < Observer
  % A potentially nonlinear observer with linear (possibly 
  % time-varying) observer gains
  % \begin{gather*} 
  %   \hat{xdot}_c(t) = f_c(t,\hat{x},u) + L_c(t) ( y - \hat{y} ) \\
  %   \hat{x}_d[n+1] = f_d(t,\hat{x},u) + L_d(t) (y - \hat{y} ) \\
  %   \hat{y} = g(t,\hat{x},u) \\
  %   y = \hat{x}
  % \end{gather*}
  
  methods
    function obj = LuenbergerObserver(sys_to_observe,Lc,Ld)
      % Construct a Luenberger observer given the observer gains.
      %
      % @param Lc continuous state observer gain matrix (can be a
      % trajectory).  If Lc is a vector, then Lc = diag(Lc)
      %
      % @param Ld discrete state observer gain matrix (can be a
      % trajectory).  If Ld is a vector, then Ld = diag(Ld)

      if isa(sys_to_observe,'HybridDrakeSystem')
        error('Drake:ObserverSystem:NoHybridSupport','hybrid systems not implemented yet.');
      end
      
      tiflag = isnumeric(Lc) && isnumeric(Ld) && isTI(sys_to_observe);
      num_xc = getNumContStates(sys_to_observe);
      num_xd = getNumDiscStates(sys_to_observe);
      num_y = getNumOutputs(sys_to_observe);
      
      if isempty(Lc)
        Lc = sparse(num_xc,num_y);
      elseif isvector(Lc)
        Lc = diag(Lc);
      end
      
      sizecheck(Lc,[num_xc,num_y]);
      if ~tiflag
        if isnumeric(Lc), Lc = ConstantTrajectory(Lc); end
        typecheck(Lc,'Trajectory');
      end
      
      if isempty(Ld)
        Ld = sparse(num_xd,num_y);
      elseif isvector(Ld)
        Ld = diag(Ld);
      end
      
      sizecheck(Ld,[num_xd,num_y]);
      if ~tiflag
        if isnumeric(Ld), Ld = ConstantTrajectory(Ld); end
        typecheck(Ld,'Trajectory');
      end
      
      
      obj = obj@Observer(sys_to_observe,...
        getNumContStates(sys_to_observe),...
        getNumDiscStates(sys_to_observe),...
        false,...
        tiflag);
      
      obj = setStateFrame(obj,getStateFrame(sys_to_observe));
      
      obj.Lc = Lc;
      obj.Ld = Ld;
    end
    
    function xcdot = dynamics(obj,t,x,uy)
      % Continuous time observer update
      [u,y] = splitCoordinates(getInputFrame(obj),uy);
      yhat = output(obj.forward_model,t,x,u);
      
      xcdot = dynamics(obj.forward_model,t,x,u);
      
      if (isTI(obj))
        xcdot = xcdot + obj.Lc*(y-yhat);
      else
        xcdot = xcdot + eval(obj.Lc,t)*(y-yhat);
      end
    end
    
    function xdn = update(obj,t,x,u)
      % Discrete time observer update
      [u,y] = splitCoordinates(getInputFrame(obj),uy);
      yhat = output(obj.forward_model,t,x,u);
      
      xdn = update(obj.forward_model,t,x,u);
      xdn = xdn + obj.Ld*(y-yhat);
      if (isTI(obj))
        xdn = xdn + obj.Ld*(y-yhat);
      else
        xdn = xdn + eval(obj.Ld,t)*(y-yhat);
      end
    end
    
    function y = output(obj,t,x,u)
      % Returns the estimated state as the output
      y = x;
    end
    
    
    function sys = extractAffineSystem(obj)
      % When the forward model is an affine system, the observer is 
      % also an affine system.  Use this method to extract that explicit
      % structure.
      
      if typecheck(obj.forward_model,'AffineSystem');
        sys = AffineSystem([obj.forward_model.Ac - obj.Lc*obj.forward_model.C],...
          [obj.forward_model.Bc - obj.Lc*obj.forward_model.D, obj.Lc],...
          obj.forward_model.xcdot0,...
          [obj.forward_model.Ad - obj.Ld*obj.forward_model.C],...
          [obj.forward_model.Bd - obj.Ld*obj.forward_model.D, obj.Ld],...
          obj.forward_model.xdn0,...
          obj.forward_model.C,...
          obj.forward_model.D);
        
        sys = setInputFrame(sys,obj.getInputFrame());
        sys = setStateFrame(sys,obj.getStateFrame());
        sys = setOutputFrame(sys,obj.getOutputFrame());
        
        sys = setSampleTime(sys,obj.getSampleTime);
      else
        % otherwise kick out to the more general (but slightly 
        % less efficient) version
        sys = extractAffineSystem@DrakeSystem(obj);
      end
    end
    
  end
  
    
  properties
    Lc;   % continuous state observer gain matrix
    Ld;   % discrete state observer gain matrix
  end
end
  