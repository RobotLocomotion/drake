classdef DrakeSystemWGaussianNoise < StochasticDrakeSystem
  % This is a wrapper class which adds Gaussian process and 
  % measurement noise to an existing DrakeSystem.  It is 
  % useful, for instance, for noisy simulation of an
  % existing model.
  %
  % Given a drake_system passed to the constructor with
  %   xdot = fc(t,x,u)
  %   xdn = fd(t,x,u)
  %   y = g(t,x[,u])
  % the resulting system is 
  %   xdot = fc(t,x,u) + wc
  %   xdn = fd(t,x,u) + wd
  %   y = g(t,x[,u]) + v
  % where wc, wd, and v are Gaussian (band-limited) noise with 
  % covariances specified by dynamics_covariance, 
  % update_covariance, and output_covariance, respectively.
  
  methods
    function obj = DrakeSystemWGaussianNoise(drake_system,dynamics_covariance,update_covariance,output_covariance,ts_noise)
      % @param ts_noise time constant of the band-limited noise (see
      % documentation for the band-limited white noise block)
      
      typecheck(drake_system,'DrakeSystem');
      sizecheck(dynamics_covariance,[getNumContStates(drake_system),getNumContStates(drake_system)]);
      sizecheck(update_covariance,[getNumDiscStates(drake_system),getNumDiscStates(drake_system)]);
      sizecheck(output_covariance,[getNumOutputs(drake_system),getNumOutputs(drake_system)]);
      if isDT(drake_system), ts_noise=1; end  % shouldn't have any effect, so set a default value
      
      obj = obj@StochasticDrakeSystem(getNumContStates(drake_system),getNumDiscStates(drake_system),getNumInputs(drake_system),getNumOutputs(drake_system),isDirectFeedthrough(drake_system),isTI(drake_system),getNumStates(drake_system)+getNumOutputs(drake_system),ts_noise);
      obj.sys = drake_system;
      if (getNumContStates(obj)) obj.Wc = chol(dynamics_covariance+eps*eye(getNumContStates(obj))); end
      if (getNumDiscStates(obj)) obj.Wd = chol(update_covariance+eps*eye(getNumDiscStates(obj))); end
      if (getNumOutputs(obj)) obj.V = chol(output_covariance+eps*eye(getNumOutputs(obj))); end

      obj = setInputFrame(obj,getInputFrame(drake_system));
      obj = setStateFrame(obj,getStateFrame(drake_system));
      obj = setOutputFrame(obj,getOutputFrame(drake_system));
      obj = setSampleTime(obj,getSampleTime(drake_system));
    end
    
    function xcdot = stochasticDynamics(obj,t,x,u,w)
      xcdot = dynamics(obj.sys,t,x,u) + obj.Wc*w(1:getNumContStates(obj),:);
    end
    
    function xdn = stochasticUpdate(obj,t,x,u,w)
      xdn = update(obj.sys,t,x,u) + obj.Wd*w(getNumContStates(obj)+(1:getNumDiscStates(obj)),:);
    end
    
    function y = stochasticOutput(obj,t,x,u,w)
      if isDirectFeedthrough(obj)
        y = output(obj.sys,t,x,u) + obj.V*w(getNumStates(obj)+1:end,:);
      else
        y = output(obj.sys,t,x) + obj.V*w(getNumStates(obj)+1:end,:);
      end
    end    
  end
  
  properties
    sys;   % wrapped system
    Wc;    % continuous state process noise sqrt covariance
    Wd;    % discrete state process noise sqrt covariance
    V;     % measurement noise sqrt covariance
  end
end