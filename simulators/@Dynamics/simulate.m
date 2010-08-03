function xtraj = simulate(obj,control,tspan,x0)

typecheck(control,'Control');
typecheck(tspan,'double');
typecheck(x0,'double');

if (length(x0)~=obj.numStates) error('x0 is the wrong size'); end
if (obj.numInputs ~= control.numInputs) error('dynamics and control have a different number of inputs'); end
if (~isempty(control.numStates) && obj.numStates ~= control.numStates) error('dynamics and control have a different number of states'); end

%todo: check this somewhere else (wrap those guys?)
if (length(obj.umin)~=obj.numInputs || length(obj.umax)~=obj.numInputs) error('inputs limits are the wrong size'); end

if (control.controlDT~=0) error('sampled-data control is not re-implemented yet'); end

sol = obj.odesolver(@closed_loop_dynamics,tspan,x0,obj.odeoptions);
xtraj = ODESolTrajectory(sol);

  function xdot = closed_loop_dynamics(t,x)
    u = control.control(t,x);
    u = min(max(u,obj.umin),obj.umax);
    xdot = obj.dynamics(t,x,u);
  end

end
