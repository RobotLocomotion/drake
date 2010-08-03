function xtraj = simulate(obj,tspan,x0,control)
% Integrates the (potentially closed-loop) dynamics forward.

typecheck(tspan,'double');
typecheck(x0,'double');
if (nargin>3)
  typecheck(control,'Control');
else
  control = ConstantControl(getDefaultInput(obj));
end

if (length(x0)~=obj.num_states) error('x0 is the wrong size'); end
if (obj.num_inputs ~= control.num_inputs) error('dynamics and control have a different number of inputs'); end
if (control.numStates~=0 && obj.num_states ~= control.num_states) error('dynamics and control have a different number of states'); end

if (control.control_dt~=0) error('sampled-data control is not re-implemented yet'); end

sol = obj.ode_solver(@closed_loop_dynamics,tspan,x0,obj.ode_options);
xtraj = ODESolTrajectory(sol);

  function xdot = closed_loop_dynamics(t,x)
    u = control.control(t,x);
    u = min(max(u,obj.umin),obj.umax);
    xdot = obj.dynamics(t,x,u);
  end

end
