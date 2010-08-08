function xtraj = simulate(obj,control,tspan,x0)
% Integrates the (potentially closed-loop) dynamics forward.

typecheck(control,'Control');
typecheck(tspan,'double');
if (nargin>3) typecheck(x0,'double'); else x0 = getInitialState(obj); end

if (length(x0)~=obj.num_states) error('x0 is the wrong size'); end
if (obj.num_inputs ~= control.num_inputs) error('dynamics and control have a different number of inputs'); end
if (control.num_states~=0 && obj.num_states ~= control.num_states) error('dynamics and control have a different number of states'); end

if (control.control_dt~=0) error('sampled-data control is not re-implemented yet'); end
if (length(closed_loop_dynamics(tspan(1),x0))~=obj.num_states) error('the closed loop dynamics is not returning an xdot of the correct size'); end

global g_scope_enable;
sebk = g_scope_enable;
g_scope_enable = false;

options = obj.ode_options;
options = odeset(options,'OutputFcn',@outfun);

sol = obj.ode_solver(@closed_loop_dynamics,tspan,x0,options);
xtraj = ODESolTrajectory(sol);

g_scope_enable = sebk;

  function xdot = closed_loop_dynamics(t,x)
    u = control.control(t,x);
    u = min(max(u,obj.umin),obj.umax);
    xdot = obj.dynamics(t,x,u);
  end

  function status = outfun(t,x,flag)
    if (isempty(flag))
      g_scope_enable = true;
      u = control.control(t(end),x(:,end));  % run control in case there were any scopes
      g_scope_enable = false;
    end
    status=0;
  end

end
