function xtraj = simulate(obj,control,tspan,x0,visualizer)
% Integrates the (potentially closed-loop) dynamics forward.

typecheck(control,'Control');
typecheck(tspan,'double');
if (nargin<4 || isempty(x0))
  x0 = getInitialState(obj);
else
  typecheck(x0,'double');
end
if (nargin<5) visualizer = []; end

if (length(x0)~=obj.num_x) error('x0 is the wrong size'); end
if (obj.num_u ~= control.num_u) error('dynamics and control have a different number of inputs'); end
if (control.num_x~=0 && obj.num_x ~= control.num_x) error('dynamics and control have a different number of states'); end

if (control.control_dt~=0) error('sampled-data control is not re-implemented yet'); end
if (length(closed_loop_dynamics(tspan(1),x0))~=obj.num_x) error('the closed loop dynamics is not returning an xdot of the correct size'); end

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
%      u = control.control(t(end),x(:,end));  % run control in case there were any scopes
      closed_loop_dynamics(t(end),x(:,end));
      g_scope_enable = false;

      if (~isempty(visualizer)) visualizer.draw(t(end),x(:,end)); end
    end
    status=0;
  end

end
