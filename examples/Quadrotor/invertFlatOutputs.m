function [xtraj,utraj] = invertFlatOutputs(plant,ytraj,options)

% @param plant a quadrotor plant (either Quadrotor or QuadPlantPenn should
% work)
% @param ytraj a PPTrajectory in the DifferentiallyFlatOutputFrame
% @option zero_acceleration_tol threshold on the magnitude of the 
%                        acceleration vector below which the quad is
%                        considered stationary. @default 1e-8
%
% implementation based on 
% Minimum Snap Trajectory Generation and Control for Quadrotors
% by Daniel Mellinger and Vijay Kumar
% ICRA 2011

typecheck(plant,{'Quadrotor','QuadPlantPenn'});
typecheck(ytraj,'PPTrajectory');
assert(ytraj.getOutputFrame == DifferentiallyFlatOutputFrame);

if nargin<3, options=struct(); end
if ~isfield(options,'zero_acceleration_tol'), options.zero_acceleration_tol = 1e-8; end

breaks = getBreaks(ytraj);
ydtraj = fnder(ytraj);
yddtraj = fnder(ydtraj);
ydddtraj = fnder(yddtraj);
g = plant.getGravity;
zW = g/norm(g);
m = plant.getMass;
I = plant.getInertia;

  function x_or_u = extractStateOrInput(t,tf_output_state)
    y = ytraj.eval(t);
    yd = ydtraj.eval(t);
    ydd = yddtraj.eval(t);
    zB = ydd(1:3)-g;
    zBmag = norm(zB);
    if zBmag<options.zero_acceleration_tol, zB = [0;0;1]; else zB = zB/zBmag; end
    xC = [cos(y(4));sin(y(4));0];
    yB = cross(zB,xC);
    yB = yB/norm(yB);
    xB = cross(yB,zB);
    R = [xB,yB,zB];
    x = [y(1:3); rotmat2rpy(R); yd(1:3); zeros(3,1)];
    
    % compute velocities
    yddd = ydddtraj.eval(t);
    adot = yddd(1:3);
    h_omega = m*(adot - (zB'*adot)*zB)/zBmag;

    omegaBW = [ - h_omega'*yB; h_omega'*xB; yd(4)*zW'*zB ];
    x(10:12) = angularvel2rpydot(x(4:6), omegaBW);

    % now extract input
    % note: my u vector = kf*omega.^2 from mellinger 
%    omegadotBW = [ nan; nan; ydd(4)*zW'*zB ];
%    mellinger_u = [zBmag; I*omegadotBW + omegaBW'*(I*omegaBW)]

    if (tf_output_state)
      x_or_u = x;
    else
      x_or_u = zeros(4,1);
    end
  end

% todo: actually output a polynomial, not just a function handle
w = warning('off','Drake:FunctionHandleTrajectory');
xtraj = FunctionHandleTrajectory(@(t)extractStateOrInput(t,true),12,breaks);
warning(w);
xtraj = setOutputFrame(xtraj,getStateFrame(plant));

if nargout>1
  warning('inputs are set to zero for now.  coming soon.');
  w = warning('off','Drake:FunctionHandleTrajectory');
  utraj = FunctionHandleTrajectory(@(t)extractStateOrInput(t,false),4,breaks);
  warning(w);
  utraj = setOutputFrame(utraj,getInputFrame(plant));
end

end
