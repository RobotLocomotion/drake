function [xtraj,utraj] = invertFlatOutputs(plant,ytraj,options)

% @param plant a quadrotor plant (either Quadrotor or QuadPlantPenn should
% work)
% @param ytraj a PPTrajectory in the DifferentiallyFlatOutputFrame
% @option zero_acceleration_tol threshold on the magnitude of the 
%                        acceleration vector below which the quad is
%                        considered stationary. @default 1e-8

% note: assumes g = [0;0;-9.81]

typecheck(plant,'DrakeSystem');
typecheck(ytraj,'PPTrajectory');
assert(ytraj.getOutputFrame == DifferentiallyFlatOutputFrame);

if nargin<3, options=struct(); end
if ~isfield(options,'zero_acceleration_tol'), options.zero_acceleration_tol = 1e-8; end

breaks = getBreaks(ytraj);
ydtraj = fnder(ytraj);
yddtraj = fnder(ydtraj);
g = [0;0;-9.81];

  function x = extractState(t)
    y = ytraj.eval(t);
    ydd = yddtraj.eval(t);
    zB = ydd(1:3)-g;
    zBmag = norm(zB);
    if zBmag<options.zero_acceleration_tol, zB = [0;0;1]; else zB = zB/zBmag; end
    xC = [cos(y(4));sin(y(4));0];
    yB = cross(zB,xC);
    yB = yB/norm(yB);
    xB = cross(yB,zB);
    R = [xB,yB,zB];
    x = [y(1:3); rotmat2rpy(R); zeros(6,1)];
  end

% todo: actually output a polynomial, not just a function handle
warning('state velocities are set to zero for now.  coming soon.');
w = warning('off','Drake:FunctionHandleTrajectory');
xtraj = FunctionHandleTrajectory(@extractState,12,breaks);
warning(w);
xtraj = setOutputFrame(xtraj,getStateFrame(plant));

if nargout>1
  warning('inputs are set to zero for now.  coming soon.');
  utraj = ConstantTrajectory(zeros(4,1));
  utraj = setOutputFrame(utraj,getInputFrame(plant));
end

end
