function [xtraj,utraj] = invertFlatOutputs(plant,ytraj)

% @param plant a quadrotor plant (either Quadrotor or QuadPlantPenn should
% work)
% @param ytraj a PPTrajectory in the DifferentiallyFlatOutputFrame

% note: assumes g = [0;0;-9.81]

typecheck(plant,'DrakeSystem');
typecheck(ytraj,'PPTrajectory');
assert(ytraj.getOutputFrame == DifferentiallyFlatOutputFrame);

breaks = getBreaks(ytraj);
ydtraj = fnder(ytraj);
yddtraj = fnder(ydtraj);
g = [0;0;-9.81];

  function x = extractState(t)
    y = ytraj.eval(t);
    ydd = yddtraj.eval(t);
    zB = ydd(1:3)-g;
    zBmag = norm(zB);
    if zBmag<1e-6, zB = [0;0;1]; else zB = zB/zBmag; end
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
