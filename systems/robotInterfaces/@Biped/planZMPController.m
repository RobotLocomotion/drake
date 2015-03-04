function [c, V, comtraj, limp_height] = planZMPController(obj, zmptraj_or_comgoal, x0, options)
if nargin < 4
  options = struct();
end
options = applyDefaults(options, struct('pelvis_height_above_sole', []));

path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'ZMP'));

nq = obj.getNumPositions();
q0 = x0(1:nq);

kinsol = doKinematics(obj, q0);

if length(x0) == nq
  com = getCOM(obj, kinsol);
  options.com0 = com(1:2);
else
  assert(length(x0) == obj.getNumStates(), 'expected a full configuration or state vector');
  [com, J] = getCOM(obj, kinsol);
  comdot = J * x0((obj.getNumPositions()+1):end);
  options.com0 = com(1:2);
  options.comdot0 = comdot(1:2);
end

foot_pos = forwardKin(obj, kinsol, [obj.foot_frame_id.right, obj.foot_frame_id.left], [0;0;0]);
zfeet = mean(foot_pos(3,:));
if isempty(options.pelvis_height_above_sole)
  limp_height = com(3) - zfeet;
else
  limp_height = options.pelvis_height_above_sole;
end

if isa(zmptraj_or_comgoal, 'Trajectory')
  [c, V, comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(limp_height, zmptraj_or_comgoal, options);
elseif isnumeric(zmptraj_or_comgoal)
  limp = LinearInvertedPendulum(com(3)-zfeet);
  [c,V] = lqr(limp,zmptraj_or_comgoal);
  comtraj = zmptraj_or_comgoal;
else
  error('Wrong type. Expected a zmp trajectory or a desired com position in xy');
end
