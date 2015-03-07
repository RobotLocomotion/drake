function runAtlasSimAsync(example_options)
%NOTEST 

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<1, example_options=struct(); end
example_options = applyDefaults(example_options, struct('use_mex', true,...
                                                        'use_bullet', false,...
                                                        'quiet', true,...
                                                        'terrain', RigidBodyFlatTerrain));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
options.ignore_self_collisions = true;
options.ignore_friction = true;
options.dt = 0.001;
options.terrain = example_options.terrain;
options.use_bullet = example_options.use_bullet;
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
xstar = r.resolveConstraints(xstar);
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.01;

x0 = xstar;

% Make our Atlas listen for ATLAS_COMMAND over LCM and publish EST_ROBOT_STATE
% command_to_effort = atlasControllers.AtlasCommandToEffortBlock(r);
% command_to_effort = command_to_effort.setInputFrame(drcFrames.AtlasInput(r));
% r = r.setInputFrame(drcFrames.AtlasInput(r));
% output_frame = drcFrames.AtlasState(r);
% output_frame.setMaxRate(2000);
% r = r.setOutputFrame(output_frame);

% sys = cascade(command_to_effort, r);
% sys = r;
sys = cascade(CommandReceiver(r), r);
sys = cascade(sys, StatePublisher(r));

output_select(1).system=1;
output_select(1).output=1;
v = v.setInputFrame(sys.getOutputFrame());
sys = mimoCascade(sys,v,[],[],output_select);

disp('sim starting');
runLCM(sys, x0, struct('tspan', [0, inf],...
                        'timekeeper', ''));

v.playback(ytraj, struct('slider', true));