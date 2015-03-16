function traj = simulateWalking(r, walking_plan_data, options)
%NOTEST

typecheck(r, 'Atlas');
typecheck(walking_plan_data, 'QPLocomotionPlan');

import atlasControllers.*;

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options = applyDefaults(options, struct('use_mex', true,...
                                        'use_bullet', false,...
                                        'use_ik', false,...
                                        'use_angular_momentum', false,...
                                        'draw_button', true));
                                      
if ~isfield(options, 'v') || isempty(options.v)
  v = r.constructVisualizer;
  v.display_dt = 0.05;
else
  v = options.v;
  rmfield(options, 'v'); % don't pass the vis down to the controllers
end

x0 = r.getInitialState();
nq = getNumPositions(r);

T = walking_plan_data.comtraj.tspan(2)-0.001;

ctrl_data = QPControllerData(true,struct(...
  'acceleration_input_frame',atlasFrames.AtlasCoordinates(r),...
  'D',-0.89/9.81*eye(2),... % assumed COM height
  'Qy',eye(2),...
  'S',walking_plan_data.V.S,... % always a constant
  's1',walking_plan_data.V.s1,...
  's2',walking_plan_data.V.s2,...
  's1dot',fnder(walking_plan_data.V.s1,1),...
  's2dot',fnder(walking_plan_data.V.s2,1),...
  'x0',ConstantTrajectory([walking_plan_data.zmptraj.eval(T);0;0]),...
  'u0',ConstantTrajectory(zeros(2,1)),...
  'qtraj',x0(1:nq),...
  'comtraj',walking_plan_data.comtraj,...
  'link_constraints',walking_plan_data.link_constraints, ...
  'support_times',walking_plan_data.support_times,...
  'supports',walking_plan_data.supports,...
  'mu',walking_plan_data.mu,...
  'ignore_terrain',false,...
  'y0',walking_plan_data.zmptraj,...
  'plan_shift',zeros(3,1),...
  'constrained_dofs',[findPositionIndices(r,'arm');findPositionIndices(r,'back');findPositionIndices(r,'neck')]));

options.dt = 0.003;
options.debug = false;

if options.use_angular_momentum
  options.Kp_ang = 1.0; % angular momentum proportunal feedback gain
  options.W_kdot = 1e-5*eye(3); % angular momentum weight
end

if (options.use_ik)
  options.w_qdd = 0.001*ones(nq,1);
  % instantiate QP controller
  qp = QPController(r,{},ctrl_data,options);

  % feedback QP controller with atlas
  ins(1).system = 1;
  ins(1).input = 2;
  ins(2).system = 1;
  ins(2).input = 3;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(qp,cascade(r, haltBlock),[],[],ins,outs);
  clear ins;

  % feedback foot contact detector with QP/atlas
  options.use_lcm=false;
  fc = FootContactBlock(r,ctrl_data,options);
  ins(1).system = 2;
  ins(1).input = 1;
  sys = mimoFeedback(fc,sys,[],[],ins,outs);
  clear ins;  
  
  % feedback PD block
  pd = IKPDBlock(r,ctrl_data,options);
  ins(1).system = 1;
  ins(1).input = 1;
  sys = mimoFeedback(pd,sys,[],[],ins,outs);
  clear ins;

else
  
  options.Kp_foot = [100; 100; 100; 150; 150; 150];
  options.foot_damping_ratio = 0.5;
  options.Kp_pelvis = [0; 0; 150; 200; 200; 200];
  options.pelvis_damping_ratio = 0.6;
  options.Kp_q = 150.0*ones(r.getNumPositions(),1);
  options.q_damping_ratio = 0.6;

  % construct QP controller and related control blocks
  [qp,lfoot_controller,rfoot_controller,pelvis_controller,pd,options] = constructQPWalkingController(r,ctrl_data,options);

  % feedback QP controller with atlas
  ins(1).system = 1;
  ins(1).input = 2;
  ins(2).system = 1;
  ins(2).input = 3;
  ins(3).system = 1;
  ins(3).input = 4;
  ins(4).system = 1;
  ins(4).input = 5;
  ins(5).system = 1;
  ins(5).input = 6;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(qp,r,[],[],ins,outs);
  clear ins outs;
  
  % feedback foot contact detector with QP/atlas
  options.use_lcm=false;
  fc = FootContactBlock(r,ctrl_data,options);
  ins(1).system = 2;
  ins(1).input = 1;
  ins(2).system = 2;
  ins(2).input = 3;
  ins(3).system = 2;
  ins(3).input = 4;
  ins(4).system = 2;
  ins(4).input = 5;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(fc,sys,[],[],ins,outs);
  clear ins outs;  
  
  % feedback PD block
  ins(1).system = 1;
  ins(1).input = 1;
  ins(2).system = 2;
  ins(2).input = 2;
  ins(3).system = 2;
  ins(3).input = 3;
  ins(4).system = 2;
  ins(4).input = 4;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(pd,sys,[],[],ins,outs);
  clear ins outs;

  % feedback body motion control blocks
  ins(1).system = 2;
  ins(1).input = 1;
  ins(2).system = 2;
  ins(2).input = 3;
  ins(3).system = 2;
  ins(3).input = 4;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(lfoot_controller,sys,[],[],ins,outs);
  clear ins outs;

  ins(1).system = 2;
  ins(1).input = 1;
  ins(2).system = 2;
  ins(2).input = 3;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(rfoot_controller,sys,[],[],ins,outs);
  clear ins outs;

  ins(1).system = 2;
  ins(1).input = 1;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(pelvis_controller,sys,[],[],ins,outs);
  clear ins outs;
end

qt = QTrajEvalBlock(r,ctrl_data);
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qt,sys,[],[],[],outs);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T],x0,struct('gui_control_interface',true));

end
