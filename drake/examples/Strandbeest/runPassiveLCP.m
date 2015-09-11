function runPassiveLCP

options.twoD = false;
options.floating = true;
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('Strandbeest.urdf',.01,options);
warning(w);
p = p.setTerrain(RigidBodyFlatTerrain()).compile();
v = p.constructVisualizer();

controller = AffineSystem([], [], [], [], [], [], [], zeros(1, p.getNumOutputs()), 300);
controller = controller.setInputFrame(p.getOutputFrame());
controller = controller.setOutputFrame(p.getInputFrame());
sys = feedback(p, controller);

% Add a visualizer
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

x0 = p.getInitialState();
x0(4:6) = 0;
x0(3) = 1.25;
% v.inspector();
xtraj = sys.simulate([0 5], x0, struct('gui_control_interface', true));

v.playback(xtraj, struct('slider', true));
