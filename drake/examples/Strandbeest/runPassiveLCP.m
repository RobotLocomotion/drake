function runPassiveLCP

options.twoD = false;
options.floating = true;
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('Strandbeest.urdf',.01,options);
warning(w);
p = p.setTerrain(RigidBodyFlatTerrain()).compile();
v = p.constructVisualizer();

controller = AffineSystem([], [], [], [], [], [], [], zeros(1, p.getNumOutputs()), 10);
controller = controller.setInputFrame(p.getOutputFrame());
controller = controller.setOutputFrame(p.getInputFrame());
sys = feedback(p, controller);

sys = cascade(p, v);

x0 = p.getInitialState();
x0(4:6) = 0;
x0(3) = 1.5;
% v.inspector();
xtraj = sys.simulate([0 5], x0);

v.playback(xtraj, struct('slider', true, 'gui_control_interface', true));
