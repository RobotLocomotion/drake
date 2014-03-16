function new_rbm = changeRootLink(rbm, link, xzy, rpy, options)
% Changes the root link of the kinematic tree. Note that the root link 
% is static in the world.
%
% @param rbm a RigidBodyManipulator object
% @param link the name or index of the link that will become the root link
% @param xyz location on link (in the link frame) for the new origin
% @param rpy orientation on link (in the link frame) for the new origin
% @option floating_base (default false)
%
% As always, any leaf nodes with zero inertia will be automatically 
% removed. This will have the intended effect of removing any 
% vestigial floating base joints from the original model.
%
% Example: attach the foot of a floating base manipulator to the
% ground (to be fleshed out with the details)
% rbm = addLink(rbm, 'ground');
% rbm = addJoint(rbm,'foot','ground', ...);
% rbm = changeRootLink(rbm,'ground');

new_rbm = rbm;
error('not implemented yet');