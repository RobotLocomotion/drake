function new_rbm = changeRootLink(rbm, link, xyz, rpy, options)
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

if isnumeric(link) && link>=0
  root = link;
elseif ischar(link)
  root = findLinkInd(new_rbm,link);
end

% basic algorithm:
%   update root node (parent + mass/kin properties)
%   todo_list = former parent;
%   while ~isempty(todo_list)
%      update next node (parent + mass/kin properties)
%      add former parent todo_list
%      don't actually need to update the children
% so I can actually get away with just iterating through the parents,
% instead of making a proper todo_list

parent_ind = 0;
current_body_ind = root;

while (true)
  current_body = getBody(new_rbm,current_body_ind);
  next_body_ind = current_body.parent;
  
  %% update body
  current_body.parent = parent_ind
  
  if (parent_ind == 0) 
    current_body.robotnum = 0;
    current_body.jointname = ''; 
    current_body.pitch = 0;
    current_body.floating = 0;
    current_body.Xtree = Xrotx(rpy(1))*Xroty(rpy(2))*Xrotz(rpy(3))*Xtrans(xyz);
    current_body.Ttree = [rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1]; 
  else
    old_child_new_parent_body = getBody(rbm,parent_ind);
    current_body.robotnum = old_child_new_parent_body.robotnum;
    current_body.jointname = old_child_new_parent_body.jointname;
    current_body.pitch = old_child_new_parent_body.pitch;
    current_body.floating = old_child_new_parent_body.floating;
    
    current_body.Xtree = inv(old_child_new_parent_body.Xtree);
    current_body.Ttree = inv(old_child_new_parent_body.Ttree);  
    
    current_body.X_joint_to_body = current_body.Xtree * old_child_new_parent_body.X_joint_to_body;
    current_body.T_body_to_joint = old_child_new_parent_body.T_body_to_joint*current_body.Ttree;
    
    % note: flip joint axis here to keep the joint direction the same (and
    % avoid flipping joint limits, etc)
    current_body.joint_axis = current_body.Ttree(1:3,:) * [-old_child_new_parent_body.joint_axis;1];

    current_body.damping = old_child_new_parent_body.damping; 
    current_body.coulomb_friction = old_child_new_parent_body.coulomb_friction; 
    current_body.static_friction = old_child_new_parent_body.static_friction;
    current_body.coulomb_window = old_child_new_parent_body.coulomb_window;
    current_body.joint_limit_min = old_child_new_parent_body.joint_limit_min;
    current_body.joint_limit_max = old_child_new_parent_body.joint_limit_max;
    current_body.effort_limit = old_child_new_parent_body.effort_limit;
    current_body.velocity_limit = old_child_new_parent_body.velocity_limit;
    current_body.has_position_sensor = old_child_new_parent_body.has_position_sensor;
  end
    
  new_rbm = setBody(new_rbm,current_body_ind,current_body);
  
  if (next_body_ind<1) break; end
  
  %% update indices for next iteration
  parent_ind = current_body_ind;
  current_body_ind = next_body_ind;
end

new_rbm = compile(new_rbm);
