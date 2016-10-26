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
  root = findLinkId(new_rbm,link);
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
grandparent_ind = 0;
current_body_ind = root;

while (true)
  current_body = getBody(new_rbm,current_body_ind);
  next_body_ind = current_body.parent;
  
  %% update body
  current_body.parent = parent_ind;
  
  if (parent_ind == 0) 
    current_body.robotnum = 0;
    current_body.jointname = ''; 
    current_body.pitch = 0;
    current_body.floating = 0;
    current_body.Ttree = eye(4);

    T_old_body_to_new_body = inv([rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1]);
    
    current_body.joint_limit_min = [];
    current_body.joint_limit_max = [];
    current_body.effort_min = [];
    current_body.effort_max = [];
    current_body.velocity_limit = [];
  else
    old_child_new_parent_body = getBody(rbm,parent_ind);
    current_body.robotnum = old_child_new_parent_body.robotnum;
    current_body.jointname = old_child_new_parent_body.jointname;
    current_body.pitch = old_child_new_parent_body.pitch;
    current_body.floating = old_child_new_parent_body.floating;

    T_old_body_to_new_body = inv(old_child_new_parent_body.Ttree);
    if (grandparent_ind == 0) % then my parent is the root
      current_body.Ttree = inv([rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),xyz; 0,0,0,1]); 
    else
      grandparent_old_body = getBody(rbm,grandparent_ind);
      current_body.Ttree = inv(grandparent_old_body.Ttree);
    end
    
    % note: flip joint axis here to keep the joint direction the same (and
    % avoid flipping joint limits, etc)
    current_body.joint_axis = T_old_body_to_new_body(1:3,1:3) * (-old_child_new_parent_body.joint_axis);
    
    current_body.damping = old_child_new_parent_body.damping; 
    current_body.coulomb_friction = old_child_new_parent_body.coulomb_friction; 
    current_body.static_friction = old_child_new_parent_body.static_friction;
    current_body.coulomb_window = old_child_new_parent_body.coulomb_window;
    current_body.joint_limit_min = old_child_new_parent_body.joint_limit_min;
    current_body.joint_limit_max = old_child_new_parent_body.joint_limit_max;
    current_body.effort_min = old_child_new_parent_body.effort_min;
    current_body.effort_max = old_child_new_parent_body.effort_max;
    current_body.velocity_limit = old_child_new_parent_body.velocity_limit;
    current_body.has_position_sensor = old_child_new_parent_body.has_position_sensor;
  end
    
  %Check before running setInertial() that the body doesn't have added-mass
  %coefficients (I haven't written up the welding support for that yet - JSI)
  if ~valuecheck(current_body.Iaddedmass,zeros(6,6));
      error('Welding with added-mass coefficients is not supported yet');
  end
  
  % todo: consider moving these into RigidBody.updateTransform, but only if
  % it gets *everything* correct
  AdT_new_body_to_old_body = transformAdjoint(homogTransInv(T_old_body_to_new_body));
  I_in_new_body = AdT_new_body_to_old_body'*current_body.I*AdT_new_body_to_old_body;
  current_body = setInertial(current_body, I_in_new_body);

  for i=1:length(current_body.visual_geometry)
    current_body.visual_geometry{i}.T = current_body.visual_geometry{i}.T*T_old_body_to_new_body;
  end
  for i=1:length(current_body.collision_geometry)
    current_body.collision_geometry{i}.T = current_body.collision_geometry{i}.T*T_old_body_to_new_body;
  end

  for j=1:length(rbm.loop)
    new_rbm.loop(j) = updateBodyCoordinates(rbm.loop(j),current_body_ind,T_old_body_to_new_body);
  end
  for j=1:length(rbm.sensor)
    new_rbm.sensor{j} = updateBodyCoordinates(rbm.sensor{j},current_body_ind,T_old_body_to_new_body);
  end
  for j=1:length(rbm.force)
    new_rbm.force{j} = updateBodyCoordinates(rbm.force{j},current_body_ind,T_old_body_to_new_body);
  end
  for j=1:length(rbm.frame)
    new_rbm.frame(j) = updateBodyCoordinates(rbm.frame(j),current_body_ind,T_old_body_to_new_body);
  end
  
  if (getNumInputs(new_rbm))
    % if there was an actuator attached to my (new) parent, it now needs to
    % be attached to me.
    actuator_inds = [rbm.actuator.joint]==parent_ind;
    for a=find(actuator_inds)
      new_rbm.actuator(a).joint=current_body_ind;
    end
  end
  
  new_rbm = setBody(new_rbm,current_body_ind,current_body);
  
  if (next_body_ind<1), break; end
  
  %% update indices for next iteration
  grandparent_ind = parent_ind;
  parent_ind = current_body_ind;
  current_body_ind = next_body_ind;
end

new_rbm = compile(new_rbm);

% add coordinate transforms to go back and forth between the old and new
% frames
if isempty(findTransform(getStateFrame(rbm),getStateFrame(new_rbm)))
  addProjectionTransformByCoordinateNames(getStateFrame(rbm),getStateFrame(new_rbm));
  addProjectionTransformByCoordinateNames(getStateFrame(new_rbm),getStateFrame(rbm));
end

if isempty(findTransform(getInputFrame(rbm),getInputFrame(new_rbm)))
  addProjectionTransformByCoordinateNames(getInputFrame(rbm),getInputFrame(new_rbm));
  addProjectionTransformByCoordinateNames(getInputFrame(new_rbm),getInputFrame(rbm));
end

if isempty(findTransform(getOutputFrame(rbm),getOutputFrame(new_rbm)))
  addProjectionTransformByCoordinateNames(getOutputFrame(rbm),getOutputFrame(new_rbm));
  addProjectionTransformByCoordinateNames(getOutputFrame(new_rbm),getOutputFrame(rbm));
end
