function model=addRobotFromURDF(model,urdf_filename,xyz,rpy,options)
% Parses URDF
%
% @param urdf_filename filename of file to parse
%
% @options floating boolean where true means that a floating joint is
% automatically added to the root. @default false
% @options inertial boolean where true means parse dynamics parameters,
% false means skip them.  @default true
% @options visual boolean where true means parse graphics parameters, false
% means skip them.  @default true
% @options visual_geometry boolean where true means to extract the
% points from the visual geometries (might be very dense for meshes).
% Useful for extracting the 2D geometries later.  @default false
% @ingroup URDF Parsing

if (nargin<3 || isempty(xyz)), xyz = zeros(3,1); end
if (nargin<4 || isempty(rpy)), rpy = zeros(3,1); end

if (nargin<5), options = struct(); end
if (~isfield(options,'floating')), options.floating = ''; end  % no floating base
if isnumeric(options.floating) || islogical(options.floating)
  if (options.floating)
    options.floating = 'rpy';
  else
    options.floating = '';
  end
end
if (~isfield(options,'inertial')), options.inertial = true; end
if (~isfield(options,'visual')), options.visual = true; end
if (~isfield(options,'collision')), options.collision = true; end
if (~isfield(options,'collision_meshes')), options.collision_meshes = true; end
if (~isfield(options,'sensors')), options.sensors = true; end
if (~isfield(options,'visual_geometry')), options.visual_geometry = false; end
if (~isfield(options,'namesuffix')), options.namesuffix = ''; end
if (~isfield(options,'inertia_error')), options.inertia_error = 0.0; end
if (~isfield(options,'damping_error')), options.damping_error = 0.0; end
if (~isfield(options,'ignore_friction')), options.ignore_friction = false; end

%disp(['Parsing ', urdf_filename]);
urdf_filename = GetFullPath(urdf_filename);
[options.urdfpath,name,ext] = fileparts(urdf_filename);

urdf = xmlread(urdf_filename);

robot = urdf.getElementsByTagName('robot').item(0);
if isempty(robot)
  robot = urdf.getElementsByTagName('model').item(0);
end

if isempty(robot)
  error('there are no robots in this urdf file');
end

model = parseRobot(model,robot,xyz,rpy,options);

model.urdf = vertcat(model.urdf, urdf_filename);
model.dirty = true;

model = compile(model);  % ideally this would happen on entry into any function...

end



function model=parseRobot(model,node,xyz,rpy,options)
% Constructs a robot from a URDF XML node

%disp(['Parsing robot ', char(node.getAttribute('name')), ' from URDF file...']);
robotname = char(node.getAttribute('name'));
robotname = regexprep(robotname, '\.', '_', 'preservecase');
robotname = [robotname,options.namesuffix];

model.name = [model.name, {robotname}];
robotnum = length(model.name);

materials = node.getElementsByTagName('material');
for i=0:(materials.getLength()-1)
  [~,options] = model.parseMaterial(materials.item(i),options);
end

parameters = node.getElementsByTagName('parameter');
for i=0:(parameters.getLength()-1)
  model = parseParameter(model,robotnum,parameters.item(i),options);
end
% set the parameter frame immediately so that I can use the indices in my
% parameter parsing
[model,paramframe,pval] = constructParamFrame(model);
if ~isequal_modulo_transforms(paramframe,getParamFrame(model)) % let the previous handle stay valid if possible
  model = setParamFrame(model,paramframe);
end

links = node.getElementsByTagName('link');
for i=0:(links.getLength()-1)
  model = parseLink(model,robotnum,links.item(i),options);
end

collision_filter_groups = node.getElementsByTagName('collision_filter_group');
for i=0:(collision_filter_groups.getLength()-1)
  model = parseCollisionFilterGroup(model,robotnum,collision_filter_groups.item(i),options);
end

joints = node.getElementsByTagName('joint');
for i=0:(joints.getLength()-1)
  model = parseJoint(model,robotnum,joints.item(i),options);
end

loopjoints = node.getElementsByTagName('loop_joint');
for i=0:(loopjoints.getLength()-1)
  [model,loop] = RigidBodyLoop.parseURDFNode(model,robotnum,loopjoints.item(i),options);
  model.loop=[model.loop,loop];
end

forces = node.getElementsByTagName('force_element');
for i=0:(forces.getLength()-1)
  model = parseForceElement(model,robotnum,forces.item(i),options);
end

transmissions = node.getElementsByTagName('transmission');
for i=0:(transmissions.getLength()-1)
  model = parseTransmission(model,robotnum,transmissions.item(i),options);
end

gazebos = node.getElementsByTagName('gazebo');
for i=0:(gazebos.getLength()-1)
  model = parseGazebo(model,robotnum,gazebos.item(i),options);
end

frames = node.getElementsByTagName('frame');
for i=0:(frames.getLength()-1)
  model = parseFrame(model,robotnum,frames.item(i),options);
end

cables = node.getElementsByTagName('cable');
for i=0:(cables.getLength()-1)
  model = parseCable(model,robotnum,cables.item(i),options);
end

% weld the root link of this robot to the world link
% or some other link if specified in options
if (isfield(options, 'weld_to_link'))
  weldLink = options.weld_to_link;
else
  weldLink = 1; % world link
end
ind = find([model.body.parent]<1);
rootlink = ind([model.body(ind).robotnum]==robotnum);

for i=1:length(rootlink)
  if (~isempty(options.floating))
    model = addFloatingBase(model,weldLink,rootlink(i),xyz,rpy,options.floating);
  else
    model = addJoint(model,'weld','fixed',weldLink,rootlink(i),xyz,rpy);
  end
end

% finish parameter parsing
model = applyToAllRigidBodyElements(model,'bindParams',model,pval);

end

function model=parseLink(model,robotnum,node,options)

ignore = char(node.getAttribute('drake_ignore'));
if strcmp(lower(ignore),'true')
  return;
end

body = RigidBody();
body.robotnum = robotnum;

body.linkname=char(node.getAttribute('name'));
body.linkname=regexprep(body.linkname, '[\[\]\\\/\.]+', '_', 'preservecase');

% Checks if the link is named "world". If it is, throw an exception since this
% feature is not supported yet.
% TODO(liangfok): Modify this URDF parser to support URDFs with world links.
if strcmp(body.linkname, 'world')
  fprintf('Detected world link in URDF. Throwing exception.\n');
  error('Drake:WorldLinkInURDFModel',...
    'World link specified in URDF. This is currently not supported.')
end

if (options.inertial && node.getElementsByTagName('inertial').getLength()>0)
  body = parseInertial(body,node.getElementsByTagName('inertial').item(0),model,options);
end

if (options.visual && node.getElementsByTagName('visual').getLength()>0)
  visualItem = 0;
  while(~isempty(node.getElementsByTagName('visual').item(visualItem)))
    body = parseVisual(body,node.getElementsByTagName('visual').item(visualItem),model,options);
    visualItem = visualItem+1;
  end
end

if options.collision && node.getElementsByTagName('collision').getLength()>0
  collisionItem = 0;
  while(~isempty(node.getElementsByTagName('collision').item(collisionItem)))
    body = parseCollision(body,node.getElementsByTagName('collision').item(collisionItem),model,options);
    collisionItem = collisionItem+1;
  end
end

if options.sensors && node.getElementsByTagName('sensor').getLength()>0
  sensorItem = 0;
  while(~isempty(node.getElementsByTagName('sensor').item(sensorItem)))
    model = parseSensor(model,robotnum,node.getElementsByTagName('sensor').item(sensorItem),numel(model.body)+1,options);
    sensorItem = sensorItem+1;
  end
end

model.body=[model.body,body];
end

function model = parseSensor(model,robotnum,node,body_ind,options)

switch char(node.getAttribute('type'))
  case 'imu'
    model = addSensor(model,RigidBodyInertialMeasurementUnit.parseURDFNode(model,robotnum,node,body_ind,options));
  otherwise
    error(['sensor element type ',type,' not supported (yet?)']);
end

end

function model = parseCollisionFilterGroup(model,robotnum,node,options)

ignore = char(node.getAttribute('drake_ignore'));
if strcmpi(ignore,'true')
  return;
end
collision_fg_name = char(node.getAttribute('name'));
if isKey(model.collision_filter_groups,collision_fg_name)
  error('RigidBodyManipulator:parseCollisionFilterGroup:repeated_collision_fg_name', ...
    ['A collision filter group with the collision_fg_name %s already exists in this '...
    'RigidBodyManipulator'], collision_fg_name);
end

model.collision_filter_groups(collision_fg_name) = CollisionFilterGroup();

members = node.getElementsByTagName('member');
if members.getLength()>0
  members_cell = cell(1,members.getLength());
  for i=0:(members.getLength()-1)
    members_cell{i+1} = char(members.item(i).getAttribute('link'));
  end
  model = addLinksToCollisionFilterGroup(model,members_cell,collision_fg_name,robotnum);
end

ignored_collision_fgs = node.getElementsByTagName('ignored_collision_filter_group');
if ignored_collision_fgs.getLength()>0
  ignored_collision_fgs_cell = cell(1,ignored_collision_fgs.getLength());
  for i=0:(ignored_collision_fgs.getLength()-1)
    ignored_collision_fgs_cell{i+1} = char(ignored_collision_fgs.item(i).getAttribute('collision_filter_group'));
  end
  model = addToIgnoredListOfCollisionFilterGroup(model,ignored_collision_fgs_cell,collision_fg_name);
end

end

function model=parseJoint(model,robotnum,node,options)

ignore = char(node.getAttribute('drake_ignore'));
if strcmp(lower(ignore),'true')
  return;
end

parentNode = node.getElementsByTagName('parent').item(0);
if isempty(parentNode) % then it's not the main joint element.  for instance, the transmission element has a joint element, too
  return
end
parent = findLinkId(model,char(parentNode.getAttribute('link')),robotnum);

childNode = node.getElementsByTagName('child').item(0);
child = findLinkId(model,char(childNode.getAttribute('link')),robotnum);

name = char(node.getAttribute('name'));
type = char(node.getAttribute('type'));
xyz=zeros(3,1); rpy=zeros(3,1);
origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
if ~isempty(origin)
  if origin.hasAttribute('xyz')
    xyz = reshape(parseParamString(model,robotnum,char(origin.getAttribute('xyz'))),3,1);
  end
  if origin.hasAttribute('rpy')
    rpy = reshape(parseParamString(model,robotnum,char(origin.getAttribute('rpy'))),3,1);
  end
end
axis=[1;0;0];  % default according to URDF documentation
axisnode = node.getElementsByTagName('axis').item(0);
if ~isempty(axisnode)
  if axisnode.hasAttribute('xyz')
    axis = reshape(parseParamString(model,robotnum,char(axisnode.getAttribute('xyz'))),3,1);
    axis = axis/(norm(axis)+eps); % normalize
  end
end
damping=0;
coulomb_friction=0;
static_friction=0;
coulomb_window=eps;
dynamics = node.getElementsByTagName('dynamics').item(0);
if ~isempty(dynamics)
  if dynamics.hasAttribute('damping')
    damping = parseParamString(model,robotnum,char(dynamics.getAttribute('damping')));
  end
  if ~options.ignore_friction && dynamics.hasAttribute('friction')
    coulomb_friction = parseParamString(model,robotnum,char(dynamics.getAttribute('friction')));
    if coulomb_friction < 0
      error('RigidBodyManipulator: coulomb_friction must be >= 0');
    end
  end
  if ~options.ignore_friction && dynamics.hasAttribute('stiction')
    warning('RigidBodyManipulator:  stiction is not supported yet.');
    static_friction = parseParamString(model,robotnum,char(dynamics.getAttribute('stiction')));
    if static_friction < 0
      error('RigidBodyManipulator: static_friction must be >= 0');
    end
  end
  if ~options.ignore_friction && dynamics.hasAttribute('coulomb_window')
    coulomb_window = parseParamString(model,robotnum,char(dynamics.getAttribute('coulomb_window')));
    if coulomb_window <= 0
      error('RigidBodyManipulator: coulomb_window must be > 0');
    end
  end
end

% add noise to damping
if ~isnumeric(damping) && options.damping_error
  warning('damping error not supported for parameterized values (yet)');
end
if isnumeric(damping)
  damping = max(0,(1+options.damping_error*randn())*damping);
end

joint_limit_min=-inf;
joint_limit_max=inf;
effort_min=-inf;
effort_max=inf;
velocity_limit=inf;
limits = node.getElementsByTagName('limit').item(0);
if ~isempty(limits)
  if limits.hasAttribute('lower')
    joint_limit_min = parseParamString(model,robotnum,char(limits.getAttribute('lower')));
  end
  if limits.hasAttribute('upper');
    joint_limit_max = parseParamString(model,robotnum,char(limits.getAttribute('upper')));
  end
  if limits.hasAttribute('effort');
    if ~isfield(options,'ignore_effort_limits') || ~options.ignore_effort_limits
      effort = parseParamString(model,robotnum,char(limits.getAttribute('effort')));
      effort_min = min(-effort,effort); % just in case someone puts the min effort in the URDF
      effort_max = max(-effort,effort);
    end
  end
  if limits.hasAttribute('effort_min');
    if ~isfield(options,'ignore_effort_limits') || ~options.ignore_effort_limits
      effort_min = parseParamString(model,robotnum,char(limits.getAttribute('effort_min')));
    end
  end
  if limits.hasAttribute('effort_max');
    if ~isfield(options,'ignore_effort_limits') || ~options.ignore_effort_limits
      effort_max = parseParamString(model,robotnum,char(limits.getAttribute('effort_max')));
    end
  end
  if limits.hasAttribute('velocity');
    warnOnce(model.warning_manager,'Drake:RigidBodyManipulator:UnsupportedVelocityLimits','RigidBodyManipulator: velocity limits are not supported yet');
    velocity_limit = parseParamString(model,robotnum,char(limits.getAttribute('velocity')));
  end
end

limits = struct();
limits.joint_limit_min = joint_limit_min;
limits.joint_limit_max = joint_limit_max;
limits.effort_min = effort_min;
limits.effort_max = effort_max;
limits.velocity_limit = velocity_limit;

name=regexprep(name, '\.', '_', 'preservecase');
model = addJoint(model,name,type,parent,child,xyz,rpy,axis,damping,coulomb_friction,static_friction,coulomb_window,limits);

if node.hasAttribute('has_position_sensor')
  model.body(child).has_position_sensor = str2num(char(node.getAttribute('has_position_sensor')));
end
end


function model = parseForceElement(model,robotnum,node,options)

name = char(node.getAttribute('name'));
name = regexprep(name, '\.', '_', 'preservecase');

fe = [];
childNodes = node.getChildNodes();
elnode = node.getElementsByTagName('linear_spring_damper').item(0);
if ~isempty(elnode)
  [model,fe] = RigidBodySpringDamper.parseURDFNode(model,name,robotnum,elnode,options);
end

elnode = node.getElementsByTagName('torsional_spring').item(0);
if ~isempty(elnode)
  [model,fe] = RigidBodyTorsionalSpring.parseURDFNode(model,name,robotnum,elnode,options);
end

elnode = node.getElementsByTagName('wing').item(0);
if ~isempty(elnode)
  [model,fe] = RigidBodyWing.parseURDFNode(model,name,robotnum,elnode,options);
end

elnode = node.getElementsByTagName('wing_with_control_surface').item(0);
if ~isempty(elnode)
  [model,fe] = RigidBodyWingWithControlSurface.parseURDFNode(model,name,robotnum,elnode,options);
end

elnode = node.getElementsByTagName('bluff_body').item(0);
if ~isempty(elnode)
  [model,fe] = RigidBodyBluffBody.parseURDFNode(model,name,robotnum,elnode,options);
end

elnode = node.getElementsByTagName('thrust').item(0);
if ~isempty(elnode)
  [model,fe] = RigidBodyThrust.parseURDFNode(model,name,robotnum,elnode,options);
end

elnode = node.getElementsByTagName('added_mass').item(0);
if ~isempty(elnode)
  [model,fe] = RigidBodyAddedMass.parseURDFNode(model,name,robotnum,elnode,options);
end

elnode = node.getElementsByTagName('buoyancy').item(0);
if ~isempty(elnode)
  [model,fe] = RigidBodyBuoyant.parseURDFNode(model,name,robotnum,elnode,options);
end

elnode = node.getElementsByTagName('propellor').item(0);
if ~isempty(elnode)
  [model,fe] = RigidBodyPropellor.parseURDFNode(model,name,robotnum,elnode,options);
end



if ~isempty(fe)
  if iscell(fe)
    model.force = {model.force{:} fe{:}};
  else
    model.force{end+1} = fe;
  end
end
end

function model = parseParameter(model,robotnum,node,options)
  name = char(node.getAttribute('name'));    % mandatory
  model.param_db{robotnum}.(name).value = str2num(char(node.getAttribute('value')));  % mandatory
  n = char(node.getAttribute('lb'));  % optional
  if isempty(n), model.param_db{robotnum}.(name).lb = -inf; else model.param_db{robotnum}.(name).lb = str2num(n); end
  n = char(node.getAttribute('ub'));  % optional
  if isempty(n), model.param_db{robotnum}.(name).ub = inf; else model.param_db{robotnum}.(name).ub = str2num(n); end
end

function model = parseGazebo(model,robotnum,node,options)
ref = char(node.getAttribute('reference'));
if ~isempty(ref)
  body_ind = findLinkId(model,ref,robotnum,-1);
  if body_ind>0
    grav = node.getElementsByTagName('turnGravityOff').item(0);
    if ~isempty(grav)
      val='';
      if grav.hasAttribute('value')
        val = grav.getAttribute('value');
      elseif grav.hasChildNodes
        val = grav.getChildNodes.item(0).getNodeValue();
      end
      if strcmpi(val,'true')
        model.body(body_ind).gravity_off = true;
      end
    end
  end
  %        joint = findJoint(model,ref,robotnum,false)
  %        if ~isempty(joint)
  %          todo: parse initial conditions here?
  %        end
end

end

function model = parseFrame(model,robotnum,node,options)
  name = char(node.getAttribute('name'));    % mandatory
  link = findLinkId(model,char(node.getAttribute('link')),robotnum);
  xyz=zeros(3,1); rpy=zeros(3,1);
  if node.hasAttribute('xyz')
    xyz = reshape(parseParamString(model,robotnum,char(node.getAttribute('xyz'))),3,1);
  end
  if node.hasAttribute('rpy')
    rpy = reshape(parseParamString(model,robotnum,char(node.getAttribute('rpy'))),3,1);
  end

  % todo: make sure frame names are unique?

  model.frame = vertcat(model.frame,RigidBodyFrame(link,xyz,rpy,name));
end

function model = parseCable(model,robotnum,node,options)
  name = char(node.getAttribute('name'));
  if isempty(name), name='cable'; end
  min_length = parseParamString(model,robotnum,char(node.getAttribute('min_length')));
  max_length = parseParamString(model,robotnum,char(node.getAttribute('max_length')));

  cable_length_function = drakeFunction.kinematic.CableLength(model,name);

  children = node.getChildNodes;
  for i=0:(children.getLength()-1)
    this_node = children.item(i);
    switch(char(getNodeName(this_node)))
      case {'terminator','pulley'}
        link = findLinkId(model,char(this_node.getAttribute('link')),robotnum);
        xyz=zeros(3,1);
        axis=[1;0;0];
        radius=0;
        num_wraps=0;
        if this_node.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(this_node.getAttribute('xyz'))),3,1);
        end
        if this_node.hasAttribute('axis')
          axis = reshape(parseParamString(model,robotnum,char(this_node.getAttribute('axis'))),3,1);
          axis = axis/(norm(axis)+eps); % normalize
        end
        if this_node.hasAttribute('radius')
          radius = parseParamString(model,robotnum,char(this_node.getAttribute('radius')));
        end
        if this_node.hasAttribute('number_of_wraps')
          num_wraps = parseParamString(model,robotnum,char(this_node.getAttribute('number_of_wraps')));
        end
        cable_length_function = addPulley(cable_length_function,link,xyz,axis,radius,num_wraps);
    end
  end

  if (min_length~=max_length)
    error('only implemented cables with min_length = max_length so far');
  end

  constraint = DrakeFunctionConstraint(min_length,max_length,cable_length_function);
  constraint = setName(constraint,cable_length_function.name);
  constraint.grad_level = 2; %declare that the second derivative is provided
  constraint.grad_method = 'user';
  model = addPositionEqualityConstraint(model,constraint);
end

function model=parseTransmission(model,robotnum,node,options)

% old type as attribute kept for convenience
typeAttribute = char(node.getAttribute('type'));
if ~isempty(typeAttribute) && isempty(strfind(typeAttribute,'SimpleTransmission'))
  return; % only parse SimpleTransmissions so far');
end

actuator = RigidBodyActuator();

childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'type'
      if ( isempty(strfind(char(thisNode.getFirstChild().getNodeValue()),'SimpleTransmission')))
          return;  % only parse SimpleTransmissions so far
      end
    case 'actuator'
      actuator.name = char(thisNode.getAttribute('name'));
      actuator.name=regexprep(actuator.name, '\.', '_', 'preservecase');
    case 'joint'
      jn=regexprep(char(thisNode.getAttribute('name')), '\.', '_', 'preservecase');
      actuator.joint = findJointId(model,jn,robotnum);
    case 'mechanicalreduction'
      actuator.reduction = str2num(char(thisNode.getFirstChild().getNodeValue()));
    case {'#text','#comment'}
      % intentionally blank
    otherwise
      warning([char(thisNode.getNodeName()),' is not a supported element of robot/transmission.']);
  end
end

if (isempty(actuator.joint)), error('transmission elements must specify a joint name'); end

model.actuator=[model.actuator,actuator];
end

    function body=parseInertial(body,node,model,options)
      mass = 0;
      inertia = zeros(3);
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('rpy'))),3,1);
        end
      end
      massnode = node.getElementsByTagName('mass').item(0);
      if ~isempty(massnode)
        if (massnode.hasAttribute('value'))
          mass = parseParamString(model,body.robotnum,char(massnode.getAttribute('value')));
        end
      end
      inode = node.getElementsByTagName('inertia').item(0);
      if ~isempty(inode)
        if inode.hasAttribute('ixx'), ixx = parseParamString(model,body.robotnum,char(inode.getAttribute('ixx'))); else ixx=0; end
        if inode.hasAttribute('ixy'), ixy = parseParamString(model,body.robotnum,char(inode.getAttribute('ixy'))); else ixy=0; end
        if inode.hasAttribute('ixz'), ixz = parseParamString(model,body.robotnum,char(inode.getAttribute('ixz'))); else ixz=0; end
        if inode.hasAttribute('iyy'), iyy = parseParamString(model,body.robotnum,char(inode.getAttribute('iyy'))); else iyy=0; end
        if inode.hasAttribute('iyz'), iyz = parseParamString(model,body.robotnum,char(inode.getAttribute('iyz'))); else iyz=0; end
        if inode.hasAttribute('izz'), izz = parseParamString(model,body.robotnum,char(inode.getAttribute('izz'))); else izz=0; end
        inertia = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
      end

      % randomly scale inertia
      % keep scale factor positive to ensure positive definiteness
      % x'*I*x > 0 && eta > 0 ==> x'*(eta*I)*x > 0
      eta = 1 + min(1,max(-0.9999,options.inertia_error*randn()));
      inertia = eta*inertia;

      if any(rpy)
        % transform inertia back into body coordinates
        R = rpy2rotmat(rpy);
        inertia = R*inertia*R';
      end
      body = setInertial(body,mass,xyz,inertia);
    end

    function body = parseVisual(body,node,model,options)
      c = .7*[1 1 1];

      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('rpy'))),3,1);
        end
      end

      matnode = node.getElementsByTagName('material').item(0);
      if ~isempty(matnode)
        c = RigidBodyManipulator.parseMaterial(matnode,options);
      end

      geomnode = node.getElementsByTagName('geometry').item(0);
      if ~isempty(geomnode)
        if (options.visual || options.visual_geometry)
         geometry = RigidBodyGeometry.parseURDFNode(geomnode,xyz,rpy,model,body.robotnum,options);
         if ~isempty(geometry)
           geometry.c = c;
           body.visual_geometry = {body.visual_geometry{:},geometry};
         end
        end
      end
    end

    function body = parseCollision(body,node,model,options)
      xyz=zeros(3,1); rpy=zeros(3,1);
      origin = node.getElementsByTagName('origin').item(0);  % seems to be ok, even if origin tag doesn't exist
      if ~isempty(origin)
        if origin.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('xyz'))),3,1);
        end
        if origin.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,body.robotnum,char(origin.getAttribute('rpy'))),3,1);
        end
      end

      geomnode = node.getElementsByTagName('geometry').item(0);
      if ~isempty(geomnode)
        geometry = RigidBodyGeometry.parseURDFNode(geomnode,xyz,rpy,model,body.robotnum,options);
        if ~isempty(geometry)
          if (node.hasAttribute('group'))
            name=char(node.getAttribute('group'));
          else
            name='default';
          end
          body = addCollisionGeometry(body,geometry,name);
        end
      end
    end
