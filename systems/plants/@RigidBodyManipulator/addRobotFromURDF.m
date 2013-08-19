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

if (nargin<3 || isempty(xyz)) xyz = zeros(3,1); end
if (nargin<4 || isempty(rpy)) rpy = zeros(3,1); end

if (nargin<5) options = struct(); end
if (~isfield(options,'floating')) options.floating = ''; end  % no floating base
if isnumeric(options.floating) || islogical(options.floating) 
  if (options.floating)
    options.floating = 'rpy';
  else
    options.floating = '';
  end
end
if (~isfield(options,'inertial')) options.inertial = true; end
if (~isfield(options,'visual')) options.visual = true; end
if (~isfield(options,'collision')) options.collision = true; end
if (~isfield(options,'visual_geometry')) options.visual_geometry = false; end
if (~isfield(options,'namesuffix')) options.namesuffix = ''; end
if (~isfield(options,'inertia_error')) options.inertia_error = 0.0; end
if (~isfield(options,'damping_error')) options.damping_error = 0.0; end

%disp(['Parsing ', urdf_filename]);
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

model.urdf = vertcat(model.urdf, GetFullPath(urdf_filename));
model.dirty = true;

model = compile(model);  % ideally this would happen on entry into any function...

end



function model=parseRobot(model,node,xyz,rpy,options)
% Constructs a robot from a URDF XML node

%disp(['Parsing robot ', char(node.getAttribute('name')), ' from URDF file...']);
robotname = char(node.getAttribute('name'));
robotname = regexprep(robotname, '\.', '_', 'preservecase');
robotname = [robotname,options.namesuffix];

model.name = {model.name{:},robotname};
robotnum = length(model.name);

materials = node.getElementsByTagName('material');
for i=0:(materials.getLength()-1)
  [~,options] = model.parseMaterial(materials.item(i),options);
end

parameters = node.getElementsByTagName('parameter');
for i=0:(parameters.getLength()-1)
  model = parseParameter(model,robotnum,parameters.item(i),options);
end

links = node.getElementsByTagName('link');
for i=0:(links.getLength()-1)
  model = parseLink(model,robotnum,links.item(i),options);
end

if isempty(model.collision_filter_groups)
  model.collision_filter_groups=containers.Map('KeyType','char','ValueType','any');     
  model.collision_filter_groups('no_collision') = CollisionFilterGroup();
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
  model = parseLoopJoint(model,robotnum,loopjoints.item(i),options);
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

% weld the root link of this robot to the world link
ind = find([model.body.parent]<1);
rootlink = ind([model.body(ind).robotnum]==robotnum);
worldlink = 1;

if ~isempty(options.floating)
  model = addFloatingBase(model,worldlink,rootlink,xyz,rpy,options.floating);
else
  model = addJoint(model,'','fixed',worldlink,rootlink,xyz,rpy);
end


end

function model = parseParameter(model,robotnum,node,options)
  p.robotnum = robotnum;
  p.name = char(node.getAttribute('name'));    % mandatory
  p.value = str2num(char(node.getAttribute('value')));  % mandatory
  n = char(node.getAttribute('lb'));  % optional
  if isempty(n), p.lb = -inf; else p.lb = str2num(n); end
  n = char(node.getAttribute('ub'));  % optional
  if isempty(n), p.ub = inf; else p.ub = str2num(n); end
  
  model.param_db = vertcat(model.param_db,p);
end

function model = parseGazebo(model,robotnum,node,options)
ref = char(node.getAttribute('reference'));
if ~isempty(ref)
  body_ind = findLinkInd(model,ref,robotnum,false);
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


function model=parseTransmission(model,robotnum,node,options)

if isempty(strfind(char(node.getAttribute('type')),'SimpleTransmission'))
  return; % only parse SimpleTransmissions so far');
end

actuator = RigidBodyActuator();

childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'actuator'
      actuator.name = char(thisNode.getAttribute('name'));
      actuator.name=regexprep(actuator.name, '\.', '_', 'preservecase');
    case 'joint'
      jn=regexprep(char(thisNode.getAttribute('name')), '\.', '_', 'preservecase');
      actuator.joint = findJointInd(model,jn,robotnum);
    case 'mechanicalreduction'
      actuator.reduction = str2num(char(thisNode.getFirstChild().getNodeValue()));
    case {'#text','#comment'}
      % intentionally blank
    otherwise
      warning([char(thisNode.getNodeName()),' is not a supported element of robot/transmission.']);
  end
end

if (isempty(actuator.joint)) error('transmission elements must specify a joint name'); end

model.actuator=[model.actuator,actuator];
end    
    
