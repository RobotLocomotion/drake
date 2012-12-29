function model=parseURDF(model,urdf_filename,options)
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


if (nargin<2) options = struct(); end
if (~isfield(options,'floating')) options.floating = false; end
if (~isfield(options,'inertial')) options.inertial = true; end
if (~isfield(options,'visual')) options.visual = true; end
if (~isfield(options,'visual_geometry')) options.visual_geometry = false; end

%disp(['Parsing ', urdf_filename]);
urdf = xmlread(urdf_filename);

robot = urdf.getElementsByTagName('robot').item(0);
if isempty(robot)
  error('there are no robots in this urdf file');
end

model = parseRobot(model,robot,options);

model=compile(model);
end



function model=parseRobot(model,node,options)
% Constructs a robot from a URDF XML node

%disp(['Parsing robot ', char(node.getAttribute('name')), ' from URDF file...']);
model.name = char(node.getAttribute('name'));
model.name = regexprep(model.name, '\.', '_', 'preservecase');

materials = node.getElementsByTagName('material');
for i=0:(materials.getLength()-1)
  [~,model] = parseMaterial(model,materials.item(i),options);
end

links = node.getElementsByTagName('link');
for i=0:(links.getLength()-1)
  model = parseLink(model,links.item(i),options);
end

joints = node.getElementsByTagName('joint');
for i=0:(joints.getLength()-1)
  model = parseJoint(model,joints.item(i),options);
end

loopjoints = node.getElementsByTagName('loop_joint');
for i=0:(loopjoints.getLength()-1)
  model = parseLoopJoint(model,loopjoints.item(i),options);
end

transmissions = node.getElementsByTagName('transmission');
for i=0:(transmissions.getLength()-1)
  model = parseTransmission(model,transmissions.item(i),options);
end

gazebos = node.getElementsByTagName('gazebo');
for i=0:(gazebos.getLength()-1)
  model = parseGazebo(model,gazebos.item(i),options);
end

if (options.floating)
  % then add a floating joint here
  model = addFloatingBase(model);
end

end



function model = parseGazebo(model,node,options)
ref = char(node.getAttribute('reference'));
if ~isempty(ref)
  body = findLink(model,ref,false);
  if ~isempty(body)
    grav = node.getElementsByTagName('turnGravityOff').item(0);
    if ~isempty(grav)
      val='';
      if grav.hasAttribute('value')
        val = grav.getAttribute('value');
      elseif grav.hasChildNodes
        val = grav.getChildNodes.item(0).getNodeValue();
      end
      if strcmpi(val,'true')
        body.gravity_off = true;
      end
    end
  end
  %        joint = findJoint(model,ref,false)
  %        if ~isempty(joint)
  %          todo: parse initial conditions here?
  %        end
end

end


function model=parseLink(model,node,options)

ignore = char(node.getAttribute('drakeIgnore'));
if strcmp(lower(ignore),'true')
  return;
end

body = newBody(model);

body.linkname=char(node.getAttribute('name'));
body.linkname=regexprep(body.linkname, '\.', '_', 'preservecase');

if (options.inertial && node.getElementsByTagName('inertial').getLength()>0)
  body = parseInertial(body,node.getElementsByTagName('inertial').item(0),options);
end

if (options.visual && node.getElementsByTagName('visual').getLength()>0)
  body = parseVisual(body,node.getElementsByTagName('visual').item(0),model,options);
end

if node.getElementsByTagName('collision').getLength()>0
  collisionItem = 0;
  while(~isempty(node.getElementsByTagName('collision').item(collisionItem)))
    body = parseCollision(body,node.getElementsByTagName('collision').item(collisionItem),options);
    collisionItem = collisionItem+1;
  end
end

model.body=[model.body,body];
end




function model=parseTransmission(model,node,options)

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
      actuator.joint = findJoint(model,jn);
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
    