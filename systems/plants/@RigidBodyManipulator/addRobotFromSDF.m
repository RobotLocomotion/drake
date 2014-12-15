function model=addRobotFromSDF(model,sdf_filename,xyz,rpy,options)
% Parses SDF.
% See, for instance, https://bitbucket.org/osrf/gazebo_models/
%
% @param sdf_filename filename of file to parse
%
% @options floating boolean where true means that a floating joint is
% automatically added to the root. @default false
% @options inertial boolean where true means parse dynamics parameters,
% false means skip them. @default true
% @options visual boolean where true means parse graphics parameters, false
% means skip them. @default true
% Useful for extracting the 2D geometries later. @default false
% @ingroup SDF Parsing

if (nargin<3 || isempty(xyz)), xyz = zeros(3,1); end
if (nargin<4 || isempty(rpy)), rpy = zeros(3,1); end

if (nargin<5), options = struct(); end
if (~isfield(options,'floating')), options.floating = ''; end % no floating base
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
if (~isfield(options,'namesuffix')), options.namesuffix = ''; end

sdf = xmlread(sdf_filename);

models = sdf.getElementsByTagName('model')
for i=0:(models.getLength-1)
  model = parseSDFModel(model,models.item(i),xyz,rpy,options);
end

model.dirty = true;
model = compile(model); % ideally this would happen on entry into any function...

end

function model=parseSDFModel(model,node,xyz,rpy,options)
% Constructs a model from an XML node

%disp(['Parsing robot ', char(node.getAttribute('name')), ' from URDF file...']);
robotname = char(node.getAttribute('name'));
robotname = regexprep(robotname, '\.', '_', 'preservecase');
robotname = [robotname,options.namesuffix];
model.name = [model.name, {robotname}];
model.urdf = vertcat(model.urdf, '');
robotnum = length(model.name);

materials = node.getElementsByTagName('material');
for i=0:(materials.getLength()-1)
  [~,options] = model.parseMaterial(materials.item(i),options);
end

links = node.getElementsByTagName('link');
for i=0:(links.getLength()-1)
  links.item(i).getAttribute('name')
  model = parseLink(model,robotnum,links.item(i),options);
end

joints = node.getElementsByTagName('joint');
for i=0:(joints.getLength()-1)
  model = parseJoint(model,robotnum,joints.item(i),options);
end

% weld the root link of this robot to the world link
ind = find([model.body.parent]<1);
rootlink = ind([model.body(ind).robotnum]==robotnum);
worldlink = 1;
for i=1:length(rootlink)
  if ~isempty(options.floating)
    model = addFloatingBase(model,worldlink,rootlink(i),xyz,rpy,options.floating);
  else
    model = addJoint(model,'','fixed',worldlink,rootlink(i),xyz,rpy);
  end
end

end

function model=parseLink(model,robotnum,node,options)

ignore = char(node.getAttribute('drakeIgnore'));
if strcmp(lower(ignore),'true')
  return;
end

body = RigidBody();
body.robotnum = robotnum;

body.linkname=char(node.getAttribute('name'));
body.linkname=regexprep(body.linkname, '[\[\]\\\/\.]+', '_', 'preservecase');

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

model.body=[model.body,body];
end

    function body=parseInertial(body,node,model,options)
      mass = 0;
      inertia = zeros(3);
      xyz=zeros(3,1); rpy=zeros(3,1);
      posenode = node.getElementsByTagName('pose').item(0);  % seems to be ok, even if pose tag doesn't exist
      if ~isempty(posenode)
        mass = sscanf(char(getNodeValue(getFirstChild(posenode))),'%f');
        xyz = mass(1:3); rpy = pos(4:6);
      end
      massnode = node.getElementsByTagName('mass').item(0);
      if ~isempty(massnode)
        mass = sscanf(char(getNodeValue(getFirstChild(massnode))),'%f');
      end
      inode = node.getElementsByTagName('inertia').item(0);
      if ~isempty(inode)
        if inode.getElementsByTagName('ixx').item(0), ixx = parseParamString(model,body.robotnum,char(inode.getAttribute('ixx'))); else ixx=0; end
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
          geometry.c = c;
          body.visual_geometry = {body.visual_geometry{:},geometry};
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
        if (node.hasAttribute('group'))
          name=char(node.getAttribute('group'));
        else
          name='default';
        end
        body = addCollisionGeometry(body,geometry,name);
      end
    end 
