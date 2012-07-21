function model = parseURDF(urdf_filename,options)
% Parse URDF into a model structure
%
% @param urdf_filename filename of file to parse
%
% @option 2D boolean where true means parse to 2D model, false means parse to 3D model. @default true
% @options inertial boolean where true means parse dynamics parameters,
% false means skip them.  @default true
% @options visual boolean where true means parse graphics parameters, false
% means skip them.  @default true
%
% @retval model Structure compatible with the Featherstone Spatial Vector
% and Dynamics library, with additional drake tags added.

if (nargin<2) options = struct(); end
if (~isfield(options,'2D')) options.twoD = true; end
if (~isfield(options,'inertial')) options.inertial = true; end
if (~isfield(options,'visual')) options.visual = true; end


if (~options.twoD) error('3D not implemented yet'); end

%disp(['Parsing ', urdf_filename]);
urdf = xmlread(urdf_filename);

if (~urdf.hasChildNodes())
  error('No nodes in URDF file');
end

bHaveRobot=false;
childNodes = urdf.getChildNodes();
for i=1:childNodes.getLength()
  node = childNodes.item(i-1);
  switch (lower(char(node.getNodeName())))
    case 'robot'
      if (bHaveRobot)
        warning('There are multiple robots in this urdf file.  Only loading the first one');
      else
        bHaveRobot=true;
        model = parseRobot(node,options);
      end
    otherwise
      warning(['node ',char(node.getNodeName()),' not supported']);
  end
end

if (~bHaveRobot)
  error('didn''t find a robot in the urdf file!');
end


end

function model = parseRobot(node,options)
%disp(['Parsing robot ', char(node.getAttribute('name')), ' from URDF file...']);

model.NB=0;
model.parent=[];
model.jcode=[];
model.Xtree={};
model.Ttree={};
model.I={};
model.geometry=struct();
model.gravity=[0;-9.81];
model.B = [];
model.damping = [];
model.linkname = {};
model.jointname = {};
model.actuatorname = {};
model.loop = [];

childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'link'
      model=parseLink(model,thisNode,options);
    case 'joint'
      model=parseJoint(model,thisNode,options);
    case 'loop_joint'
      model=parseLoopJoint(model,thisNode,options);
    case 'transmission'
      model=parseTransmission(model,thisNode,options);
    case {'sensor','gazebo','#text'}
      % intentionally empty. ok to skip these quietly.
    otherwise
      warning([char(thisNode.getNodeName()),' is not a supported element of robot.']);
  end
end

% remove root link
root = find(model.parent < 0);
if (length(root)<1) error('no root element'); end
if (length(root)>1) error('multiple root elements'); end
model.parent(find(model.parent==root))=0;  % set pointers to the root to zero
model.parent(find(model.parent>root))=model.parent(find(model.parent>root))-1; % all link ids greater than root will shrink by one
model.NB=model.NB-1;  % now zap the details of the root link
model.parent(root)=[];
model.jcode(root)=[];
model.Xtree(root)=[]; % note: this works to delete from cell arrays, too
model.Ttree(root)=[];
model.I(root)=[];
model.damping(root)=[];
if (length(model.damping)<model.NB) model.damping(model.NB)=0; end % zero-pad the damping
% note: model geometry is 1:NB+1
% clean up loops
for i=1:length(model.loop)
  if (model.loop(i).link1==root) model.loop(i).link1 = 0; end
  if (model.loop(i).link1>root) model.loop(i).link1 = model.loop(i).link1-1; end
  if (model.loop(i).link2==root) model.loop(i).link2 = 0; end
  if (model.loop(i).link2>root) model.loop(i).link2 = model.loop(i).link2-1; end
  if (model.loop(i).least_common_ancestor==root) model.loop(i).least_common_ancestor = 0; end
  if (model.loop(i).least_common_ancestor>root) model.loop(i).least_common_ancestor = model.loop(i).least_common_ancestor-1; end
end

end

function model=parseLink(model,node,options)
linknum=length(model.linkname)+1;
model.linkname{linknum} = char(node.getAttribute('name'));

childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'inertial'
      if (options.inertial)
        model=parseInertial(model,thisNode,linknum,options);
      end
    case 'visual'
      if (options.visual)
        model=parseVisual(model,thisNode,linknum,options);
      end
    case {'collision','#text'}
      % intentionally empty. ok to skip these quietly.
    otherwise
      warning([char(thisNode.getNodeName()),' is not a supported element of robot.']);
  end
end

model.NB = model.NB+1;
model.parent(linknum)=-1;  % mark that this link has no parent yet
end

function model=parseInertial(model,node,linknum,options)

mass = 0;
com = zeros(2,1);
theta = 0;
inertia = 1;

childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'origin'
      at = thisNode.getAttributes();
      for j=1:at.getLength()
        thisAt = at.item(j-1);
        switch (lower(char(thisAt.getName())))
          case 'xyz'
            com = reshape(str2num(char(thisAt.getValue())),3,1);
            com = com([1 3]); % ignore y
          case 'rpy'
            rpy=str2num(char(thisNode.getAttribute('rpy')));
            theta = rpy(2);
            if (any(rpy([1 3])))
              warning([obj.linkname{linknum} ,' has an inertial block with out-of-plane rotations.  these will be ignored.']);
            end
        end
      end
    case 'mass'
      mass = str2num(char(thisNode.getAttribute('value')));
    case 'inertia'
      at = thisNode.getAttributes();
      for j=1:at.getLength()
        thisAt = at.item(j-1);
        switch (lower(char(thisAt.getName())))
          case 'iyy'
            inertia = str2num(char(thisAt.getValue()));
          case {'ixx','ixy','ixz','iyz','izz'}
            % ignore y
        end
      end
    case '#text'
      % intentionally empty. ok to skip these quietly.
    otherwise
      warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/inertial.']);
  end
end
model.I{linknum}=mcIp(mass,com,inertia);
end

function model=parseJoint(model,node,options)


parentNode = node.getElementsByTagName('parent').item(0);
parent = char(parentNode.getAttribute('link'));
parentID = strmatch(lower(parent),lower(model.linkname));

childNode = node.getElementsByTagName('child').item(0);
child = char(childNode.getAttribute('link'));
childID = strmatch(lower(child),lower(model.linkname));

model.jointname{childID-1} = char(node.getAttribute('name'));

if (model.parent(childID)>=0)
  error('there is already a joint connecting this child to a parent');
end
model.parent(childID)=parentID;

type = char(node.getAttribute('type'));
switch (lower(type))
  case {'revolute','continuous'}
    model.jcode(childID)=1;
  case 'prismatic'
    model.jcode(childID)=2;
  otherwise
    error(['joint type ',type,' not supported (yet?)']);
end

xz=zeros(2,1);
theta=0;

childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'origin'
      at = thisNode.getAttributes();
      for j=1:at.getLength()
        thisAt = at.item(j-1);
        switch (lower(char(thisAt.getName())))
          case 'xyz'
            xyz = reshape(str2num(char(thisAt.getValue())),3,1);
            xz = xyz([1 3]); % ignore y
          case 'rpy'
            rpy=str2num(char(thisNode.getAttribute('rpy')));
            theta = rpy(2);
        end
      end
    case 'axis'
      ax = reshape(str2num(char(thisNode.getAttribute('xyz'))),3,1);
      switch (model.jcode(childID))
        case 1
          if (abs((ax'*[0; 1; 0]) / (ax'*ax) - 1)>1e-6) error('for 2D processing, revolute and continuous joints must be aligned with [0 1 0]'); end
        case 2
          if (abs((ax'*[1; 0; 0]) / (ax'*ax) - 1)>1e-6) 
            if (abs((ax'*[0; 0; 1]) / (ax'*ax) - 1)>1e-6)
              error('Currently prismatic joints must have their axis in the x-axis or z-axis are supported right now'); 
            else
              model.jcode(childID) = 3;
            end
          end
        otherwise
          error('shouldn''t get here');
      end
    case 'dynamics'
      at = thisNode.getAttributes();
      for j=1:at.getLength()
        thisAt = at.item(j-1);
        switch (lower(char(thisAt.getName())))
          case 'damping'
            model.damping(childID) = str2num(char(thisAt.getValue()));
          otherwise
            warning([char(thisAt.getName()),' is not a supported attribute of robot/joint/dynamics.']);
        end
      end
      
      
    case {'parent','child','#text'}
      % intentionally empty. ok to skip these quietly.
    otherwise
      warning([char(thisNode.getNodeName()),' is not a supported element of robot/joint.']);
  end
end

model.Xtree{childID} = Xpln(theta,xz);
model.Ttree{childID} = [rotmat(theta),xz;0,0,1];
end

function model = parseLoopJoint(model,node,options)

loopID = length(model.loop)+1;
model.loop(loopID).name = char(node.getAttribute('name'));

link1Node = node.getElementsByTagName('link1').item(0);
link1 = char(link1Node.getAttribute('link'));
link1ID = strmatch(lower(link1),lower(model.linkname));
model.loop(loopID).link1 = link1ID;
model.loop(loopID).T1 = parseLoopJointLink(link1Node,options);

link2Node = node.getElementsByTagName('link2').item(0);
link2 = char(link2Node.getAttribute('link'));
link2ID = strmatch(lower(link2),lower(model.linkname));
model.loop(loopID).link2 = link2ID;
model.loop(loopID).T2 = parseLoopJointLink(link2Node,options);

%% find the lowest common ancestor
link1_ancestors=link1ID;
while (link1_ancestors(end)>0)
  link1_ancestors(end+1)=model.parent(link1_ancestors(end));
end
link2_ancestors=link2ID;
while (link2_ancestors(end)>0)
  link2_ancestors(end+1)=model.parent(link2_ancestors(end));
end
% pad (with different numbers) so that they are aligned at the end
n=max(length(link1_ancestors),length(link2_ancestors));
link1_ancestors=[repmat(-2,1,n-length(link1_ancestors)),link1_ancestors];
link2_ancestors=[repmat(-3,1,n-length(link2_ancestors)),link2_ancestors];
% link1_ancestors - link2_ancestors should be non-zero, then all zeros.
lca_ind = find(link1_ancestors == link2_ancestors,1);
model.loop(loopID).least_common_ancestor = link1_ancestors(lca_ind);
%%

type = char(node.getAttribute('type'));
switch (lower(type))
  case {'revolute','continuous'}
    model.loop(loopID).jcode=1;
  case 'prismatic'
    model.loop(loopID).jcode=2;
  otherwise
    error(['joint type ',type,' not supported (yet?)']);
end

childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'axis'
      ax = reshape(str2num(char(thisNode.getAttribute('xyz'))),3,1);
      switch (model.loop(loopID).jcode)
        case 1
          if (abs((ax'*[0; 1; 0]) / (ax'*ax) - 1)>1e-6) error('for 2D processing, revolute and continuous joints must be aligned with [0 1 0]'); end
        case 2
          if (abs((ax'*[1; 0; 0]) / (ax'*ax) - 1)>1e-6) 
            if (abs((ax'*[0; 0; 1]) / (ax'*ax) - 1)>1e-6)
              error('Currently prismatic joints must have their axis in the x-axis or z-axis are supported right now'); 
            else
              model.loop(loopID).jcode = 3;
            end
          end
        otherwise
          error('shouldn''t get here');
      end
  end
end

end


function T = parseLoopJointLink(node,options)
  xz=[0; 0];
  theta=0;

  childNodes = node.getChildNodes();
  for i=1:childNodes.getLength()
    thisNode = childNodes.item(i-1);
    switch (lower(char(thisNode.getNodeName())))
      case 'origin'
        at = thisNode.getAttributes();
        for j=1:at.getLength()
          thisAt = at.item(j-1);
          switch (lower(char(thisAt.getName())))
            case 'xyz'
              xyz = reshape(str2num(char(thisAt.getValue())),3,1);
              xz = xyz([1 3]); % ignore y
            case 'rpy'
              rpy=str2num(char(thisNode.getAttribute('rpy')));
              theta = rpy(2);
          end
        end
      case {'#text'}
        % intentionally empty. ok to skip these quietly.
      otherwise
        warning([char(thisNode.getNodeName()),' is not a supported element of robot/loop_joint/link#.']);
    end
  end
  
  T = [rotmat(theta),xz;0,0,1];
end

function model = parseVisual(model,node,linknum,options)

x0=zeros(3,1);
rpy=zeros(3,1);
xpts = [];
zpts = [];
c = .7*[1 1 1];

% first pass: 
childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'origin'
      at = thisNode.getAttributes();
      for j=1:at.getLength()
        thisAt = at.item(j-1);
        switch (lower(char(thisAt.getName())))
          case 'xyz'
            x0 = reshape(str2num(char(thisAt.getValue())),3,1);
          case 'rpy'
            rpy=str2num(char(thisNode.getAttribute('rpy')));
        end
      end
    case {'geometry','#text'}
      % intentionally fall through
    case 'material'
      c = parseMaterial(thisNode,options);
    otherwise
      warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/inertial.']);
  end
end

% second pass: through and handle the geometry (after the
% coordinates)
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'geometry'
      if (~isempty(xpts)) error('multiple geometries not handled yet (but would be trivial)'); end
      [xpts,zpts] = parseGeometry(thisNode,x0,rpy,options);
  end
end

% % useful for testing local geometry
% h=patch(xpts,zpts,.7*[1 1 1]);
% axis equal
% pause;
% delete(h);

model.geometry(linknum).x = xpts;
model.geometry(linknum).z = zpts;
model.geometry(linknum).c = c;

end

function [x,z] = parseGeometry(node,x0,rpy,options)
  % param node DOM node for the geometry block
  % param X coordinate transform for the current body
  % option twoD true implies that I can safely ignore y.
  x=[];z=[];
  T = [rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)),x0]; % intentially leave off the bottom row [0,0,0,1];

  childNodes = node.getChildNodes();
  for i=1:childNodes.getLength()
    thisNode = childNodes.item(i-1);
    cx=[]; cy=[]; cz=[];
    switch (lower(char(thisNode.getNodeName())))
      case 'box'
        s = str2num(char(thisNode.getAttribute('size')));

        cx = s(1)/2*[-1 1 1 -1 -1 1 1 -1];
        cy = s(2)/2*[1 1 1 1 -1 -1 -1 -1];
        cz = s(3)/2*[1 1 -1 -1 -1 -1 1 1];
        
      case 'cylinder'
        r = str2num(char(thisNode.getAttribute('radius')));
        l = str2num(char(thisNode.getAttribute('length')));
        
        if (options.twoD && rpy(1)==0) % then it just looks like a box
          cx = r*[-1 1 1 -1];
          cy = [0 0 0 0];
          cz = l/2*[1 1 -1 -1];
        elseif (options.twoD && abs(mod(rpy(1),pi)-pi/2)<1e-4) % then it just looks like a circle
          error('not implemented yet');
        else  % full cylinder geometry
          error('not implemented yet');
        end
        
      case '#text'
        % intentionally blank
      otherwise
        warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual/material.']);
    end
    npts = size(cx,2);
    if (npts>0)
      % transform into this body's coordinates
      pts = T*[cx;cy;cz;ones(1,npts)];

      if (~isempty(x)) error('multiple geometries not handled yet (but would be trivial)'); end

      % convex hull?
      i = convhull(pts(1,:),pts(3,:));
      x=pts(1,i);z=pts(3,i);
    end
  end
  
end

function c = parseMaterial(node,options)

% todo:  support material lookup via name attribute
c = .7*[1 1 1];

childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'color'
      c = str2num(char(thisNode.getAttribute('rgba')));
      c = c(1:3);
    case {'texture','#text'}
      % intentionally blank
    otherwise
      warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual/material.']);
  end
end

end


function model=parseTransmission(model,node,options)

jointID=[];
actuatorname='';
B=1;

childNodes = node.getChildNodes();
for i=1:childNodes.getLength()
  thisNode = childNodes.item(i-1);
  switch (lower(char(thisNode.getNodeName())))
    case 'actuator'
      actuatorname = char(thisNode.getAttribute('name'));
    case 'joint'
      jointname = char(thisNode.getAttribute('name'));
      jointID = strmatch(lower(jointname),lower(model.jointname));
      if (isempty(jointID)) error(['unknown joint name: ', jointname]); end
    case 'mechanicalreduction'
      B = str2num(char(thisNode.getFirstChild().getNodeValue()));
    case '#text'
      % intentionally blank
    otherwise
      warning([char(thisNode.getNodeName()),' is not a supported element of robot/transmission.']);
  end
end

if (isempty(jointID)) error('transmission elements must specify a joint name'); end
Bcol = zeros(model.NB-1,1); Bcol(jointID)=B;
model.B = [model.B,Bcol];

end

