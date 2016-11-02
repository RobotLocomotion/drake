classdef RigidBodyLoop < RigidBodyElement
  
  properties
    name
    frameA
    frameB
    axis=[1;0;0];
    constraint_id
  end
  
  methods   
    function [obj,model] = updateConstraints(obj,model)

      % todo: support planar kinematics here (should output only 2
      % constraints instead of 3)
      
      relative_position_fun = drakeFunction.kinematic.RelativePosition(model,obj.frameA,obj.frameB,zeros(3,1));
%      relative_position_fun = relative_position_fun.addInputFrame(model.getVelocityFrame);
      position_constraint = DrakeFunctionConstraint(zeros(3,1),zeros(3,1),relative_position_fun);
      position_constraint.grad_level = 2;
      % todo: naming logic should go into the constraint classes
      % todo: support 2D constraints for planar loops?
      position_constraint = setName(position_constraint,{[obj.name,'_x'];[obj.name,'_y'];[obj.name,'_z']});
      
      % a second relative position constraint enforces the orientation
      relative_position_fun = drakeFunction.kinematic.RelativePosition(model,obj.frameA,obj.frameB,obj.axis);
%      relative_position_fun = relative_position_fun.addInputFrame(model.getVelocityFrame);
      orientation_constraint = DrakeFunctionConstraint(obj.axis,obj.axis,relative_position_fun);
      orientation_constraint.grad_level = 2;
      orientation_constraint = setName(orientation_constraint,{[obj.name,'_ax'];[obj.name,'_ay'];[obj.name,'_az']});
            
      if isempty(obj.constraint_id)
        [model,obj.constraint_id(1)] = addPositionEqualityConstraint(model,position_constraint);
        [model,obj.constraint_id(2)] = addPositionEqualityConstraint(model,orientation_constraint);
      else
        model = updatePositionEqualityConstraint(model,obj.constraint_id(1),position_constraint);
        model = updatePositionEqualityConstraint(model,obj.constraint_id(2),orientation_constraint);
      end
    end

  end
  
  methods (Static)
    function [model,loop] = parseURDFNode(model,robotnum,node,options)
      loop = RigidBodyLoop();
      loop.name = char(node.getAttribute('name'));
      loop.name = regexprep(loop.name, '\.', '_', 'preservecase');

      link1Node = node.getElementsByTagName('link1').item(0);
      body = findLinkId(model,char(link1Node.getAttribute('link')),robotnum);
      xyz = zeros(3,1);
      if link1Node.hasAttribute('xyz')
        xyz = reshape(str2num(char(link1Node.getAttribute('xyz'))),3,1);
      end
      rpy=[0;0;0];  % default according to URDF documentation
      if link1Node.hasAttribute('rpy')
        rpy = reshape(str2num(char(link1Node.getAttribute('rpy'))),3,1);
      end
      [model,loop.frameA] = addFrame(model,RigidBodyFrame(body,xyz,rpy,[loop.name,'FrameA']));

      link2Node = node.getElementsByTagName('link2').item(0);
      body = findLinkId(model,char(link2Node.getAttribute('link')),robotnum);
      xyz = zeros(3,1);
      if link2Node.hasAttribute('xyz')
        xyz = reshape(str2num(char(link2Node.getAttribute('xyz'))),3,1);
      end
      rpy=[0;0;0];  % default according to URDF documentation
      if link2Node.hasAttribute('rpy')
        rpy = reshape(str2num(char(link2Node.getAttribute('rpy'))),3,1);
      end
      [model,loop.frameB] = addFrame(model,RigidBodyFrame(body,xyz,rpy,[loop.name,'FrameB']));
      
      axisnode = node.getElementsByTagName('axis').item(0);
      if ~isempty(axisnode)
        if axisnode.hasAttribute('xyz')
          axis = reshape(parseParamString(model,robotnum,char(axisnode.getAttribute('xyz'))),3,1);
          axis = axis/(norm(axis)+eps); % normalize
        end
      end
      loop.axis = axis;

      type = char(node.getAttribute('type'));
      if ~strcmpi(type,'continuous')
        error(['joint type ',type,' not supported (yet?)']);
      end
    end
  end  
end
