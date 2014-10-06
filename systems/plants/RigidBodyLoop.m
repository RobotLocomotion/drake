classdef RigidBodyLoop < RigidBodyElement
  
  properties
    name
    body1
    pt1=[0;0;0]
    body2
    pt2=[0;0;0]
    axis=[1;0;0]
    constraint_id
  end
  
  methods   
    function [obj,model] = updateConstraints(obj,model)

      % todo: support planar kinematics here (should output only 2
      % constraints instead of 3)
      
      relative_position_fun = drakeFunction.kinematic.RelativePosition(model,obj.body1,obj.body2,obj.pt1);
%      relative_position_fun = relative_position_fun.addInputFrame(model.getVelocityFrame);
      con = DrakeFunctionConstraint(obj.pt2,obj.pt2,relative_position_fun);
      % todo: naming logic should go into the constraint classes
      % todo: support 2D constraints for planar loops?
      con = setName(con,{[obj.name,'_x'];[obj.name,'_y'];[obj.name,'_z']});
      
      if isempty(obj.constraint_id)
        [model,obj.constraint_id] = addPositionEqualityConstraint(model,con);
      else
        model = updatePositionEqualityConstraint(model,obj.constraint_id,con);
      end
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.body1 = map_from_old_to_new(obj.body1);
      obj.body2 = map_from_old_to_new(obj.body2);
    end
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      if (obj.body1 == body_ind)
        obj.pt1 = model.body(body_ind).Ttree(1:end-1,:)*[obj.pt1;1];
        obj.body1 = model.body(body_ind).parent;
      end
      if (obj.body2 == body_ind)
        obj.pt2 = model.body(body_ind).Ttree(1:end-1,:)*[obj.pt2;1];
        obj.body2 = model.body(body_ind).parent;
      end
    end
    
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      error('need to implement this.  (see changeRootLink)');
    end
    
  end
  
  methods (Static)
    function [model,loop] = parseURDFNode(model,robotnum,node,options)
      loop = RigidBodyLoop();
      loop.name = char(node.getAttribute('name'));
      loop.name = regexprep(loop.name, '\.', '_', 'preservecase');

      link1Node = node.getElementsByTagName('link1').item(0);
      link1 = findLinkInd(model,char(link1Node.getAttribute('link')),robotnum);
      loop.body1 = link1;
      if link1Node.hasAttribute('xyz')
        loop.pt1 = reshape(str2num(char(link1Node.getAttribute('xyz'))),3,1);
      end

      link2Node = node.getElementsByTagName('link2').item(0);
      link2 = findLinkInd(model,char(link2Node.getAttribute('link')),robotnum);
      loop.body2 = link2;
      if link2Node.hasAttribute('xyz')
        loop.pt2 = reshape(str2num(char(link2Node.getAttribute('xyz'))),3,1);
      end

      axis=[1;0;0];  % default according to URDF documentation
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

      % todo: add relativeQuatConstraint, too
      warning('Drake:RigidBodyManipulator:ThreeDLoopJoints','3D loop joints do not properly enforce the joint axis constraint.  (they perform more like a ball joint)');
    end
  end  
end
