classdef RigidBodyLoop
  
  properties
    name
    jcode
    body1
    T1
    body2
    T2
    least_common_ancestor
  end

  methods (Static)
    
    
    function T = parseLink(node,options)
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
          case {'#text','#comment'}
            % intentionally empty. ok to skip these quietly.
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/loop_joint/link#.']);
        end
      end
      
      T = [rotmat(theta),xz;0,0,1];
    end
    
  end
end
