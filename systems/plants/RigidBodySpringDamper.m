classdef RigidBodySpringDamper < RigidBodyForceElement
  
  properties
    rest_length=0
    k=0
    b=0
    body1; 
    pos1 = zeros(3,1);  % in body1 frame coordinates
    body2;
    pos2 = zeros(3,1);
  end
  
  methods
    function [f_ext,bodyind] = computeSpatialForce(obj,manip,q,qd)
      kinsol = doKinematics(manip,q,false,false);  % use_mex = false because bodyKin doesn't have a mex implementation yet
      
      if (obj.b~=0)
        [x1,J1] = forwardKin(manip,kinsol,obj.body1,obj.pos1);
        v1 = J1*qd;
        [x2,J2] = forwardKin(manip,kinsol,obj.body2,obj.pos2);
        v2 = J2*qd;
        % r = x1-x2; l=sqrt(r'r); ldot=(r'rdot)/sqrt(r'r); 
        length = norm(x1-x2);
        vel = ((x1-x2)'*(v1-v2))/length;   
      else
        x1 = forwardKin(manip,kinsol,obj.body1,obj.pos1);
        x2 = forwardKin(manip,kinsol,obj.body2,obj.pos2);
        length = norm(x1-x2);
        vel=0;
      end
      
      force = obj.k*(length-obj.rest_length) + obj.b*vel;
      
      if size(x1,1)==3  % then it's in 3D
        f_ext = sparse(6,getNumDOF(manip));
      else % then 2D
        f_ext = sparse(3,getNumDOF(manip));
      end
      
      if (obj.body1.dofnum>0)
        f_ext(:,obj.body1.dofnum)=cartesianForceToSpatialForce(manip,kinsol,obj.body1,obj.pos1,force*(x2-x1)/length);
      end
      if (obj.body2.dofnum>0)
        f_ext(:,obj.body2.dofnum)=cartesianForceToSpatialForce(manip,kinsol,obj.body2,obj.pos2,force*(x1-x2)/length);
      end
    end
  end
end

% NORELEASE