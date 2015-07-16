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
    function [f_ext, df_ext] = computeSpatialForce(obj,manip,q,qd)
      % todo: re-enable mex for planar version when i write mex the planer
      % bodykin
      
      nq = size(q,1);

      if (obj.b~=0)
        if (nargout>1)
          kinsol = doKinematics(manip,q,qd,struct('compute_gradients', true));
          [x1,J1] = forwardKin(manip,kinsol,obj.body1,obj.pos1);
          [v1, J1dot] = forwardJacDotTimesV(manip, kinsol, obj.body1, obj.pos1, 0, 0);
          dv1dq = J1dot;
          dv1dqd = J1;
          [x2,J2] = forwardKin(manip,kinsol,obj.body2,obj.pos2);
          [v2, J2dot] = forwardJacDotTimesV(manip, kinsol, obj.body2, obj.pos2, 0, 0);
          dv2dq = J2dot;
          dv2dqd = J2;
          % r = x1-x2; l=sqrt(r'r); ldot=(r'rdot)/sqrt(r'r);
          length = norm(x1-x2);
          dlengthdq = ((x1-x2)'/(length+eps))*(J1-J2);
          vel = ((x1-x2)'*(v1-v2))/(length+eps);
          dveldq =  (((v1-v2)'*(J1-J2)+(x1-x2)'*(dv1dq-dv2dq))*(length+eps)-((x1-x2)'*(v1-v2))*dlengthdq)/(length+eps)^2;
          dveldqd = (((x1-x2)'*(dv1dqd-dv2dqd))*(length+eps))/(length+eps)^2;
        else
          kinsol = doKinematics(manip,q);
          [x1,J1] = forwardKin(manip,kinsol,obj.body1,obj.pos1);
          v1 = J1*qd;
          [x2,J2] = forwardKin(manip,kinsol,obj.body2,obj.pos2);
          v2 = J2*qd;
          % r = x1-x2; l=sqrt(r'r); ldot=(r'rdot)/sqrt(r'r);
          length = norm(x1-x2);
          vel = ((x1-x2)'*(v1-v2))/(length+eps);
        end
      else
        kinsol = doKinematics(manip,q);
        if (nargout>1)
          [x1,J1] = forwardKin(manip,kinsol,obj.body1,obj.pos1);
          [x2,J2] = forwardKin(manip,kinsol,obj.body2,obj.pos2);
          length = norm(x1-x2);
          dlengthdq = ((x1-x2)'/(length+eps))*(J1-J2);
          vel = 0;
          dveldq = zeros(1,nq);
          dveldqd = zeros(1,nq);
        else
          x1 = forwardKin(manip,kinsol,obj.body1,obj.pos1);
          x2 = forwardKin(manip,kinsol,obj.body2,obj.pos2);
          length = norm(x1-x2);
          vel=0;
        end
      end
      
      force = obj.k*(length-obj.rest_length) + obj.b*vel;
      if (nargout>1)
        dforcedq = obj.k*dlengthdq + obj.b*dveldq;
        dforcedqd = obj.b*dveldqd;
      end
        
      f_ext = sparse(6,getNumBodies(manip));
      if (nargout>1)
          df_ext = sparse(6,getNumBodies(manip)*2*nq);
      end
      
      fvect1 = force*(x2-x1)/(length+eps);
      fvect2 = force*(x1-x2)/(length+eps);
      if (nargout>1)
          dfvect1dq = (((x2-x1)*dforcedq+force*(J2-J1))*(length+eps)-force*(x2-x1)*dlengthdq)/(length+eps)^2;
          dfvect1dqd = (x2-x1)*dforcedqd*(length+eps)/(length+eps)^2;
          dfvect2dq = (((x1-x2)*dforcedq+force*(J1-J2))*(length+eps)-force*(x1-x2)*dlengthdq)/(length+eps)^2;
          dfvect2dqd = (x1-x2)*dforcedqd*(length+eps)/(length+eps)^2;
      end
      
      if (nargout>1)
        [f1,df1dq,df1dfvect1] = cartesianForceToSpatialForce(manip,kinsol,obj.body1,obj.pos1,fvect1); 
        df1dq = df1dq + df1dfvect1*dfvect1dq;
        df1dqd = df1dfvect1*dfvect1dqd;
        [f2,df2dq,df2dfvect2] = cartesianForceToSpatialForce(manip,kinsol,obj.body2,obj.pos2,fvect2); 
        df2dq = df2dq + df2dfvect2*dfvect2dq;
        df2dqd = df2dfvect2*dfvect2dqd;
      else
        f1 = cartesianForceToSpatialForce(manip,kinsol,obj.body1,obj.pos1,fvect1);
        f2 = cartesianForceToSpatialForce(manip,kinsol,obj.body2,obj.pos2,fvect2);
      end
      
      f_ext(:,obj.body1)=f1;
      f_ext(:,obj.body2)=f2;
      if (nargout>1)        
        df1 = [df1dq, df1dqd];
        df2 = [df2dq, df2dqd];        
        numbodies = getNumBodies(manip);
        for i=1:2*nq
          df_ext(:,(i-1)*numbodies+obj.body1) = df1(:,i);
          df_ext(:,(i-1)*numbodies+obj.body2) = df2(:,i);
        end
      end
      
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.body1 = map_from_old_to_new(obj.body1);
      obj.body2 = map_from_old_to_new(obj.body2);
    end
    
    function obj = updateForRemovedLink(obj,model,body_ind)
      if (obj.body1 == body_ind)
        obj.pos1 = model.body(body_ind).Ttree(1:3,:)*[obj.pos1;1];
        obj.body1 = model.body(body_ind).parent;
      end
      if (obj.body2 == body_ind)
        obj.pos2 = model.body(body_ind).Ttree(1:3,:)*[obj.pos2;1];
        obj.body2 = model.body(body_ind).parent;
      end
    end
    
    function [T,U] = energy(obj,manip,q,qd)
      error('not implemented yet (but would be trivial)');
    end
  end
  
  methods (Static)
    function [model,obj] = parseURDFNode(model,name,robotnum,node,options)
      obj = RigidBodySpringDamper();
      obj.name = name;
      
      if node.hasAttribute('rest_length')
        obj.rest_length = parseParamString(model,robotnum,char(node.getAttribute('rest_length')));
      end
      if node.hasAttribute('stiffness')
        obj.k = parseParamString(model,robotnum,char(node.getAttribute('stiffness')));
      end
      if node.hasAttribute('damping')
        obj.b = parseParamString(model,robotnum,char(node.getAttribute('damping')));
      end
      
      linkNode = node.getElementsByTagName('link1').item(0);
      obj.body1 = findLinkId(model,char(linkNode.getAttribute('link')),robotnum);
      if linkNode.hasAttribute('xyz')
        obj.pos1 = reshape(parseParamString(model,robotnum,char(linkNode.getAttribute('xyz'))),3,1);
      end
      linkNode = node.getElementsByTagName('link2').item(0);
      obj.body2 = findLinkId(model,char(linkNode.getAttribute('link')),robotnum);
      if linkNode.hasAttribute('xyz')
        obj.pos2 = reshape(parseParamString(model,robotnum,char(linkNode.getAttribute('xyz'))),3,1);
      end
    end
  end
end

