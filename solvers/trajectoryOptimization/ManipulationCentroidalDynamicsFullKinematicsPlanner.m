classdef ManipulationCentroidalDynamicsFullKinematicsPlanner < ComDynamicsFullKinematicsPlanner
  % only compute the centroidal dynamics on the object being grasped
  properties(SetAccess = protected)
    grasp_object_idx  % The index of the object being grasped.
    grasp_object_xyz_idx % q(grasp_object_xyz_idx) are the xyz position of the grasped object
    grasp_object_rpy_idx % q(grasp_object_rpy_idx) are the rpy angles of the grasped object
  end
  
  properties(Access = protected)
    grasp_object_com
    grasp_object_inertia;
    nq;
  end
  
  methods
    function obj = ManipulationCentroidalDynamicsFullKinematicsPlanner(robot,grasp_object_idx,N,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct,options)
      if nargin < 10, options = struct(); end
      obj = obj@ComDynamicsFullKinematicsPlanner(robot,N,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct,options);
      if(~isnumeric(grasp_object_idx) || numel(grasp_object_idx) ~= 1)
        error('Drake:ManipulationCentroidalDynamicsFullKinematicsPlanner: grasp_object_idx should be a scalar');
      end
      obj.grasp_object_idx = grasp_object_idx;
      obj.grasp_object_com = obj.robot.getBody(obj.grasp_object_idx).com;
      obj.grasp_object_inertia = obj.robot.getBody(obj.grasp_object_idx).inertia;
      obj.robot_mass = obj.robot.getBody(obj.grasp_object_idx).mass;
      obj.nq = obj.robot.getNumPositions();
    end
  end
  
  methods(Access = protected)
    function obj = addSimpleDynamicConstraints(obj)
      function [c,dc] = comMatch(q,com)
        xyz = q(obj.grasp_object_xyz_idx);
        rpy = q(obj.grasp_object_rpy_idx);
        [R,dR] = rpy2rotmat(rpy);
        c = xyz+R*obj.grasp_object_com-com;
        dc = zeros(3,obj.nq+3);
        dc(:,obj.grasp_object_xyz_idx) = eye(3);
        dc(:,obj.grasp_object_rpy_idx) = sparse(reshape(bsxfun(@times,(1:3)',ones(1,3)),[],1),...
          1:9,reshape(bsxfun(@times,obj.grasp_object_com,ones(1,3)),[],1),3,9)*dR;
        dc(:,obj.nq+(1:3)) = -eye(3);
      end
      
      function [c,dc] = angularMomentumMatch(q,v,k)
        xyz = q(obj.grasp_object_xyz_idx);
        rpy = q(obj.grasp_object_rpy_idx);
        rpydot = v(obj.grasp_object_rpy_idx);
        [R,dR] = rpy2rotmat(rpy);
        Rdot = reshape(dR*rpydot,3,3);
        omega = R'*Rdot;
      end
    end
  end
end