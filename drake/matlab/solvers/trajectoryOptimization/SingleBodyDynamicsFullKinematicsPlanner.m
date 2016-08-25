classdef SingleBodyDynamicsFullKinematicsPlanner < ComDynamicsFullKinematicsPlanner
  % only compute the centroidal dynamics on the object being grasped
  properties(SetAccess = protected)
    grasp_object_idx  % The index of the object being grasped.
    grasp_object_xyz_idx % q(grasp_object_xyz_idx) are the xyz position of the grasped object
    grasp_object_rpy_idx % q(grasp_object_rpy_idx) are the rpy angles of the grasped object
  end
  
  properties(Access = protected)
    grasp_object_com
    grasp_object_inertia_origin; % The inertia of the object being grasped, about its body origin
		grasp_object_inertia_com; % The inertia of the object being grasped, about its body COM, with axes aligned with the body frame
		add_simple_dynamics = false;
  end
  
  methods
    function obj = SingleBodyDynamicsFullKinematicsPlanner(robot,grasp_object_idx,N,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct,options)
      % @param grasp_object_idx   The index of the object being grasped
      if nargin < 11, options = struct(); end
      obj = obj@ComDynamicsFullKinematicsPlanner(robot,N,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct,options);
      if(~isnumeric(grasp_object_idx) || numel(grasp_object_idx) ~= 1)
        error('Drake:ManipulationCentroidalDynamicsFullKinematicsPlanner: grasp_object_idx should be a scalar');
      end
      obj.grasp_object_idx = grasp_object_idx;
			obj.grasp_object_xyz_idx = obj.robot.getBody(obj.grasp_object_idx).position_num(1:3);
			obj.grasp_object_rpy_idx = obj.robot.getBody(obj.grasp_object_idx).position_num(4:6);
      obj.grasp_object_com = obj.robot.getBody(obj.grasp_object_idx).com;
      obj.grasp_object_inertia_origin = obj.robot.getBody(obj.grasp_object_idx).inertia;
      obj.robot_mass = obj.robot.getBody(obj.grasp_object_idx).mass;
      obj.grasp_object_inertia_com = obj.grasp_object_inertia_origin-obj.robot_mass*(obj.grasp_object_com'*obj.grasp_object_com)*eye(3)+obj.robot_mass*obj.grasp_object_com*obj.grasp_object_com';
			obj.add_simple_dynamics = true;
			obj = addSimpleDynamicConstraints(obj);
    end

  end
  
	
  methods(Access = protected)
    function obj = addSimpleDynamicConstraints(obj)
		  if(obj.add_simple_dynamics)
				obj = addSimpleDynamicConstraints@ComDynamicsFullKinematicsPlanner(obj);
			end
    end
    
    function obj = addCentroidalDynamicConstraints(obj)
      function [c,dc] = comMatch(q,com)
        xyz = q(obj.grasp_object_xyz_idx);
        rpy = q(obj.grasp_object_rpy_idx);
        [R,dR] = rpy2rotmat(rpy);
        c = xyz+R*obj.grasp_object_com-com;
        dc = zeros(3,obj.nq+3);
        dc(:,obj.grasp_object_xyz_idx) = eye(3);
        dc(:,obj.grasp_object_rpy_idx) = matGradMult(dR,obj.grasp_object_com);
        dc(:,obj.nq+(1:3)) = -eye(3);
      end
      
      function [c,dc] = angularMomentumMatch(q,v,k)
        rpy = q(obj.grasp_object_rpy_idx);
        rpydot = v(obj.grasp_object_rpy_idx);
        [omega,domega] = rpydot2angularvel(rpy,rpydot);
        [R,dR] = rpy2rotmat(rpy);
				c = R*obj.grasp_object_inertia_com*omega-k;
        dc = zeros(3,obj.nq+obj.nv+3);
        dcdrpy = matGradMult(dR,obj.grasp_object_inertia_com*omega)+R*obj.grasp_object_inertia_com*domega(:,1:3);
        dcdrpydot = R*obj.grasp_object_inertia_com*domega(:,4:6);
        dc(:,obj.grasp_object_rpy_idx) = dcdrpy;
        dc(:,obj.nq+obj.grasp_object_rpy_idx) = dcdrpydot;
        dc(:,obj.nq+obj.nv+(1:3)) = -eye(3);
      end
      
      for i = 1:obj.N
        com_cnstr = FunctionHandleConstraint(zeros(3,1),zeros(3,1), obj.nq+3, @comMatch);
        com_cnstr = com_cnstr.setName([{sprintf('com_x(q)=com_x[%d]',i)};{sprintf('com_y(q)=com_y[%d]',i)};{sprintf('com_z(q)=com_z[%d]',i)}]);
        obj = obj.addConstraint(com_cnstr,[{obj.q_inds(:,i)};{obj.com_inds(:,i)}]);
        k_cnstr = FunctionHandleConstraint(zeros(3,1),zeros(3,1),obj.nq+obj.nv+3,@angularMomentumMatch);
        k_cnstr = k_cnstr.setName([{sprintf('I*omega=k_x[%d]',i)};{sprintf('I*omega=k_y[%d]',i)};{sprintf('I*omega=k_z[%d]',i)}]);
        obj = obj.addConstraint(k_cnstr,[{obj.q_inds(:,i)};{obj.v_inds(:,i)};{obj.H_inds(:,i)}]);
      end
    end
  end
end
