classdef SynthesizeGraspCylinderSide < ForceClosureContacts
  % find the force closure contacts on the side (not top and bottom) of the
  % cylinder,parameterized as
  % R_cylinder*[r_cylinder*cos(theta);r_cylinder*sin(theta);z]+b_cylinder, where R_cylinder
  % is a rotation matrix, and b_cylinder is the translation. -h_cylinder<=z<=h_cylinder
  
  methods
    function obj = SynthesizeGraspCylinderSide(quat_cylinder,b_cylinder,r_cylinder,h_cylinder,num_contacts,mu_face,options)
      % quat_cylinder, A 4 x 1 unit quaternion, R_cylinder is the rotation
      % matrix corresponding to quat_cylinder
      if(nargin<7)
        options = struct();
      end
      grasped_geometry = GraspedCylinder(quat_cylinder,b_cylinder,r_cylinder,h_cylinder);
      epsilonG = grasped_geometry.r_cylinder^2;
      obj = obj@ForceClosureContacts(grasped_geometry.R_cylinder*diag([grasped_geometry.r_cylinder,grasped_geometry.r_cylinder,grasped_geometry.h_cylinder]),grasped_geometry.b_cylinder,num_contacts,mu_face,epsilonG,options);
      obj = obj.setGraspedGeometry(grasped_geometry);
      
      for i = 1:obj.num_contacts
        % f(:,i)*c(:,i)' = f(:,i)*[xc(1:2,i);0]'*obj.R_cylinder'
        obj = obj.withEqs(obj.XCCF{i}(7:9,4:6)-[obj.XCCF{i}(7:9,1:2) zeros(3,1)]*obj.grasped_geometry.R_cylinder');
      end
    end
    
    function obj = addObject2LinkCollisionAvoidance(obj,link_idx,margin)
      obj = addObject2LinkCollisionAvoidance@ForceClosureContacts(obj,link_idx,margin);
      if(margin>=0)
        % Now we can add a constraint that x^2+y^2+z^2 for all [x;y;z]
        % being obj.A_xc\(xhat-obj.b_xc) where xhat are the vertices of the
        % link in the world frame
        rotmat_i = rotmatFromQuatBilinear(obj.body_Quat{link_idx});
        num_vertices = size(obj.body_collision_pts{link_idx},2);
        link_vertices = repmat(obj.body_pos(:,link_idx),1,num_vertices)+rotmat_i*obj.body_collision_pts{link_idx};
        link_vertices_cylinder_frame = inv(obj.R_cylinder*diag([obj.r_cylinder;obj.r_cylinder;obj.h_cylinder]))*(link_vertices-repmat(obj.b_cylinder,1,num_vertices));
        Quat_tril_mask = tril(ones(4,4))~=0;
        Quat_tril = obj.body_Quat{link_idx}(Quat_tril_mask);
        obj = obj.withPos(replaceBilinearProduct(sum(link_vertices_cylinder_frame.^2,1),[obj.body_pos(:,link_idx);Quat_tril],obj.c_bodyposQuat{link_idx}(4:16,4:16))-ones(1,num_vertices));
      end
    end
  end
end
