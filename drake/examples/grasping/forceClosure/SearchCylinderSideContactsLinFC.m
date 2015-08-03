classdef SearchCylinderSideContactsLinFC < SearchContactsLinFC
  
  methods
    function obj = SearchCylinderSideContactsLinFC(quat_cylinder,b_cylinder,r_cylinder,h_cylinder,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options)
      if(nargin<9)
        options = struct();
      end
      grasped_geometry = GraspedCylinder(quat_cylinder,b_cylinder,r_cylinder,h_cylinder);
      obj = obj@SearchContactsLinFC(grasped_geometry.R_cylinder*diag([grasped_geometry.r_cylinder,grasped_geometry.r_cylinder,grasped_geometry.h_cylinder]),grasped_geometry.b_cylinder,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options);
      obj = obj.setGraspedGeometry(grasped_geometry);
    end
  end
end