classdef SearchEllipseContactsLinFC < SearchContactsLinFC
  
  methods
    function obj = SearchEllipseContactsLinFC(A_ellipsoid,b_ellipsoid,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw)
      grasped_geometry = GraspedEllipsoid(A_ellipsoid,b_ellipsoid);
      obj = obj@SearchContactsLinFC(grasped_geometry.A_ellipsoid,grasped_geometry.b_ellipsoid,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw);
      obj = obj.setGraspedGeometry(grasped_geometry);
    end
  end
  
end