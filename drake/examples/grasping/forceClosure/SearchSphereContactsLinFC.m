classdef SearchSphereContactsLinFC < SearchContactsLinFC
  
  methods
    function obj = SearchSphereContactsLinFC(sphere_radius,sphere_center,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options)
      if(nargin<8)
        options = struct();
      end
      grasped_geometry = GraspedSphere(sphere_radius,sphere_center);
      obj = obj@SearchContactsLinFC(sphere_radius*eye(3),sphere_center,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options);
      obj = obj.setGraspedGeometry(grasped_geometry);      
    end  
  end
  
end