classdef SynthesizeGraspSphere < ForceClosureContacts
  
  methods
    function obj = SynthesizeGraspSphere(sphere_radius,sphere_center,num_contacts,mu_face,options)
      if(nargin<5)
        options = struct();
      end
      grasped_geometry = GraspedSphere(sphere_radius,sphere_center);
      epsilonG = 0.01*min(1,grasped_geometry.sphere_radius)^2;
      obj = obj@ForceClosureContacts(sphere_radius*eye(3),sphere_center,num_contacts,mu_face,epsilonG,options);
      obj = obj.setGraspedGeometry(grasped_geometry);
    end
  end
  
end