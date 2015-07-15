classdef SynthesizeGraspEllipse < ForceClosureContacts
  % Find the force closure grasp on an ellipse, whose geometry is specified
  % as A_ellipse*r+b_ellipse, |r|<=1
  
  methods
    function obj = SynthesizeGraspEllipse(A_ellipsoid,b_ellipsoid,num_contacts,mu_face,options)
      % @param A_ellipsoid  A 3 x 3 matrix 
      % @param b_ellipsoid  A 3 x 1 vector
      if(nargin<5)
        options = struct();
      end
      grasped_geometry = GraspedEllipsoid(A_ellipsoid,b_ellipsoid);
      epsilonG = 0.01*min(1,grasped_geometry.min_eig_A)^2;
      obj = obj@ForceClosureContacts(grasped_geometry.A_ellipsoid,grasped_geometry.b_ellipsoid,num_contacts,mu_face,epsilonG,options);
      obj = obj.setGraspedGeometry(grasped_geometry);
    end
    
  end
end