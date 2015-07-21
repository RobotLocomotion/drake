classdef SynthesizeGraspFreeFaces < ForceClosureContacts
  % Find contact points that can achieve force closure on a polyhedron
  % The force closure constraint is that
  % rank(G) = 6, where G is the grasp matrix, G = [I         I       ... I;
  %                                                \hat(x1) \hat(x2) ... \hat(xN)]
  %              where xi is the location of the i'th contact point
  % Gf = 0, f1, f2, ..., fN in their friction cones
  % f'f = 1
  
  methods
    function obj = SynthesizeGraspFreeFaces(verts,num_contacts,mu_face,shrink_factor,options)
      % @param verts. A 3 x N matrix, obj.Pobj is the convex hull of all the verts
      % @param num_contacts   A scalar. The number of contact points
      % @param mu_face    A scalar. The friction coefficient
      % @param shrink_factor   To avoid edge contact, we define the allowalbe contact 
      %                        region to be the shrunk region of each facet.
      %                        shrink_factor = 1 corresponds to the whole
      %                        region, shrink_factor = 0 corresponds to
      %                        only the center point of each facet
      % @param options      A struct, refer to the parent class for more
      %                     details
      if(nargin<5)
        options = struct();
      end
      grasped_geometry = GraspedPolyhedron(verts,shrink_factor);
      epsilonG = 0.01*min(1,grasped_geometry.inner_sphere_radius)^2;
      obj = obj@ForceClosureContacts(grasped_geometry.inner_sphere_radius*eye(3),grasped_geometry.inner_sphere_center,num_contacts,mu_face,epsilonG,options);
      obj = obj.setGraspedGeometry(grasped_geometry);
    end
    
  end
  
end