classdef SearchPolyhedronContactsLinFC < SearchContactsLinFC
  
  methods
    function obj = SearchPolyhedronContactsLinFC(verts,disturbance_pos,num_contacts,mu_face,num_fc_edges,shrink_factor,Qw,options)
      % @param verts. A 3 x N matrix, obj.Pobj is the convex hull of all the verts
      % @param num_contacts   A scalar. The number of contact points
      % @param mu_face    A scalar. The friction coefficient
      % @param shrink_factor   The factor to shrink each face of the polyhedron. The
      % shrunk region is the allowable region for contact
      % @param num_fc_edges   The number of edges in each linearized
      % friction cone
      if(nargin<7)
        options = struct();
      end
      grasped_geometry = GraspedPolyhedron(verts,shrink_factor);
      obj = obj@SearchContactsLinFC(grasped_geometry.inner_sphere_radius*eye(3),grasped_geometry.inner_sphere_center,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options);
      obj = obj.setGraspedGeometry(grasped_geometry);
      obj.itr_max = 100;
    end
  end
end