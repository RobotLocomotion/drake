classdef OptimalSphereGraspLinFC < OptimalGrasp
  
  methods
    function obj = OptimalSphereGraspLinFC(sphere_radius,sphere_center,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options)
      if(nargin < 8)
        options = struct();
      end
      options.lin_fc_flag = true;
      options.num_fc_edges = num_fc_edges;
      initial_grasp_prog = SynthesizeGraspSphere(sphere_radius,sphere_center,num_contacts,mu_face,options);
      lagrangian_step = FixedContactsSearchQ1LinFC(disturbance_pos,num_contacts,num_fc_edges,Qw);
      contact_step = SearchSphereContactsLinFC(sphere_radius,sphere_center,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options);
      obj = obj@OptimalGrasp(initial_grasp_prog,lagrangian_step,contact_step);      
    end
  end
end