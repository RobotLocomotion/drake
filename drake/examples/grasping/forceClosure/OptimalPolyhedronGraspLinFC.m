classdef OptimalPolyhedronGraspLinFC < OptimalGrasp
  
  methods
    function obj = OptimalPolyhedronGraspLinFC(verts,disturbance_pos,num_contacts,mu_face,num_fc_edges,shrink_factor,Qw,options)
      if(nargin<7)
        options = struct();
      end
      options.lin_fc_flag = true;
      options.num_fc_edges = num_fc_edges;
      initial_grasp_prog = SynthesizeGraspFreeFaces(verts,num_contacts,mu_face,shrink_factor,options);
      lagrangian_step = FixedContactsSearchQ1LinFC(disturbance_pos,num_contacts,num_fc_edges,Qw);
      lagrangian_step.backoff_scale = 0.95;
      contact_step = SearchPolyhedronContactsLinFC(verts,disturbance_pos,num_contacts,mu_face,num_fc_edges,shrink_factor,Qw,options);
      obj = obj@OptimalGrasp(initial_grasp_prog,lagrangian_step,contact_step);
    end
    
  end
end