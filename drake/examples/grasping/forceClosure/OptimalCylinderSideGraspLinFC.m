classdef OptimalCylinderSideGraspLinFC < OptimalGrasp
  
  methods
    function obj = OptimalCylinderSideGraspLinFC(quat_cylinder,b_cylinder,r_cylinder,h_cylinder,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options)
      if(nargin<9)
        options = struct();
      end
      options.lin_fc_flag = true;
      options.num_fc_edges = num_fc_edges;
      initial_grasp_prog = SynthesizeGraspCylinderSide(quat_cylinder,b_cylinder,r_cylinder,h_cylinder,num_contacts,mu_face,options);
      lagrangian_step = FixedContactsSearchQ1LinFC(disturbance_pos,num_contacts,num_fc_edges,Qw);
      contact_step = SearchCylinderSideContactsLinFC(quat_cylinder,b_cylinder,r_cylinder,h_cylinder,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options);
      obj = obj@OptimalGrasp(initial_grasp_prog,lagrangian_step,contact_step);
    end
    
  end
end