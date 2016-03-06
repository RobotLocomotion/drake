classdef OptimalEllipseGraspLinFC < OptimalGrasp
  
  methods
    function obj = OptimalEllipseGraspLinFC(A_ellipsoid,b_ellipsoid,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw)
      initial_grasp_prog = SynthesizeGraspEllipse(A_ellipsoid,b_ellipsoid,num_contacts,mu_face,struct('lin_fc_flag',true,'num_fc_edges',num_fc_edges));
      lagrangian_step = FixedContactsSearchQ1LinFC(disturbance_pos,num_contacts,num_fc_edges,Qw);
      contact_step = SearchEllipseContactsLinFC(A_ellipsoid,b_ellipsoid,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw);
      obj = obj@OptimalGrasp(initial_grasp_prog,lagrangian_step,contact_step);
    end
    
    function plotSolution(obj,sol)
      h = figure;
      obj.initial_grasp_prog.grasped_geometry.plotGeometry(false);
      hold on
      contact_pos_pt = cell(obj.initial_grasp_prog.num_contacts,1);
      for i = 1:obj.initial_grasp_prog.num_contacts
        contact_pos_pt{i} = zeros(3,length(sol));
        for j = 1:length(sol)
          contact_pos_pt{i}(:,j) = sol{j}.contact_pos(:,i);
        end
        plot3(contact_pos_pt{i}(1,:),contact_pos_pt{i}(2,:),contact_pos_pt{i}(3,:),'Color','b','LineWidth',2,'Marker','x');
      end
      axis equal;
      drawnow;
      hold off;
    end
  end
end