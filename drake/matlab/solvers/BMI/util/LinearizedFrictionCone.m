classdef LinearizedFrictionCone < FrictionCone
  % The linearized friction cone defined by friction cone edges
  properties(SetAccess = protected)
    num_fc_edges % number of edges in the friction cone
    fc_edges % A 3 x num_fc_edges, fc_edges(:,i) is the i'th edge of the linearized friction cone
  end
  
  methods 
    function obj = LinearizedFrictionCone(contact_pos,cone_axis,mu_face,fc_edges)
      obj = obj@FrictionCone(contact_pos,cone_axis,mu_face);
      assert(size(fc_edges,1) == 3);
      obj.num_fc_edges = size(fc_edges,2);
      obj.fc_edges = fc_edges;
    end
    
    function h = plot(obj,use_lcmgl,cone_length,cone_name)
      % @param use_lcmgl  A boolean, true if visualize in lcmgl, false if
      % visualize in MATLAB viewer
      % @param cone_length  A scalar. The length of the cone axis being
      % visualized
      % @param cone_name  A string. The name of the cone, this is only used
      % if use_lcmgl = true
      assert(numel(use_lcmgl) == 1 && islogical(use_lcmgl));
      assert(numel(cone_length) == 1 && cone_length > 0);
      assert(ischar(cone_name));
      if(use_lcmgl)
        for j = 1:obj.num_fc_edges
          h = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,sprintf('%s_edge%d',cone_name,j));
          h.glColor4f(1,0,0,0.5);
          face_vertex = [obj.contact_pos, obj.contact_pos+obj.fc_edges(:,j)*cone_length, obj.contact_pos+obj.fc_edges(:,mod(j,obj.num_fc_edges)+1)*cone_length];
          h.polygon(face_vertex(1,:),face_vertex(2,:),face_vertex(3,:));
          h.switchBuffers();
        end
      else
        hold on
        [x_fc0,y_fc0,z_fc0] = cylinder([0,obj.mu_face*1.001],obj.num_fc_edges);
        fc_origin = bsxfun(@times,obj.contact_pos,ones(1,obj.num_fc_edges+1));
        R = rotateVectorToAlign([0;0;1],obj.cone_axis);
        fc_top = R*[x_fc0(2,:);y_fc0(2,:);z_fc0(2,:)]*cone_length+fc_origin;
        h = surf([fc_origin(1,:);fc_top(1,:)],[fc_origin(2,:);fc_top(2,:)],[fc_origin(3,:);fc_top(3,:)],'LineStyle','--','FaceColor',[1,0,0.2]);
        set(h,'FaceAlpha',0.5);
        hold off
      end
    end
  end
end
