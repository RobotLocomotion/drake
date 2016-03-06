classdef FrictionCone
  % The nonlinear friction cone defined by unit cone axis cone_axis and friction
  % coefficient mu_face
  properties(SetAccess = protected)
    contact_pos % A 3 x 1 vector. The position of the contact point
    cone_axis % A 3 x 1 unit length vector, the cone axis
    mu_face % A scalar, the friction cone coefficient
  end
  
  methods
    function obj = FrictionCone(contact_pos,cone_axis,mu_face)
      if(any(size(contact_pos) ~= [3,1]))
        error('contact_pos should be a 3 x 1 vector');
      end
      obj.contact_pos = contact_pos;
      if(any(size(cone_axis) ~= [3,1]))
        error('cone axis should be a 3 x 1 vector');
      end
      obj.cone_axis = cone_axis;
      if(numel(mu_face) ~= 1 || mu_face <0)
        error('mu_face should be a non-negative scalar');
      end
      obj.mu_face = mu_face;
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
        h = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,cone_name);
        h.glColor4f(1,0,0,0.5);
        h.glPushMatrix();
        h.glTranslated(obj.contact_pos(1),obj.contact_pos(2),obj.contact_pos(3));
        R = rotateVectorToAlign([0;0;1],obj.cone_axis);
        axis = rotmat2axis(R);
        h.glRotated(axis(4)/pi*180,axis(1),axis(2),axis(3));
        h.cylinder([0;0;0],0,cone_length*obj.mu_face*1.01,cone_length,20,20);
        h.glPopMatrix();
        h.switchBuffers();
      else
        hold on;
        n_cone_edges = 50;
        [x_fc0,y_fc0,z_fc0] = cylinder([0,obj.mu_face*1.001],n_cone_edges);
        fc_origin = bsxfun(@times,obj.contact_pos,ones(1,n_cone_edges+1));
        R = rotateVectorToAlign([0;0;1],obj.cone_axis);
        fc_top = R*[x_fc0(2,:);y_fc0(2,:);z_fc0(2,:)]*cone_length+fc_origin;
        h = surf([fc_origin(1,:);fc_top(1,:)],[fc_origin(2,:);fc_top(2,:)],[fc_origin(3,:);fc_top(3,:)],'LineStyle','none','FaceColor',[1,0,0.2]);
        set(h,'FaceAlpha',0.5);
        hold off;
      end
    end
  end
end