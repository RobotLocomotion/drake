classdef SalmonBarVisualizer < ContactWrenchVisualizer
  properties
    l_hand
    r_hand
    l_hand_pt
    r_hand_pt
  end
  methods
    function obj = SalmonBarVisualizer(manip,t_knot,wrench_sol,l_hand,r_hand,l_hand_pt,r_hand_pt,use_collision_geometry)
      if(nargin<8)
        use_collision_geometry = false;
      end
      obj = obj@ContactWrenchVisualizer(manip,t_knot,wrench_sol,use_collision_geometry);
      obj.l_hand = l_hand;
      obj.r_hand = r_hand;
      obj.l_hand_pt = l_hand_pt;
      obj.r_hand_pt = r_hand_pt;
    end
    
    function draw(obj,t,y)
      draw@ContactWrenchVisualizer(obj,t,y);
      q = y(1:obj.model.getNumPositions);
      kinsol = doKinematics(obj.model,q);
      lhand_pos = obj.model.forwardKin(kinsol,obj.l_hand,obj.l_hand_pt,0);
      rhand_pos = obj.model.forwardKin(kinsol,obj.r_hand,obj.r_hand_pt,0);
      drawBar(obj,lhand_pos,rhand_pos,0.025);
    end
    
    function drawBar(obj,lhand_pos,rhand_pos,bar_radius)
      bar_length = 3;
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'bar');
      lcmgl.glColor3f(0.5,0.5,0);
      bar_middle = (lhand_pos(1:3)+rhand_pos(1:3))/2;
      hand_diff = rhand_pos(1:3)-lhand_pos(1:3);
      hand_diff = hand_diff/norm(hand_diff);
      rotate_axis = cross([0;0;1],hand_diff);
      rotate_angle = asin(norm(rotate_axis));
      lcmgl.glPushMatrix();
      lcmgl.glTranslated(bar_middle(1),bar_middle(2),bar_middle(3));
      lcmgl.glRotated(-rotate_angle/pi*180,rotate_axis(1),rotate_axis(2),rotate_axis(3));
      lcmgl.cylinder([0;0;-bar_length/2],bar_radius,bar_radius,bar_length,20,20);
      lcmgl.glPopMatrix;
      lcmgl.switchBuffers();
    end
  end
end
