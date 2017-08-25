classdef testCassieControl3D < DrakeSystem
  
  properties 
    p;
    s;
  end
  
  methods
    function obj = testCassieControl3D(plant)
      % choose the first for planar and the second for 3D drake visualizer
      %obj = obj@DrakeSystem(0,0,18,4,true,true);
      obj = obj@DrakeSystem(0,0,28,6,true,true);

      obj.p = plant;
      disp(plant.getStateFrame);
      obj.s = containers.Map({'base_x',
                              'base_y',
                              'base_z',
                              'base_roll',
                              'base_pitch',
                              'base_yaw',
                              'shoulder_left',
                              'shoulder_left2',
                              'shoulder_right',
                              'shoulder_right2',
                              'ankle_left',
                              'ankle_right',
                              'left_foot_to_toe',
                              'right_foot_to_toe',
                              'base_xdot',
                              'base_ydot',
                              'base_zdot',
                              'base_rolldot',
                              'base_pitchdot',
                              'base_yawdot',
                              'shoulder_leftdot',
                              'shoulder_left2dot',
                              'shoulder_rightdot',
                              'shoulder_right2dot',
                              'ankle_leftdot',
                              'ankle_rightdot',
                              'left_foot_to_toedot',
                              'right_foot_to_toedot'}, [1:28]);
      obj = obj.setInputFrame(plant.getStateFrame);
      obj = obj.setOutputFrame(plant.getInputFrame);
    end
       
        
    function u = output(obj,t,junk,x)
      
      % pd control on legs
      [l_disp, l_vel, r_disp,r_vel] = legs(t);
      
      % first option is for planar
      if 0
          r_disp_err = -x(10) + r_disp; r_vel_err = -x(24) + r_vel;
          l_disp_err = -x(9) + l_disp; l_vel_err = -x(23) + l_vel;
          shoulder_info = [x(4); x(5); x(18); x(19)];
          head_angle = x(3);
      else
          r_disp_err = -x(obj.s('ankle_right')) + r_disp; r_vel_err = -x(obj.s('ankle_rightdot')) + r_vel;
          l_disp_err = -x(obj.s('ankle_left')) + l_disp; l_vel_err = -x(obj.s('ankle_leftdot')) + l_vel;
          shoulder_info = [x(obj.s('shoulder_left2')); x(obj.s('shoulder_right2')); x(obj.s('shoulder_left2dot')); x(obj.s('shoulder_right2dot')); x(obj.s('shoulder_left')); x(obj.s('shoulder_right')); x(obj.s('shoulder_leftdot')); x(obj.s('shoulder_rightdot')) ];
          head_pitch = x(obj.s('base_pitch'));
          head_roll = x(obj.s('base_roll'));
          head_yaw = x(obj.s('base_yaw'));
          lat_vel = x(obj.s('base_xdot'));
          forward_vel = x(obj.s('base_ydot'));
          head_height = x(obj.s('base_z'));
          head_rolldot = x(obj.s('base_rolldot'));
          head_pitchdot = x(obj.s('base_pitchdot'));
          head_yawdot = x(obj.s('base_yawdot'));
          
      end
      
      % found good pd constants on the force that changes leg-length 
      r_leg_force = 30000 * r_disp_err + 100 * r_vel_err;
      l_leg_force = 30000 *l_disp_err + 100 * l_vel_err;
      
      
      % Here is a function of the desired lateral and forward velocities as
      % a function of time. Create whatever function you'd like.
      if t <1
          LATERAL_VEL_GOAL = 0.0;
          FORWARD_VEL_GOAL = 0.0;
      elseif t < 20
          LATERAL_VEL_GOAL = 1.5 * t/20;
          FORWARD_VEL_GOAL = 1.5 * t/20;
      % this should make it change direction if t >= 20
      else
          LATERAL_VEL_GOAL = .5;
          FORWARD_VEL_GOAL = -.5 * 2*(t - 10)/10;
      end
          
      % this has to do with the control behind Atrias 
      % to move forward at x speed, the control acts as if its trying to
      % cancel out -x speed
      lat_vel = lat_vel - LATERAL_VEL_GOAL;
      forward_vel = forward_vel - FORWARD_VEL_GOAL;

      % this just finds the joint angles for a given velocity that its
      % trying to cancel. The zero angles are the joint angles to get the arm
      % to be perpendicular to the ground
      [angle_for_forward_vel, angle_for_lat] = getArmAngles(head_height, lat_vel, forward_vel, head_pitch, head_roll, head_yaw, t);
      [zero_forward_angle, zero_lat_angle] = getArmAngles(head_height, 0, 0, head_pitch, head_roll, head_yaw, t);
      
      
      contactConstraints = obj.p.contactConstraints(obj.p.doKinematics(x(1:14)));
      
      leftContact = contactConstraints(3);
      rightContact = contactConstraints(4);
      %right_contact = r.contactConstraints(r.doKinematics(x(1:14));

      % tell leg to go to goal angle (relative to the world) or go to zero
      % angle (relative to the world)
      [stanceLeg,ratio] = percentIntoStance(t);
      if (leftContact < 0.01)
          %disp(stanceLeg,ratio);
          l_lat_ang_goal = -zero_lat_angle;
          l_walk_ang_goal = zero_forward_angle;
          r_lat_ang_goal = -angle_for_lat;
          r_walk_ang_goal = angle_for_forward_vel;
          
      else
          r_lat_ang_goal = -zero_lat_angle;
          r_walk_ang_goal = zero_forward_angle;
          l_lat_ang_goal = -angle_for_lat;
          l_walk_ang_goal = angle_for_forward_vel;
      end
      
      
      angle_goals = [l_lat_ang_goal; r_lat_ang_goal; l_walk_ang_goal; r_walk_ang_goal];
      
      % get torques for two joints on each leg using desired angle and
      % current leg state
      [l_should_torque, r_should_torque, l_walk_torque, r_walk_torque] = getPDAngleTorques(angle_goals, shoulder_info);
        
      % choose swing leg based on which foot is off ground
      if ((leftContact < 0.01) && (rightContact > 0.01)) 
        u = [l_leg_force; r_leg_force; 200*head_roll + 100*head_rolldot; r_walk_torque; 200*head_pitch + 100*head_pitchdot; r_should_torque];
      elseif ((rightContact < 0.01) && (leftContact > 0.01))
        u = [l_leg_force; r_leg_force; l_walk_torque ; 200*head_roll + 100*head_rolldot; l_should_torque ; 200*head_pitch + 100*head_pitchdot];
      % else if both feet are on the ground
      elseif ((rightContact < 0.01) && (leftContact < 0.01))
          u = [l_leg_force; r_leg_force; 0;0;0;0];
      % else they're both in the air
      else
          u = [l_leg_force; r_leg_force; l_walk_torque ; r_walk_torque; l_should_torque ; r_should_torque];
      end

    end
  end 
  
end

% returns desired positions and velocities of legs according to Atrias
% spring mass model
function [l_displacement, l_vel, r_displacement,r_vel] = legs(t)
    l_ret = -.15;
    l0 = 0;
    w = 2*pi/.7;
    
    l_displacement = (sin(w*t) < 0) * (l0 + l_ret * sin(w*t)) + (sin(w*t) >= 0) * (l0); 
    l_vel = (sin(w*t) < 0) * (l_ret * w * cos(w*t)) + 0;
    
    r_displacement = (sin(w*t) > 0) * (l0 - l_ret * sin(w*t)) + (sin(w*t) <= 0) * (l0);
    r_vel = (sin(w*t) > 0) * (-l_ret* w * cos(w*t)) + 0;
end

function [l_should_torque, r_should_torque, l_walk_torque, r_walk_torque] = getPDAngleTorques(angle_goals, shoulder_info)
    l_should_torque =  100 * (angle_goals(1) - shoulder_info(1)) -  8.0 * shoulder_info(3);
    r_should_torque = 100 * (angle_goals(2) - shoulder_info(2)) -  8.0 * shoulder_info(4);
    
    l_walk_torque = 100 * (angle_goals(3) - shoulder_info(5)) -  8.0 * shoulder_info(7);
    r_walk_torque = 100 * (angle_goals(4) - shoulder_info(6)) -  8.0 * shoulder_info(8);
    
end

function [stanceLeg,ratio] = percentIntoStance(t)
    w = 2*pi/.7;
    if (sin(w*t) >= 0)
       stanceLeg = 'l';
    else
       stanceLeg = 'r';
    end
    ratio = (w*t/pi) - floor(w*t/pi);
end

% calculates the two joint angles to put arm/leg at desired velocity
% this function breaks down when pitch is pi/2, but hopefully the robot
% should never be at pi/2 (unless it is doing flips -- we're not there
% yet!)
function [cyl_angle, reach_angle] = getArmAngles(h, xdot, ydot, head_pitch, head_roll, head_yaw, t)
     
    v = sqrt(xdot^2 + ydot^2);
   
    beta = .25 * v;
    xgoal = h * tan(beta) * (xdot/v);
    ygoal = h * tan(beta) * (ydot/v);
    
    if isnan(xgoal)
        xgoal = 0.0;
    end
    if isnan(ygoal)
        ygoal = 0.0;
    end
    
    goal_vector = [xgoal ygoal -h];
    
    pitch_plane_n = [-cos(head_pitch)*cos(head_yaw) -cos(head_pitch)*sin(head_yaw) sin(head_pitch)];
    roll_plane_n = [-cos(head_roll)*sin(head_yaw) cos(head_roll)*cos(head_yaw) sin(head_roll)];
    
    pitch_plane_n = -pitch_plane_n;
        
    g_dot_pitchplane_n = dot(goal_vector, pitch_plane_n);
    
    pitch_plane_norm = norm(pitch_plane_n);
    
    answer = asin(g_dot_pitchplane_n/(norm(goal_vector)*pitch_plane_norm));
    
 
    if isnan(answer)
        answer = 0;
    end
    
    reach_angle = answer;
    
    projected_goal = goal_vector - (g_dot_pitchplane_n/(pitch_plane_norm^2)) * pitch_plane_n;
    
    projected_minus_z = [0.0 0.0 -1.0] - (dot([0.0 0.0 -1.0], pitch_plane_n)/(pitch_plane_norm^2)) * pitch_plane_n;
    
    x = dot(projected_goal,projected_minus_z);
    y = norm(projected_goal)*norm(projected_minus_z);
    
    w = x/y;
    if (w >= 1) || (x == y)
        velocity_angle = 0.0;
    else
        velocity_angle = acos(x/y);
    end
    

    if isnan(velocity_angle)
        velocity_angle = 0.0;
    end
    
    
    cyl_angle = -head_roll + (velocity_angle * sign(ydot));

    
end
