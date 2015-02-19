classdef WalkingPlanData
% Container for the results of the ZMP walking planning, which can be consumed by planWalkingStateTraj
% to generate the full walking motion.
  properties
    biped
    x0
    support_times
    supports
    link_constraints
    zmptraj
    V
    qstar = [];
    c = [];
    comtraj
    mu=1;
    t_offset=0;
    ignore_terrain=false;
  end

  methods(Static)
    function obj = from_biped_footstep_plan(footstep_plan, biped, x0, zmp_options)
      if nargin < 4
        zmp_options = struct();
      end
      for j = 1:length(footstep_plan.footsteps)
        footstep_plan.footsteps(j).walking_params = applyDefaults(struct(footstep_plan.footsteps(j).walking_params),...
          biped.default_walking_params);
      end
      [zmp_knots, foot_origin_knots] = biped.planZMPTraj(x0(1:biped.getNumPositions()), footstep_plan.footsteps, zmp_options);
      obj = WalkingPlanData.from_biped_foot_and_zmp_knots(foot_origin_knots, zmp_knots, biped, x0);
    end

    function obj = from_biped_foot_and_zmp_knots(foot_origin_knots, zmp_knots, biped, x0)
      [supports, support_times] = WalkingPlanData.getSupports(zmp_knots);
      zmptraj = WalkingPlanData.getZMPTraj(zmp_knots);
      link_constraints = biped.getLinkConstraints(foot_origin_knots, zmptraj, supports, support_times);
      [c, V, comtraj] = biped.planZMPController(zmptraj, x0);

      obj = WalkingPlanData();
      obj.biped = biped;
      obj.x0 = x0;
      obj.support_times = support_times;
      obj.supports = supports;
      obj.link_constraints = link_constraints;
      obj.zmptraj = zmptraj;
      obj.V = V;
      obj.c = c;
      obj.comtraj = comtraj;
    end

    function [supports, support_times] = getSupports(zmp_knots)
      supports = [zmp_knots.supp];
      support_times = [zmp_knots.t];
    end

    function zmptraj = getZMPTraj(zmp_knots)
      zmptraj = PPTrajectory(foh([zmp_knots.t], [zmp_knots.zmp]));
      zmptraj = setOutputFrame(zmptraj, SingletonCoordinateFrame('desiredZMP',2,'z',{'x_zmp','y_zmp'}));
    end
  end


  methods
    function obj = fix_link(obj, biped, kinsol, link, pt, tolerance_xyz, tolerance_rpy)
      % Add a new link constraint which fixes the point pt on the given link to its
      % pose as given in kinsol, within the tolerance specified.
      pos = biped.forwardKin(kinsol, link, pt, 1);
      pos_min = pos - [tolerance_xyz; tolerance_rpy];
      pos_max = pos + [tolerance_xyz; tolerance_rpy];
      obj.link_constraints(end+1) = struct('link_ndx', link, 'pt', pt, 'min_traj', ConstantTrajectory(pos_min), 'max_traj', ConstantTrajectory(pos_max));
    end

    function draw_lcmgl(obj, lcmgl)
      function plot_traj_foh(traj, color)
        ts = traj.getBreaks();
        pts = traj.eval(ts);
        if size(pts,1) == 2
          pts = [pts; zeros(1,size(pts,2))];
        end
        lcmgl.glColor3f(color(1), color(2), color(3));
        lcmgl.glBegin(lcmgl.LCMGL_LINES);
        for j = 1:length(ts)-1
          lcmgl.glVertex3f(pts(1,j), pts(2,j),pts(3,j));
          lcmgl.glVertex3f(pts(1,j+1), pts(2,j+1), pts(3,j+1));
        end
        lcmgl.glEnd();
      end

      if ~isfield(obj.link_constraints(1), 'traj')
        for j = 1:length(obj.link_constraints)
          obj.link_constraints(j).traj = PPTrajectory(mkpp(obj.link_constraints(j).ts, cat(3, obj.link_constraints(j).a3, obj.link_constraints(j).a2, obj.link_constraints(j).a1, obj.link_constraints(j).a0), 6));
        end
      end
      for j = 1:length(obj.link_constraints)
        if ~isempty(obj.link_constraints(j).traj)
          plot_traj_foh(obj.link_constraints(j).traj, [0.8, 0.8, 0.2]);
        else
          plot_traj_foh(obj.link_constraints(j).traj_min, [0.8, 0.8, 0.2]);
          plot_traj_foh(obj.link_constraints(j).traj_max, [0.2, 0.8, 0.8]);
        end
      end
      plot_traj_foh(obj.comtraj, [0,1,0]);
      plot_traj_foh(obj.zmptraj, [0,0,1]);
    end

  end
end


