classdef WalkingPlanData
% Container for the results of the ZMP walking planning, which can be consumed by planWalkingStateTraj
% to generate the full walking motion.
  properties
    x0
    support_times
    supports
    link_constraints
    zmptraj
    V
    c
    comtraj
    mu
  end

  methods
    function obj = WalkingPlanData(x0, support_times, supports, link_constraints, zmptraj, V, c, comtraj, mu)
      obj.x0 = x0;
      obj.support_times = support_times;
      obj.supports = supports;


      obj.link_constraints = link_constraints;

      obj.zmptraj = zmptraj;
      obj.V = V;
      obj.c = c;
      obj.comtraj = comtraj;
      obj.mu = mu;
    end

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


