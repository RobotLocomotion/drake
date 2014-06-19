classdef WalkingPlanData
  properties
    x0
    support_times
    supports
    bodytraj
    zmptraj
    V
    c
    comtraj
    mu
  end

  methods
    function obj = WalkingPlanData(x0, support_times, supports, bodytraj, zmptraj, V, c, comtraj, mu)
      obj.x0 = x0;
      obj.support_times = support_times;
      obj.supports = supports;
      obj.bodytraj = bodytraj;
      obj.zmptraj = zmptraj;
      obj.V = V;
      obj.c = c;
      obj.comtraj = comtraj;
      obj.mu = mu;
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

      keys = obj.bodytraj.keys();
      plot_traj_foh(obj.bodytraj(keys{1}), [0.8, 0.8, 0.2]);
      plot_traj_foh(obj.bodytraj(keys{2}), [0.2, 0.8, 0.8]);
      plot_traj_foh(obj.comtraj, [0,1,0]);
      plot_traj_foh(obj.zmptraj, [0,0,1]);

      lcmgl.switchBuffers();
    end

  end
end


