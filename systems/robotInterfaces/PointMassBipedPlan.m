classdef PointMassBipedPlan
  properties
    ts
    xcom;
    qr;
    ql;
    qcop;
    support;
    omega;
  end

  methods
    function utraj = getUtraj(obj)
      breaks = obj.ts;
      traj = PPTrajectory(foh(breaks, obj.qcop));
      traj = traj.vertcat(PPTrajectory(foh(breaks, obj.qr)));
      traj = traj.vertcat(PPTrajectory(foh(breaks, obj.ql)));
      contact = obj.getContactSequence();
      traj = traj.vertcat(PPTrajectory(zoh(breaks, contact)));
      traj = traj.vertcat(PPTrajectory(pchipDeriv(breaks, obj.xcom(1:2,:), obj.xcom(3:4,:))));

      utraj = traj.setOutputFrame(PointMassBiped.constructInputFrame());

    end

    function contact = getContactSequence(obj)
      motion = [any(abs(diff(obj.qr, 1, 2)) >= 0.005), false;
                any(abs(diff(obj.ql, 1, 2)) >= 0.005), false];
      contact = ~motion;
    end
  end
end
