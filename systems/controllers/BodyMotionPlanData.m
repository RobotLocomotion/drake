classdef BodyMotionPlanData < ControllerData
  properties
    support_times;
    supports;
    link_constraints;
    zmptraj;
    V
    qtraj
    comtraj;
    mu=1;
    t_offset=0;
  end

  methods
    function obj = BodyMotionPlanData(data)
      obj = obj@ControllerData(data);
    end
  end
end
