classdef HuboJointCommand < LCMCoordinateFrame
  
  methods
    function obj = HuboJointCommand
      obj = obj@LCMCoordinateFrame('HuboJointCommand','lcmtypes.lcmt_huboplus_input','d');
      setDefaultChannel(obj,'HuboInput');
    end
  end
end
