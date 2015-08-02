classdef HuboState < LCMCoordinateFrame
  
  methods
    function obj = HuboState
      obj = obj@LCMCoordinateFrame('HuboState','lcmtypes.lcmt_huboplus_state','x');
    end
  end
end
