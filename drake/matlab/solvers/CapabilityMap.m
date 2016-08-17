classdef CapabilityMap
  
  properties
    map
    reachabilityIndex
    sphCenters
    sphDiameter
    nSamples
    angTolerance
    posTolerance
    urdf
    nSph
    rootLink
    rootPoint
    endEffectorLink
    endEffectorPoint
    baseLink
  end
  
  methods
    
    function obj = CapabilityMap(matFile)
      if nargin > 0
        obj = obj.generateFromFile(matFile);
      end
    end
    
    function obj = generateFromFile(obj, matFile)
      vars = load(matFile);
      obj.map = vars.map;
      obj.reachabilityIndex = vars.reachabilityIndex;
      obj.sphCenters = vars.sphCenters;
      obj.sphDiameter = vars.options.sphDiameter;
      obj.nSamples = vars.options.nSamples;
      obj.angTolerance = vars.options.angTolerance;
      obj.posTolerance = vars.options.posTolerance;
      obj.urdf = vars.options.urdf;
      obj.nSph = size(obj.map, 1);
      obj.rootLink = vars.options.rootLink;
      obj.rootPoint = vars.options.rootPoint;
      obj.endEffectorLink = vars.options.endEffectorLink;
      obj.endEffectorPoint = vars.options.endEffectorPoint;
      obj.baseLink = vars.options.baseLink;
    end
    
  end
  
end