classdef RigidBodyTerrain < RigidBodyElement
% basic interface class for terrain
  
  methods (Abstract)
    [z,normal] = getHeight(obj,xy)
    writeWRL(obj,fptr)
  end
  methods 
    function obj = RigidBodyTerrain()
      obj.geom = constructRigidBodyGeometry(obj);
    end

    function feas_check = getStepFeasibilityChecker(obj,foot_radius,options)
      % Stub for a general purpose function for checking step feasibility. See DRCTerrainMap.m for a real implementation.

      function feas = feas_check_fcn(xy)
        feas = ones(1, size(xy, 2));
      end
      feas_check = @feas_check_fcn;
    end
    
    function geom = getRigidBodyGeometry(obj)
      geom = obj.geom;
    end

    function geom = constructRigidBodyGeometry(obj)
      geom = [];
    end
  end

  properties (Access = protected)
    geom % RigidBodyGeometry object
  end
  
end
