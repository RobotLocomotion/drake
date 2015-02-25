classdef AtlasPlanEval < PlanEval
  properties  
    robot_property_cache
    robot
  end

  methods
    function obj = AtlasPlanEval(r)
      obj = obj@PlanEval();
      obj.robot = r;

      % getTerrainContactPoints is pretty expensive, so we'll just call it
      % for all the bodies and cache the results
      contact_group_cache = cell(1, nbod);
      for j = 1:nbod
        contact_group_cache{j} = struct();
        for f = 1:length(obj.robot.getBody(j).collision_geometry_group_names)
          name = obj.robot.getBody(j).collision_geometry_group_names{f};
          contact_group_cache{j}.(name) = obj.robot.getBody(j).getTerrainContactPoints(name);
        end
      end
      obj.robot_property_cache = struct('pelvis_body_id', obj.robot.findLinkId('pelvis'),...
                                        'neck_id', obj.robot.findPositionIndices('neck'),...
                                        'numq', obj.robot.getNumPositions(),...
                                        'contact_group_cache', contact_group_cache);
    end
  end
end
