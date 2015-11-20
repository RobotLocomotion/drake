function prop_cache = propertyCache(r)
% NOTEST
% Functions like findLinkId, getTerrainContactPoints, etc. can be too slow to call
% in the inner loop of our controller or planner, so we cache some useful information
% at setup time. 
% @param r an Atlas

typecheck(r, 'Atlas');
prop_cache = struct('contact_groups', [],...
                    'body_ids', struct(),...
                    'position_indices', struct(),...
                    'actuated_indices', [],...
                    'nq', 0,...
                    'nv', 0,...
                    'num_bodies', 0);

% getTerrainContactPoints is pretty expensive, so we'll just call it
% for all the bodies and cache the results
nbod = length(r.getManipulator().body);
contact_group_cache = cell(1, nbod);
for j = 1:nbod
  contact_group_cache{j} = struct();
  for f = 1:length(r.getBody(j).collision_geometry_group_names)
    name = r.getBody(j).collision_geometry_group_names{f};
    if r.getBody(j).robotnum == 1
      contact_group_cache{j}.(name) = r.getBody(j).getTerrainContactPoints(name);
    end
  end
end

prop_cache.contact_groups = contact_group_cache;

prop_cache.nq = r.getNumPositions();
prop_cache.nv = r.getNumVelocities();
prop_cache.num_bodies = length(r.getManipulator().body);

for b = {'pelvis', 'r_foot', 'l_foot'}
  prop_cache.body_ids.(b{1}) = r.findLinkId(b{1});
end

for j = {'neck', 'r_leg_ak', 'l_leg_ak', 'r_leg', 'l_leg', 'r_leg_kny', 'l_leg_kny', 'arm', 'back_bkz', 'back_bky', 'neck'}
  prop_cache.position_indices.(j{1}) = r.findPositionIndices(j{1});
end

prop_cache.actuated_indices = r.getActuatedJoints();



