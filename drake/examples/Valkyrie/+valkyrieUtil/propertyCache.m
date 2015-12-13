function prop_cache = propertyCache(r)
% NOTEST
% Functions like findLinkId, getTerrainContactPoints, etc. can be too slow to call
% in the inner loop of our controller or planner, so we cache some useful information
% at setup time. 
% @param r an Valkyrie

typecheck(r, 'Valkyrie');
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

prop_cache.body_ids.('pelvis') = r.findLinkId('pelvis');
prop_cache.body_ids.('r_foot') = r.findLinkId('rightFoot');
prop_cache.body_ids.('l_foot') = r.findLinkId('leftFoot');

prop_cache.position_indices.('neck') = r.findPositionIndices('lowerNeckPitch');
prop_cache.position_indices.('r_leg_ak') = [r.findPositionIndices('rightAnklePitch'); r.findPositionIndices('rightAnkleRoll')];
prop_cache.position_indices.('l_leg_ak') = [r.findPositionIndices('leftAnklePitch'); r.findPositionIndices('leftAnkleRoll')];

prop_cache.position_indices.('r_leg') = [r.findPositionIndices('rightHipYaw'); r.findPositionIndices('rightHipRoll'); r.findPositionIndices('rightHipPitch'); r.findPositionIndices('rightKneePitch'); r.findPositionIndices('rightAnklePitch'); r.findPositionIndices('rightAnkleRoll')];
prop_cache.position_indices.('l_leg') = [r.findPositionIndices('leftHipYaw'); r.findPositionIndices('leftHipRoll'); r.findPositionIndices('leftHipPitch'); r.findPositionIndices('leftKneePitch'); r.findPositionIndices('leftAnklePitch'); r.findPositionIndices('leftAnkleRoll')];

prop_cache.position_indices.('r_leg_kny') = r.findPositionIndices('rightKneePitch');
prop_cache.position_indices.('l_leg_kny') = r.findPositionIndices('leftKneePitch');

prop_cache.position_indices.('arm') = [r.findPositionIndices('leftShoulderPitch'); r.findPositionIndices('leftShoulderRoll'); r.findPositionIndices('leftShoulderYaw'); r.findPositionIndices('leftElbowPitch'); r.findPositionIndices('leftForearm'); r.findPositionIndices('leftWristRoll'); r.findPositionIndices('leftWristPitch'); r.findPositionIndices('rightShoulderPitch'); r.findPositionIndices('rightShoulderRoll'); r.findPositionIndices('rightShoulderYaw'); r.findPositionIndices('rightElbowPitch'); r.findPositionIndices('rightForearm'); r.findPositionIndices('rightWristRoll'); r.findPositionIndices('rightWristPitch')];

prop_cache.position_indices.('back_bkz') = r.findPositionIndices('torsoYaw');
prop_cache.position_indices.('back_bky') = r.findPositionIndices('torsoPitch');
prop_cache.position_indices.('neck') = r.findPositionIndices('lowerNeckPitch');

prop_cache.actuated_indices = r.getActuatedJoints();



