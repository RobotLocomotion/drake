function step = fitStepToTerrain(biped, step)
% Take the pose of the center of the foot as pos and return an adjusted pose which matches the height and normal of the terrain under that point

closest_terrain_pos = step.pos(1:3,:);
[closest_terrain_pos(3,:), normal] = biped.getTerrainHeight(step.pos(1:3,:));
normal(:,normal(3,:) < 0) = -normal(:,normal(3,:) < 0);

ground_pos = step.pos;

assert(~any(isnan(closest_terrain_pos)));

ground_pos(1:3,:) = closest_terrain_pos(1:3,:);
ground_pos(:,:) = fitPoseToNormal(ground_pos, normal);

step.pos = ground_pos;
