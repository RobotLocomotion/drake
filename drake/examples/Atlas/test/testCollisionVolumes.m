function testCollisionVolumes()

checkDependency('iris');
options.atlas_version = 3;
path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'Atlas'));
r = Atlas(fullfile(getDrakePath(), 'examples', 'Atlas', 'urdf', 'atlas_convex_hull.urdf'),options);
load(r.fixed_point_file, 'xstar');
q0 = xstar(1:r.getNumPositions());

collision_model = r.getFootstepPlanningCollisionModel(q0);
slices = collision_model.body_slices;

figure(1);
clf
hold on
for j = 1:length(slices.z)
  zmin = slices.z(j);
  if j < length(slices.z)
    zmax = slices.z(j+1);
  else
    zmax = zmin + 1;
  end
  verts = [slices.xy(:,:,j), slices.xy(:,:,j); repmat(zmin, 1, 4), repmat(zmax, 1, 4)];
  iris.drawing.drawPolyFromVertices(verts, 'b');
  axis equal
end
