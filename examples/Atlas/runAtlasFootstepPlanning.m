function plan = runAtlasFootstepPlanning()

load('data/atlas_fp.mat', 'xstar');
r = Atlas('urdf/atlas_minimal_contact.urdf');
start_pos = struct('right', [0;0;0;0;0;0], 'left', [0;0.26;0;0;0;0]);
goal_pos = struct('right', [1;0;0;0;0;0], 'left', [1;0.26;0;0;0;0]);
plan = r.planFootsteps(start_pos, goal_pos, []);
  

end

