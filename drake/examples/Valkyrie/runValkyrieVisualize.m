function runValkyrieVisualize()
% visualize the robot 

visualize = true;

options =[];
r = Valkyrie(fullfile(getDrakePath,'examples','Valkyrie','urdf','urdf','valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf'),options);
r = compile(r);

if visualize
  v = r.constructVisualizer;
  v.display_dt = 0.01;
end

end
