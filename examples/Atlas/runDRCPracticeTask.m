function runDRCPracticeTask

task_number=2; % consider taking the task number as an input if/when we load the other tasks

clear gazeboModelPath;
setenv('GAZEBO_MODEL_PATH',[fullfile(getDrakePath,'examples','Atlas','sdf')]); 

% note: could speed this up by saving the heightmap terrain to disk and
% loading it directly
terrain = RigidBodyManipulator(['sdf/drc_practice_task_',num2str(task_number),'.world']);
options.terrain = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(terrain,[],-3:.02:10,-3:.02:3,10);
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

v = r.constructVisualizer();
x0 = Point(r.getStateFrame());
x0.base_x = -3.5;
v.draw(0,x0)
