function runDRCPracticeTask

task_number=2; % consider taking the task number as an input if/when we load the other tasks

clear gazeboModelPath;
setenv('GAZEBO_MODEL_PATH',[fullfile(getDrakePath,'examples','Atlas','sdf')]); 

% note: could speed this up by saving the heightmap terrain to disk and
% loading it directly
terrain = RigidBodyManipulator(['sdf/drc_practice_task_',num2str(task_number),'.world']);
options.terrain = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(terrain,[],-3:.02:10,-3:.02:3,10);
options.ignore_self_collisions = true;
%options.use_bullet=false;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

v = r.constructVisualizer();
x0 = Point(r.getStateFrame());
x0.base_x = 3.5;
x0 = resolveConstraints(r,x0);
v.draw(0,x0)
simulate(cascade(r,v),[0 5],x0);