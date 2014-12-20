function runDRCPracticeTask

task_number=2; % consider taking the task number as an input if/when we load the other tasks

clear gazeboModelPath;
setenv('GAZEBO_MODEL_PATH',[fullfile(getDrakePath,'examples','Atlas','sdf')]); 

r = Atlas('urdf/atlas_minimal_contact.urdf');
r = r.addRobotFromSDF(['sdf/drc_practice_task_',num2str(task_number),'.world'],[0;0;0]);

v = r.constructVisualizer();
x0 = Point(r.getStateFrame());
x0.base_x = -3.5;
v.draw(0,x0)
