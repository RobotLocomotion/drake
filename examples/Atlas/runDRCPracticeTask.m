function runDRCPracticeTask(task_number)

if nargin<1, task_number=2; end

clear gazeboModelPath;
setenv('GAZEBO_MODEL_PATH',[fullfile(getDrakePath,'examples','Atlas','sdf')]); 

r = Atlas('urdf/atlas_minimal_contact.urdf');
r = r.addRobotFromSDF(['sdf/drc_practice_task_',num2str(task_number),'.world'],[0;0;0]);

v = r.constructVisualizer();
x0 = Point(r.getStateFrame());
x0.base_x = -3.5;
v.inspector(x0)
