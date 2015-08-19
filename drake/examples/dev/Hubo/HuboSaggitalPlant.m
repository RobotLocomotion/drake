function [plant,v,r] = HuboSaggitalPlant

options.floating = true;
options.twoD = true;
options.view = 'right';
r = TimeSteppingRigidBodyManipulator('Hubo_description/urdf/jaemiHubo_minimalContact.urdf',.0005,options);
r = setStateFrame(r,HuboSaggitalState());
r = setOutputFrame(r,HuboSaggitalState());

kp = diag([5,repmat(5,1,6),repmat(200,1,6)]); 
kd=diag([ones(7,1);5*ones(6,1)]);
plant = pdcontrol(r,kp,kd,3+(1:13));
plant = setInputFrame(plant,HuboSaggitalJointCommand());

if nargout>1
  v = r.constructVisualizer();
  v.display_dt=0.05;
end

end