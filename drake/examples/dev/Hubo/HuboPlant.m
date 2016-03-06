function [plant,v,r] = HuboPlant

options.floating = true;
r = TimeSteppingRigidBodyManipulator('model/huboplus.urdf',.0005,options);
r = setStateFrame(r,HuboState());
r = setOutputFrame(r,HuboState());

kp = diag([repmat(150,1,18),repmat(200,1,9)]); % note: not tunes AT ALL yet
kd=diag([ones(18,1);5*ones(9,1)]);
plant = pdcontrol(r,kp,kd,6+(1:27));
plant = setInputFrame(plant,HuboJointCommand());

if nargout>1
  v = r.constructVisualizer();
  v.display_dt=0.05;
end

end
