function runSteppingDemo

options.floating = true;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

% set initial state to fixed point
load('data/atlas_fp.mat');
r = r.setInitialState(xstar);

v = r.constructVisualizer();
v.display_dt = .05;

[Kp,Kd] = getPDGains(r);
sys = pdcontrol(r,Kp,Kd);

c = StandingEndEffectorControl(sys,r);

% set up the two feet as end effectors
joint_names = r.getJointNames();
joint_names = joint_names(2:end); % get rid of null string at beginning..

right_ee = EndEffector(r,'atlas','r_foot',[0;0;0]);
right_ee = right_ee.setMask(~cellfun(@isempty,strfind(joint_names,'r_leg')));
c = c.addEndEffector(right_ee);
left_ee = EndEffector(r,'atlas','l_foot',[0;0;0]);
left_ee = left_ee.setMask(~cellfun(@isempty,strfind(joint_names,'l_leg')));
c = c.addEndEffector(left_ee);

sys = sys.setInputFrame(c.getOutputFrame());

% set up MIMO connections
outs(1).system = 1;
outs(1).output = 1;
sys = mimoFeedback(sys,c,[],[],[],outs);

% nominal position goal
x0 = r.getInitialState(); 
qgen = ConstOrPassthroughSystem(x0(7:r.getNumStates()/2),0);
qgen = qgen.setOutputFrame(AtlasJointConfig(r,true));
sys = mimoCascade(qgen,sys);

kinsol = doKinematics(r,x0(1:getNumDOF(r)));
rfoot_body = r.findLink('r_foot');
lfoot_body = r.findLink('l_foot');

rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0]);
lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0]);
midfoot = mean([rfoot0,lfoot0],2);
com0 = getCOM(r,x0(1:getNumDOF(r)));

com = [midfoot,rfoot0,rfoot0,rfoot0,rfoot0,lfoot0,lfoot0,lfoot0,lfoot0,midfoot];
com(3,:) = com0(3);
tstep = 2*((1:size(com,2))-1);
rfootsupport = 1+0*tstep;
rfootpos = repmat([0;rfoot0],1,length(tstep));
lfootsupport = 1+0*tstep;
lfootpos = repmat([0;lfoot0],1,length(tstep));

lfootpos(1,3:4) = 1;
lfootsupport(3:4) = 0;
lfootpos(4,3) = .15;

rfootsupport(7:8) = 0;

comgoal = setOutputFrame(PPTrajectory(zoh(tstep,com)),AtlasCOM(r));
sys = mimoCascade(comgoal,sys);

rfootpos = setOutputFrame(PPTrajectory(zoh(tstep,rfootpos)),right_ee.frame);
sys = mimoCascade(rfootpos,sys);

lfootpos = setOutputFrame(PPTrajectory(zoh(tstep,lfootpos)),left_ee.frame);
sys = mimoCascade(lfootpos,sys);

supp = repmat(0*tstep,length(r.getLinkNames),1);
supp(strcmp('r_foot',r.getLinkNames),:) = rfootsupport;
supp(strcmp('l_foot',r.getLinkNames),:) = lfootsupport;
supp = setOutputFrame(PPTrajectory(zoh(tstep,supp)),AtlasBody(r));
sys = mimoCascade(supp,sys);


T = 20.0; % sec
if (0)
  traj = simulate(sys,[0 T]); 
  playback(v,traj,struct('slider',true));
else
  warning('off','Drake:DrakeSystem:UnsupportedSampleTime'); 
  sys = cascade(sys,v);
  simulate(sys,[0 T]);
end

end