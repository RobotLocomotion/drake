function runSteppingDemo

options.floating = true;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

% set initial state to fixed point
load('data/atlas_fp2.mat');
r = r.setInitialState(xstar);

v = r.constructVisualizer();
v.display_dt = .05;
v.draw(0,xstar);

if (0)
  [Kp,Kd] = getPDGains(r);
  sys = pdcontrol(r,Kp,Kd);
  poscontrolsys = sys;
else
  r = enableIdealizedPositionControl(r,true);
  r = compile(r);
  sys = r;
  poscontrolsys = r;
end

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
x0 = r.getInitialState(); q0 = x0(1:getNumDOF(r));
qgen = ConstOrPassthroughSystem(x0(7:r.getNumStates()/2),0);
qgen = qgen.setOutputFrame(AtlasJointConfig(r,true));
sys = mimoCascade(qgen,sys);

kinsol = doKinematics(r,q0);
rfoot_body = r.findLink('r_foot');
lfoot_body = r.findLink('l_foot');

rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0]);
lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0]);

gc = r.contactPositions(q0);

% compute desired COM projection
% assumes minimal contact model for now
k = convhull(gc(1:2,1:4)');
lfootcen = [mean(gc(1:2,k),2);0];
k = convhull(gc(1:2,5:8)');
rfootcen = [mean(gc(1:2,4+k),2);0];
%rfootcen = rfoot0;
%lfootcen = lfoot0;

midfoot = mean([rfootcen,lfootcen],2);
com0 = getCOM(r,kinsol);

com = [midfoot,midfoot,rfootcen,rfootcen,rfootcen,rfootcen,rfootcen,midfoot,lfootcen,lfootcen,lfootcen,lfootcen,lfootcen,midfoot];
com(3,:) = com0(3);
tstep = .5*((1:size(com,2))-1);
rfootsupport = 1+0*tstep;
rfootpos = repmat([0;rfoot0;0;0;0],1,length(tstep));
lfootsupport = 1+0*tstep;
lfootpos = repmat([0;lfoot0;0;0;0],1,length(tstep));

lfootpos(1,4:6) = 1;
lfootsupport(4:6) = 0;
lfootpos(4,5) = .2;

rfootsupport(10:12) = 0;
rfootpos(1,10:12) = 1;
rfootpos(4,11) = .2;

comgoal = PPTrajectory(foh(tstep,com));

if (1) % use zmp
  addpath(fullfile(getDrakePath,'examples','ZMP'));
  limp = LinearInvertedPendulum2D(com(3,1));
  zmp = setOutputFrame(comgoal(2),desiredZMP1D);
  zmpcom = ZMPplanner(limp,com(2,1),0,zmp);
  figure(2); clf; fnplt(zmpcom);
  comgoal(2) = zmpcom;
end
comgoal = setOutputFrame(comgoal,AtlasCOM(r));
sys = mimoCascade(comgoal,sys);

rfootpostraj = setOutputFrame(PPTrajectory(foh(tstep,rfootpos(1:4,:))),right_ee.frame);
sys = mimoCascade(rfootpostraj,sys);

lfootpostraj = setOutputFrame(PPTrajectory(foh(tstep,lfootpos(1:4,:))),left_ee.frame);
sys = mimoCascade(lfootpostraj,sys);

supp = repmat(0*tstep,length(r.getLinkNames),1);
supp(strcmp('r_foot',r.getLinkNames),:) = rfootsupport;
supp(strcmp('l_foot',r.getLinkNames),:) = lfootsupport;
supp = setOutputFrame(PPTrajectory(zoh(tstep,supp)),AtlasBody(r));
sys = mimoCascade(supp,sys);


if (1) %%  short-cut COM control, and just call IK
  rfootpos = PPTrajectory(foh(tstep,rfootpos(2:end,:)));
  lfootpos = PPTrajectory(foh(tstep,lfootpos(2:end,:)));
  ts = linspace(0,tstep(end),100);
  ind = getActuatedJoints(r);
  cost = Point(r.getStateFrame,1);
  cost.pelvis_x = 0;
  cost.pelvis_y = 0;
  cost.pelvis_z = 0;
  cost.pelvis_roll = 1000;
  cost.pelvis_pitch = 1000;
  cost.pelvis_yaw = 0;
  cost.back_mby = 100;
  cost.back_ubx = 100;
  cost = double(cost);
  options = struct();
  options.Q = diag(cost(1:r.getNumDOF));
  options.q_nom = q0;
  
  for i=1:length(ts)
    t = ts(i);
    if (i>1)
      q(:,i) = inverseKin(r,q(:,i-1),0,comgoal.eval(t),rfoot_body,rfootpos.eval(t),lfoot_body,lfootpos.eval(t),options);
    else
      q = q0;
    end
    q_d(:,i) = q(ind,i);
    v.draw(t,q(:,i));
  end
  q_dtraj = setOutputFrame(PPTrajectory(spline(ts,q_d)),getInputFrame(poscontrolsys));
  
  sys = cascade(q_dtraj,poscontrolsys);
end

T = tstep(end); % sec
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T]);
save stepping.mat v traj;
playback(v,traj,struct('slider',true));

end