function runLQR(segment_number)

if nargin < 1
  segment_number = -1; % do full traj
end
if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Atlasv4'));

if nargin < 1
  segment_number = -1; % do full traj
end

options.twoD = true;
options.view = 'right';
options.floating = true;
options.ignore_self_collisions = true;
options.terrain = RigidBodyFlatTerrain();
s = '../urdf/atlas_simple_planar_contact.urdf';
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = Atlas(s,options);
r = r.setOutputFrame(AtlasXZState(r));
r = r.setStateFrame(AtlasXZState(r));
warning(w);

nx = getNumStates(r);
nq = getNumPositions(r);
nu = getNumInputs(r);

v = r.constructVisualizer;
v.display_dt = 0.01;

%traj_file = 'data/atlas_passiveankle_traj_lqr_090314_zoh.mat';
traj_file = 'data/atlas_lqr_fm2_periodic_100.mat';

load(traj_file);
xtraj = xtraj.setOutputFrame(getStateFrame(r));
v.playback(xtraj);

if segment_number<1
  % just do a foh for the full traj for now...
  ts=[];
  for i=1:length(Ktraj)
    ts = [ts,Ktraj{i}.getBreaks()];
  end
  ts = unique(ts);
  Ks = zeros(nu,nx,length(ts));
  i=1;
  for j=1:length(ts)
    Ks(:,:,j) = Ktraj{i}.eval(ts(j));
    if ts(j)>=Ktraj{i}.tspan(end)
      i=i+1;
    end
  end
  Ktraj = PPTrajectory(zoh(ts,Ks));
else
  Ktraj = Ktraj{segment_number};
end
c = AffineSystem([],[],[],[],[],[],[],-Ktraj,Ktraj*xtraj + utraj);
c = c.setInputFrame(r.getOutputFrame);
c = c.setOutputFrame(r.getInputFrame);
t0 = round(1000*Ktraj.tspan(1))/1000;
tf = Ktraj.tspan(2);

sys = feedback(r,c);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

x0 = xtraj.eval(t0);

traj = simulate(sys,[t0 tf],x0);
playback(v,traj,struct('slider',true));


if 1
  % plot position tracking
  
  xtraj_ts = xtraj.getBreaks();
  xtraj_pts = xtraj.eval(xtraj_ts);
  
  traj_ts = traj.getBreaks();
  traj_pts = traj.eval(traj_ts);
  
  for i=1:10
    figure(100+i);
    hold on;
    title(r.getStateFrame.coordinates{i});
    plot(xtraj_ts,xtraj_pts(i,:),'g.-');
    plot(traj_ts,traj_pts(i,:),'r.-');
    hold off;
  end
end

end

