function runLQR(traj_params,segment_number)

% traj params: 1-fully actuated, periodic; 2-fully actuated, step; 3-spring ankle 
if nargin<1
  traj_params = 1;
end

if nargin < 2
  segment_number = -1; % do full traj
end

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Atlasv4'));

options.twoD = true;
options.view = 'right';
options.floating = true;
options.ignore_self_collisions = true;
options.enable_fastqp = false;
if traj_params==1
  s = '../urdf/atlas_simple_planar_contact.urdf';
  traj_file = 'data/atlas_more_clearance_3mode_lqr'; 
  options.terrain = RigidBodyFlatTerrain();
elseif traj_params==2
  s = '../urdf/atlas_simple_planar_contact.urdf';
  traj_file = 'data/atlas_step_lqr_sk'; 
  step_height = .1;
  options.terrain = RigidBodyLinearStepTerrain(step_height,.35,.02);
elseif traj_params==3
  s = '../urdf/atlas_simple_spring_ankle_planar_contact.urdf';
  traj_file = 'data/atlas_passiveankle_traj_lqr_zoh.mat';
  %traj_file = 'data/atlas_passiveankle_traj_lqr_090314_zoh.mat';
  options.terrain = RigidBodyFlatTerrain();
else
  error('unknown traj_params');
end

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

load(traj_file);

if segment_number<1
  if iscell(xtraj)
    xtraj_full = xtraj{1};
    for i=2:length(xtraj)
      xtraj_full = xtraj_full.append(xtraj{i});
    end
    xtraj = xtraj_full; 
  end

  if iscell(utraj)
    utraj_full = utraj{1};
    for i=2:length(utraj)
      utraj_full = utraj_full.append(utraj{i});
    end
    utraj = utraj_full; 
  end

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
  xtraj = xtraj{segment_number};
  utraj = utraj{segment_number};
  Ktraj = Ktraj{segment_number};
end

xtraj = xtraj.setOutputFrame(getStateFrame(r));
v.playback(xtraj);

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
  traj_ts = traj.getBreaks();
  traj_pts = traj.eval(traj_ts);
  xtraj_pts = xtraj.eval(traj_ts);
  
  figure(111);
  for i=1:nq
    subplot(2,5,i);
    hold on;
    title(r.getStateFrame.coordinates{i});
    plot(traj_ts,xtraj_pts(i,:),'g.-');
    plot(traj_ts,traj_pts(i,:),'r.-');
    hold off;
  end
  figure(112);
  for i=1:10
    subplot(2,5,i);
    hold on;
    title(r.getStateFrame.coordinates{nq+i});
    plot(traj_ts,xtraj_pts(nq+i,:),'g.-');
    plot(traj_ts,traj_pts(nq+i,:),'r.-');
    hold off;
  end
end

end

