function runLQR(segment_number)

if nargin < 1
  segment_number = -1; % do full traj
end

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.twoD = true;
options.view = 'right';
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
s = 'OneLegHopper.urdf';
dt = 0.001;
r = TimeSteppingRigidBodyManipulator(s,dt,options);
r = r.setStateFrame(OneLegHopperState(r));
r = r.setOutputFrame(OneLegHopperState(r));

nx = getNumStates(r);
nq = getNumPositions(r);
nu = getNumInputs(r);
v = r.constructVisualizer;
v.display_dt = 0.001;

load('data/hopper_hybrid_lqr_sk.mat');
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

utraj = PPTrajectory(foh(Ktraj.getBreaks(),utraj.eval(Ktraj.getBreaks())));

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
  
  for i=1:10
    figure(100+i);
    hold on;
    title(r.getStateFrame.coordinates{i});
    plot(traj_ts,xtraj_pts(i,:),'g.-');
    plot(traj_ts,traj_pts(i,:),'r.-');
    hold off;
  end
end

end

