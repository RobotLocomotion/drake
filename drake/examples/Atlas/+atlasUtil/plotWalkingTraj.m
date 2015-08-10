function [com, rms_com] = plotWalkingTraj(r, traj, walking_plan_data)
%NOTEST

comtraj = walking_plan_data.settings.comtraj;
if ~isa(comtraj, 'Trajectory')
  comtraj = ExpPlusPPTrajectory(comtraj.breaks,...
                                                  comtraj.K,...
                                                  comtraj.A,...
                                                  comtraj.alpha,...
                                                  comtraj.gamma);
end
zmptraj = walking_plan_data.settings.zmptraj;

nq = r.getNumPositions();
tts = traj.getBreaks();
T = tts(end);
dt = tts(2) - tts(1);

nx = r.getNumStates();
x_smooth=zeros(nx,length(tts));
x_breaks = traj.eval(tts);
for i=1:nx
  x_smooth(i,:) = smooth(x_breaks(i,:),15,'lowess');
end
dtraj = fnder(PPTrajectory(spline(tts,x_smooth)));
qddtraj = dtraj(nq+(1:nq));

walking_plan_body_motions = walking_plan_data.settings.body_motions();

body_motions = struct('right', [], 'left', []);
for j = 1:length(walking_plan_body_motions)
  if walking_plan_body_motions(j).body_id == r.foot_body_id.right || ...
     (walking_plan_body_motions(j).body_id < 0 && r.getFrame(walking_plan_body_motions(j).body_id).body_ind == r.foot_body_id.right)
     body_motions.right = walking_plan_body_motions(j);
  elseif  walking_plan_body_motions(j).body_id == r.foot_body_id.left || ...
     (walking_plan_body_motions(j).body_id < 0 && r.getFrame(walking_plan_body_motions(j).body_id).body_ind == r.foot_body_id.left)
     body_motions.left = walking_plan_body_motions(j);
  end
end

foot_bodies = struct();
for f = {'left', 'right'}
  foot = f{1};
  if body_motions.(foot).body_id > 0
    foot_bodies.(foot) = body_motions.(foot).body_id;
  else
    foot_bodies.(foot) = r.getFrame(body_motions.(foot).body_id).body_ind;
  end
end

step_counter = struct('left', 0, 'right', 0);

ts = linspace(tts(1), tts(end), 150);

rms_zmp = 0;
rms_com = 0;
rms_foot = 0;
T = floor(T/dt)*dt;
com = zeros(3, length(ts));
comdes = zeros(2, length(ts));
zmpdes = zeros(2, length(ts));
zmpact = zeros(2, length(ts));
foot_pos = struct('left', zeros(6, length(ts)),...
                  'right', zeros(6, length(ts)));

foot_traj = struct('left', PPTrajectory(body_motions.left.getPP()),...
                   'right', PPTrajectory(body_motions.right.getPP()));

foot_steps = struct('left', [], 'right', []);

for i=1:length(ts)
  % ts is from the walking plan, but traj is only defined at the dt
  % intervals
  t = round(ts(i)/dt)*dt;
  t = min(max(t,0),T);
  
  xt=traj.eval(t);
  q=xt(1:nq);
  qd=xt(nq+(1:nq));
  qdd=qddtraj.eval(t);

  kinsol = doKinematics(r, q, qd);

  [com(:,i),J]=getCOM(r,kinsol);
  Jdotv = centerOfMassJacobianDotTimesV(r,kinsol,0);
  comdes(:,i)=comtraj.eval(t);
  zmpdes(:,i)=zmptraj.eval(t);
  zmpact(:,i)=com(1:2,i) - com(3,i)/9.81 * (J(1:2,:)*qdd + Jdotv(1:2));

  for f = {'left', 'right'}
    foot = f{1};
    foot_cpos = terrainContactPositions(r, kinsol, foot_bodies.(foot));

    foot_p = forwardKin(r, kinsol, foot_bodies.(foot), [0;0;0], 1);

    foot_pos.(foot)(:,i) = foot_p;

    if any(foot_cpos(3,:) < 1e-4)
      step_counter.(foot) = step_counter.(foot) + 1;
      foot_steps.(foot)(:,step_counter.(foot)) = foot_p;
    end

    foot_frame_des = eval(foot_traj.(foot), t);
    foot_frame_act = forwardKin(r, kinsol, body_motions.(foot).body_id, [0;0;0], 2);
    rms_foot = rms_foot + norm(foot_frame_des(1:3) - foot_frame_act(1:3))^2;
  end

  rms_zmp = rms_zmp+norm(zmpdes(:,i)-zmpact(:,i))^2;
  rms_com = rms_com+norm(comdes(:,i)-com(1:2,i))^2;
end

rms_zmp = sqrt(rms_zmp/length(ts))
rms_com = sqrt(rms_com/length(ts))
rms_foot = sqrt(rms_foot/(step_counter.left + step_counter.right))

figure(2);
clf;
subplot(2,1,1);
plot(ts,zmpdes(1,:),'b');
hold on;
plot(ts,zmpact(1,:),'r.-');
plot(ts,comdes(1,:),'g');
plot(ts,com(1,:),'m.-');
hold off;

subplot(2,1,2);
plot(ts,zmpdes(2,:),'b');
hold on;
plot(ts,zmpact(2,:),'r.-');
plot(ts,comdes(2,:),'g');
plot(ts,com(2,:),'m.-');
hold off;

figure(3)
clf;
plot(zmpdes(1,:),zmpdes(2,:),'b','LineWidth',3);
hold on;
plot(zmpact(1,:),zmpact(2,:),'r.-','LineWidth',1);
%plot(comdes(1,:),comdes(2,:),'g','LineWidth',3);
%plot(com(1,:),com(2,:),'m.-','LineWidth',1);

tc = struct('left', getTerrainContactPoints(r,foot_bodies.left, {{'heel', 'toe'}}),...
            'right', getTerrainContactPoints(r,foot_bodies.right, {{'heel', 'toe'}}));

for f = {'left', 'right'}
  foot = f{1};
  planned_foot_steps = eval(foot_traj.(foot), foot_traj.(foot).getBreaks());
  for i=1:size(planned_foot_steps,2);
    cpos = rpy2rotmat(planned_foot_steps(4:6,i)) * tc.(foot).pts + repmat(planned_foot_steps(1:3,i),1,4);
    if all(cpos(3,:)<=0.001)
      plot(cpos(1,[1,2]),cpos(2,[1,2]),'k-','LineWidth',2);
      plot(cpos(1,[1,3]),cpos(2,[1,3]),'k-','LineWidth',2);
      plot(cpos(1,[2,4]),cpos(2,[2,4]),'k-','LineWidth',2);
      plot(cpos(1,[3,4]),cpos(2,[3,4]),'k-','LineWidth',2);
    end
  end

  for i=1:step_counter.(foot)
    cpos = rpy2rotmat(foot_steps.(foot)(4:6,i)) * tc.(foot).pts + repmat(foot_steps.(foot)(1:3,i),1,4);
    plot(cpos(1,[1,2]),cpos(2,[1,2]),'g-','LineWidth',1.65);
    plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',1.65);
    plot(cpos(1,[2,4]),cpos(2,[2,4]),'g-','LineWidth',1.65);
    plot(cpos(1,[3,4]),cpos(2,[3,4]),'g-','LineWidth',1.65);
  end
end

plot(zmpdes(1,:),zmpdes(2,:),'b','LineWidth',3);
plot(zmpact(1,:),zmpact(2,:),'r.-','LineWidth',1);

axis equal;