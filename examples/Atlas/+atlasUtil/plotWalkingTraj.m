function [com, rms_com] = plotWalkingTraj(r, traj, walking_plan_data)
%NOTEST

nq = r.getNumPositions();
tts = traj.getBreaks();
T = tts(end);
dt = tts(2) - tts(1);

x_smooth=smoothts(traj.eval(tts),'e',150);
dtraj = fnder(PPTrajectory(spline(tts,x_smooth)));
qddtraj = dtraj(nq+(1:nq));

lfoot = findLinkId(r,'l_foot');
rfoot = findLinkId(r,'r_foot');

lstep_counter = 0;
rstep_counter = 0;

ts = linspace(tts(1), tts(end), 150);

rms_zmp = 0;
rms_com = 0;
rms_foot = 0;
T = floor(T/dt)*dt;
com = zeros(3, length(ts));
comdes = zeros(2, length(ts));
zmpdes = zeros(2, length(ts));
zmpact = zeros(2, length(ts));
lfoot_pos = zeros(6, length(ts));
rfoot_pos = zeros(6, length(ts));

rfoottraj = PPTrajectory(pchip(walking_plan_data.link_constraints(1).ts,...
                               walking_plan_data.link_constraints(1).poses));
lfoottraj = PPTrajectory(pchip(walking_plan_data.link_constraints(2).ts,...
                               walking_plan_data.link_constraints(2).poses));

for i=1:length(ts)
  % ts is from the walking plan, but traj is only defined at the dt
  % intervals
  t = round(ts(i)/dt)*dt;
  t = min(max(t,0),T);
  
  xt=traj.eval(t);
  q=xt(1:nq);
  qd=xt(nq+(1:nq));
  qdd=qddtraj.eval(t);

  kinsol = doKinematics(r,q);

  [com(:,i),J]=getCOM(r,kinsol);
  Jdot = forwardJacDot(r,kinsol,0);
  comdes(:,i)=walking_plan_data.comtraj.eval(t);
  zmpdes(:,i)=walking_plan_data.zmptraj.eval(t);
  zmpact(:,i)=com(1:2,i) - com(3,i)/9.81 * (J(1:2,:)*qdd + Jdot(1:2,:)*qd);

  lfoot_cpos = terrainContactPositions(r,kinsol,lfoot);
  rfoot_cpos = terrainContactPositions(r,kinsol,rfoot);

  lfoot_p = forwardKin(r,kinsol,lfoot,[0;0;0],1);
  rfoot_p = forwardKin(r,kinsol,rfoot,[0;0;0],1);

  lfoot_pos(:,i) = lfoot_p;
  rfoot_pos(:,i) = rfoot_p;

  if any(lfoot_cpos(3,:) < 1e-4)
    lstep_counter=lstep_counter+1;
    lfoot_steps(:,lstep_counter) = lfoot_p;
  end
  if any(rfoot_cpos(3,:) < 1e-4)
    rstep_counter=rstep_counter+1;
    rfoot_steps(:,rstep_counter) = rfoot_p;
  end


  lfoot_des = eval(lfoottraj,t);
  lfoot_des(3) = max(lfoot_des(3), 0.0811);     % hack to fix footstep planner bug
  rms_foot = rms_foot+norm(lfoot_des([1:3])-lfoot_p([1:3]))^2;

  rfoot_des = eval(rfoottraj,t);
  rfoot_des(3) = max(rfoot_des(3), 0.0811);     % hack to fix footstep planner bug
  rms_foot = rms_foot+norm(rfoot_des([1:3])-rfoot_p([1:3]))^2;

  rms_zmp = rms_zmp+norm(zmpdes(:,i)-zmpact(:,i))^2;
  rms_com = rms_com+norm(comdes(:,i)-com(1:2,i))^2;
end

rms_zmp = sqrt(rms_zmp/length(ts))
rms_com = sqrt(rms_com/length(ts))
rms_foot = sqrt(rms_foot/(lstep_counter+rstep_counter))

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

left_foot_steps = eval(lfoottraj,lfoottraj.getBreaks);
tc_lfoot = getTerrainContactPoints(r,lfoot);
tc_rfoot = getTerrainContactPoints(r,rfoot);
for i=1:size(left_foot_steps,2);
  cpos = rpy2rotmat(left_foot_steps(4:6,i)) * tc_lfoot.pts + repmat(left_foot_steps(1:3,i),1,4);
  if all(cpos(3,:)<=0.001)
    plot(cpos(1,[1,2]),cpos(2,[1,2]),'k-','LineWidth',2);
    plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',2);
    plot(cpos(1,[1,3]),cpos(2,[1,3]),'k-','LineWidth',2);
    plot(cpos(1,[2,4]),cpos(2,[2,4]),'k-','LineWidth',2);
    plot(cpos(1,[3,4]),cpos(2,[3,4]),'k-','LineWidth',2);
  end
end

right_foot_steps = eval(rfoottraj,rfoottraj.getBreaks);
for i=1:size(right_foot_steps,2);
  cpos = rpy2rotmat(right_foot_steps(4:6,i)) * tc_rfoot.pts + repmat(right_foot_steps(1:3,i),1,4);
  if all(cpos(3,:)<=0.001)
    plot(cpos(1,[1,2]),cpos(2,[1,2]),'k-','LineWidth',2);
    plot(cpos(1,[1,3]),cpos(2,[1,3]),'k-','LineWidth',2);
    plot(cpos(1,[2,4]),cpos(2,[2,4]),'k-','LineWidth',2);
    plot(cpos(1,[3,4]),cpos(2,[3,4]),'k-','LineWidth',2);
  end
end

for i=1:lstep_counter
  cpos = rpy2rotmat(lfoot_steps(4:6,i)) * tc_lfoot.pts + repmat(lfoot_steps(1:3,i),1,4);
  plot(cpos(1,[1,2]),cpos(2,[1,2]),'g-','LineWidth',1.65);
  plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',1.65);
  plot(cpos(1,[2,4]),cpos(2,[2,4]),'g-','LineWidth',1.65);
  plot(cpos(1,[3,4]),cpos(2,[3,4]),'g-','LineWidth',1.65);
end

for i=1:rstep_counter
  cpos = rpy2rotmat(rfoot_steps(4:6,i)) * tc_rfoot.pts + repmat(rfoot_steps(1:3,i),1,4);
  plot(cpos(1,[1,2]),cpos(2,[1,2]),'g-','LineWidth',1.65);
  plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',1.65);
  plot(cpos(1,[2,4]),cpos(2,[2,4]),'g-','LineWidth',1.65);
  plot(cpos(1,[3,4]),cpos(2,[3,4]),'g-','LineWidth',1.65);
end

plot(zmpdes(1,:),zmpdes(2,:),'b','LineWidth',3);
plot(zmpact(1,:),zmpact(2,:),'r.-','LineWidth',1);

axis equal;