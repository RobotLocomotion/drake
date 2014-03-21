function [zmptraj,lfoottraj,rfoottraj,supporttraj] = ZMPandFootTrajectory(r,q0,num_steps,step_length,step_time)

% @param r is RigidBodyManipulator/TimeSteppingRigidBodyManipulator
% @param q0 is the initial pos
% @param num_steps the number of discrete foot steps (right foot is 1 step,
% right foot + left foot is 2 steps)
% @param step_length is forward displacement of the foot on a single step
%   note: the first and last steps will be half this distance, to start and
%   stop
% @param step_time is the total time taken to move from zmp at center, to
% the center of the foot, to back at center (but again for just one foot)

% NOTEST

typecheck(r,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
typecheck(q0,'numeric');
sizecheck(q0,[r.getNumDOF,1]);
sizecheck(num_steps,1);
rangecheck(num_steps,0,inf);
sizecheck(step_length,1);
sizecheck(step_time,1);

kinsol = doKinematics(r,q0);
rfoot_body = findLinkInd(r,'r_foot');
lfoot_body = findLinkInd(r,'l_foot');

com0 = getCOM(r,q0);
rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0],true);
lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0],true);

gc = r.contactPositions(q0);

% compute desired COM projection
% assumes minimal contact model for now
k = convhull(gc(1:2,1:4)');
lfootcen0 = [mean(gc(1:2,k),2);0];
k = convhull(gc(1:2,5:8)');
rfootcen0 = [mean(gc(1:2,4+k),2);0];
roffset = rfootcen0 - rfoot0(1:3) + [0;0.02;0];
loffset = lfootcen0 - lfoot0(1:3) + [0;-0.02;0];

function pos = rfootCenter(rfootpos)
  % orientation of foot is always zero in this demo
  pos = rfootpos(1:2)+roffset(1:2);
end    
  
function pos = lfootCenter(lfootpos)
  % orientation of foot is always zero in this demo
  pos = lfootpos(1:2)+loffset(1:2);
end

function pos = feetCenter(rfootpos,lfootpos)
  % orientation of foot is always zero in this demo
  rcen = rfootpos(1:2)+roffset(1:2);
  lcen = lfootpos(1:2)+loffset(1:2);
  pos = mean([rcen,lcen],2);
end

step_length = repmat(step_length,1,num_steps);
step_length(1) = step_length(1)/2;
step_length(end) = step_length(end)/2;

% get rid of offset from initial conditions
step_length(1) = step_length(1)+rfoot0(1)-lfoot0(1);

% move from initial conditions to zmp at the center of the support polygon
ts = [0,.5];
rfootpos = [rfoot0,rfoot0];
lfootpos = [lfoot0,lfoot0];
zmp = [com0(1:2),feetCenter(rfootpos(:,2),lfootpos(:,2))];

rfootsupport = [1 1];
lfootsupport = [1 1];

bRightStep = true;
for istep=1:num_steps
  lf = repmat(lfootpos(:,end),1,4);
  rf = repmat(rfootpos(:,end),1,4);
  tstep = ts(end)+[.1,.5,.925,.975,1]*step_time;
  if (bRightStep)
    rf(1,:) = rf(1,:)+[0,step_length(istep)/2,step_length(istep),step_length(istep)];
    rf(3,:) = rf(3,:)+[0,.05,0,0];
    stepzmp = [repmat(lfootCenter(lf(:,1)),1,3),feetCenter(rf(:,end),lf(:,end))];
    rfootsupport = [rfootsupport 0 0 0.5 1 1]; 
    lfootsupport = [lfootsupport 1 1 1 1 1]; 
  else
    lf(1,:) = lf(1,:)+[0,step_length(istep)/2,step_length(istep),step_length(istep)];
    lf(3,:) = lf(3,:)+[0,.05,0,0];
    stepzmp = [repmat(rfootCenter(rf(:,1)),1,3),feetCenter(rf(:,end),lf(:,end))];
    rfootsupport = [rfootsupport 1 1 1 1 1]; 
    lfootsupport = [lfootsupport 0 0 0.5 1 1]; 
  end
  rfootpos = [rfootpos,rf,rf(:,end)];
  lfootpos = [lfootpos,lf,lf(:,end)];
  zmp = [zmp,stepzmp,stepzmp(:,end)];
  ts = [ts,tstep];
  bRightStep = ~bRightStep;
end

% add a segment at the end to recover
ts = [ts, ts(end)+2];
rfootpos = [rfootpos,rfootpos(:,end)];
lfootpos = [lfootpos,lfootpos(:,end)];
zmp = [zmp,feetCenter(rfootpos(:,end),lfootpos(:,end))];
rfootsupport = [rfootsupport 1]; 
lfootsupport = [lfootsupport 1]; 

zmptraj = PPTrajectory(foh(ts,zmp));
lfoottraj = PPTrajectory(foh(ts,lfootpos));
rfoottraj = PPTrajectory(foh(ts,rfootpos));

% create support body trajectory
supporttraj = repmat(0*ts,length(r.getLinkNames),1);
supporttraj(strcmp('r_foot',r.getLinkNames),:) = rfootsupport;
supporttraj(strcmp('l_foot',r.getLinkNames),:) = lfootsupport;
supporttraj = setOutputFrame(PPTrajectory(zoh(ts,supporttraj)),AtlasBody(r));

end
