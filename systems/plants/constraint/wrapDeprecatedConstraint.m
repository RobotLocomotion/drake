function [kc_cell,qsc_pts] = wrapDeprecatedConstraint(robot,body,pts,pos,options)
% wrap the deprecated form of constraint used in IK to an object in
% Constraint class
% @param robot              The RigidBodyManipulator or TimeSteppingRigidBodyManipulator
% @param body               The body index
% @param pts                body points, a 3xn_pts double matrix
% @param pos                Can be a 3xn_pts, 6xn_pts or 7xn_pts double matrix
%                           It can also have field 'max' 'min' for a box
%                           constraint on position and orientation. Also it
%                           can have field 'gaze' and 'contact_state'
% @param options   -- use_mex   A boolean flag, set to true would construct
%                               mex constraint object only, default is true
%                  -- tspan     A 1x2 double vector, the time span of the
%                               constraint. default is [-inf inf];
if(~isstruct(options))
  error('The last argument option must be a struct')
end
if(~isfield(options,'use_mex'))
  options.use_mex = true;
end
if(isfield(options,'tspan'))
  tspan = options.tspan;
else
  tspan = [-inf inf];
end
if(isfield(options,'robotnum'))
  robotnum = options.robotnum;
else
  robotnum = 1;
end
if(isa(robot,'RigidBodyManipulator') || isa(robot,'TimeSteppingRigidBodyManipulator'))
  robot_ptr = robot.getMexModelPtr;
elseif(isa(robot,'DrakeMexPointer'))
  robot_ptr = robot;
end
qsc_pts = [];
if(isstruct(pos)&&isfield(pos,'type'))
  if(strcmp(pos.type,'gaze'))
    if(isfield(pos,'gaze_conethreshold'))
      conethreshold = pos.gaze_conethreshold;
    else
      conethreshold = [];
    end
    if(isfield(pos,'gaze_threshold'))
      threshold = pos.gaze_threshold;
    else
      threshold = [];
    end
    axis = pos.gaze_axis;
    if(isfield(pos,'gaze_orientation'))
      if(length(pos.gaze_orientation) == 3)
        quat_des = rpy2quat(pos.gaze_orientation);
      else
        quat_des = pos.gaze_orientation/norm(pos.gaze_orientation);
      end
      kc_cell = {constructRigidBodyConstraint(RigidBodyConstraint.WorldGazeOrientConstraintType,options.use_mex,robot,body,axis,quat_des,conethreshold,threshold,tspan)};
    end
    if(isfield(pos,'gaze_target'))
      gaze_target = pos.gaze_target;
      kc_cell = {constructRigidBodyConstraint(RigidBodyConstraint.WorldGazeTargetConstraintType,options.use_mex,robot,body,axis,gaze_target,[0;0;0],conethreshold,tspan)};
    end
    if(isfield(pos,'gaze_dir'))
      gaze_dir = pos.gaze_dir./norm(pos.gaze_dir);
      kc_cell = {constructRigidBodyConstraint(RigidBodyConstraint.WorldGazeDirConstraintType,options.use_mex,robot,body,axis,gaze_dir,conethreshold,tspan)};
    end
  end
else
  if(isstruct(pos))
    posmax = pos.max;
    posmin = pos.min;
  else
    posmax = pos;
    posmin = pos;
  end
  posmax(isnan(posmax)) = inf;
  posmin(isnan(posmin)) = -inf;
  rows = size(posmax,1);
  if(body == 0)
    kc_cell = {constructRigidBodyConstraint(RigidBodyConstraint.WorldCoMConstraintType,options.use_mex,robot,posmin,posmax,tspan,robotnum)};
  else
    if(ischar(pts))
      pts = robot.getBody(body).getContactPoints(pts);
    end
    kc1 = constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,options.use_mex,robot,body,pts,posmin(1:3,:),posmax(1:3,:),tspan);
    qsc_pts_idx = false(1,size(pts,2));
    if(isfield(pos,'contact_state'))
      if(iscell(pos.contact_state))
        for contact_aff_idx = 1:length(pos.contact_state)
          qsc_pts_idx = (pos.contact_state{contact_aff_idx} == 3)|qsc_pts_idx;
        end
      else
        qsc_pts_idx = (pos.contact_state == 3)|qsc_pts_idx;
      end
    end
    qsc_pts_idx = (abs(posmin(3,:))<1e-6)&(abs(posmax(3,:))<1e-6)|qsc_pts_idx;
    qsc_pts = pts(:,qsc_pts_idx);
    if(rows == 3)
      kc_cell = {kc1};
    elseif(rows == 6)
      rpymax = posmax(4:6);
      rpymin = posmin(4:6);
      kc2 = constructRigidBodyConstraint(RigidBodyConstraint.WorldEulerConstraintType,options.use_mex,robot,body,rpymin,rpymax,tspan);
      kc_cell = {kc1,kc2};
    elseif(rows == 7)
      if(any(isinf(posmax(4:7,1))|isinf(posmin(4:7,1))))
        kc_cell = {kc1};
        warning('The quaternion bounds have value nan or inf, are you sure this is a correct constraint?');
      else
        quatmax = min([posmax(4:7,1) ones(4,1)],[],2);
        quatmin = max([posmin(4:7,1) -ones(4,1)],[],2);
        if(all(quatmin==quatmax))
          quat_des = quatmin;
          tol = 0;
        else
          quat_des = 0.5*(quatmin+quatmax);
          tol = max([1-(quatmin'*quat_des)^2 1-(quatmax'*quat_des)^2]);
        end
        kc2 = constructRigidBodyConstraint(RigidBodyConstraint.WorldQuatConstraintType,options.use_mex,robot,body,quat_des,tol,tspan);
        kc_cell = {kc1,kc2};
      end
    end
  end
end
end