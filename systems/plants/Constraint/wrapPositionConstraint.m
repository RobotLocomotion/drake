function kc_cell = wrapPositionConstraint(robot,body,pts,pos)
if(isstruct(pos))
  posmax = pos.max;
  posmin = pos.min;
else
  posmax = pos;
  posmin = pos;
end
rows = size(posmax,1);
if(body == 0)
  kc_cell = {WorldCoMConstraint(robot,[],posmin,posmax)};
else
  kc1 = WorldPositionConstraint(robot,[],body,pts,posmin(1:3,:),posmax(1:3,:));
  if(rows == 3)
    kc_cell = {kc1};
  else
    % How should I handle the case of inf or nan in the orientation?
    if(~(any(isinf(posmax(4:end,1))|isinf(posmin(4:end,1))|isnan(posmin(4:end,1))|isnan(posmax(4:end,1)))))
      if(rows == 6)
        rpymax = posmax(4:6);
        rpymin = posmin(4:6);
        diff_rpy_idx = rpymax~=rpymin;
        num_diff_rpy = sum(diff_rpy_idx);
        if(num_diff_rpy == 1)
          % treat as a gaze constraint
          quat_des = rpy2quat(0.5*(rpymin+rpymax));
          gaze_axis = diff_rpy_idx;
          threshold = min(0.5*(rpymax-rpymin),pi);
          kc2 = WorldGazeOrientConstraint(robot,[],body,gaze_axis,quat_des,0,threshold);
        elseif(num_diff_rpy == 0)
          quat_des = rpy2quat(rpymin);
          tol = 0;
          kc2 = WorldOrientConstraint(robot,[],body,quat_des,tol);
        else
          quat_des = rpy2quat(0.5*(rpymin+rpymax));
          tol = max([1-(rpy2quat(rpymin)'*quat_des)^2 1-(rpy2quat(rpymax)'*quat_des)^2]);
          kc2 = WorldOrientConstraint(robot,[],body,quat_des,tol);
        end
        kc_cell = {kc1,kc2};
      elseif(rows == 7)
        if(all(posmin(4:7,1)==posmax(4:7,1)))
          quat_des = posmin(4:7,1);
          tol = 0;
        else
          quat_des = 0.5*(posmin(4:7,1)+posmax(4:7,1));
          tol = max([1-(posmin(4:7,1)'*quat_des)^2 1-(posmax(4:7,1)'*quat_des)^2]);
        end
        kc2 = WorldOrientConstraint(robot,[],body,quat_des,tol);
        kc_cell = {kc1,kc2};
      end
    else
      kc_cell = {kc1};
    end
  end
end
end