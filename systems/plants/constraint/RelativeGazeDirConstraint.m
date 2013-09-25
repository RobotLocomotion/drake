classdef RelativeGazeDirConstraint < GazeDirConstraint
  
  methods  
    function obj = RelativeGazeDirConstraint(robot,body_a_idx, ...
                                             body_b_idx,axis,dir, ...
                                             conethreshold,tspan)
      % dir is a 3x1 vector in Frame B
      typecheck(dir,'double');
      sizecheck(dir,[3,1]);
      typecheck(axis,'double');
      sizecheck(axis,[3,1]);
      len_dir = norm(dir);
      if(len_dir == 0)
        error('Drake:RelativeGazeDirConstraint: gaze direction has 0 length');
      end
      dir = dir/len_dir;
      len_axis = norm(axis);
      if(len_axis == 0)
        error('Drake:RelativeGazeDirConstraint: gaze axis has 0 length');
      end
      axis = axis/len_axis;
      quat_des = quatTransform(axis,dir);
      obj = obj@GazeDirConstraint(robot,body_a_idx,body_b_idx,axis,quat_des,conethreshold,tspan);
    end
  end
end
