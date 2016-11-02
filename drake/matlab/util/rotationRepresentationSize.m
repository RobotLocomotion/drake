function ret = rotationRepresentationSize(rotation_type)
% @param rotation_type: 0: no rotation, 1: rpy, 2: quaternion
% @retval ret size of rotation representation

switch rotation_type
  case 0
    ret = 0;
  case 1
    ret = 3;
  case 2
    ret = 4;
  otherwise
    error('rotation type not recognized');
end
end
