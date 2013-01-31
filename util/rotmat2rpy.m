function rpy = rotmat2rpy(R)

if sizecheck(R,[2 2])
  rpy = atan2(R(2,1),R(1,1));
else
  % NOTE: assumes we're using an X-Y-Z convention to construct R
  rpy = [atan2(R(3,2),R(3,3)); ...
    atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2)); ...
    atan2(R(2,1),R(1,1)) ];
end

