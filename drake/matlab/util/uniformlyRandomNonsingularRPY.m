function rpy = uniformlyRandomNonsingularRPY()
% via rejection sampling

rpy = uniformlyRandomRPY();
while abs(abs(rpy(2)) - pi / 2) < 0.15 % then i'm close to the singularity
  rpy = uniformlyRandomRPY();
end

end