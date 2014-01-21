function twist = relativeTwist(obj, kinsol, base, endEffector, expressedIn)
% note: need obj for mex pointer info in the future

baseTwist = kinsol.twist{base};
endEffectorTwist = kinsol.twist{endEffector};

twist = ...
  transformTwists(kinsol.T{expressedIn} \ kinsol.T{endEffector}, endEffectorTwist) - ...
  transformTwists(kinsol.T{expressedIn} \ kinsol.T{base}, baseTwist);

end