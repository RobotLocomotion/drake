function dpHat = dvectorToSkewSymmetric(dp)
% gradient version of vectorToSkewSymmetric
pToPHatVec = [...
  vectorToSkewSymmetric([-1; 0; 0]);
  vectorToSkewSymmetric([0; -1; 0]);
  vectorToSkewSymmetric([0; 0; -1])];
dpHat = pToPHatVec * dp;
end