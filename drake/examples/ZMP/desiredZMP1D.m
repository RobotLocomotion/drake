function fr = desiredZMP1D

fr = SingletonCoordinateFrame('desiredZMP1D',1,'z',{'x_zmp'});
if isempty(findTransform(fr,desiredZMP2D))
  addTransform(fr,AffineTransform(fr,desiredZMP2D,[1;0],[0;0]));
end

% NOTEST