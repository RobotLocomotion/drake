function [phi,normal,xB,idxB] = collisionDetectTerrain(obj, xA_in_world)
  [z,normal] = getHeight(obj.terrain,xA_in_world(1:2,:));
  xB = [xA_in_world(1:2,:);z];
  idxB = ones(1,size(xA_in_world,2));
  %phi = sqrt(sum((xA_in_world-xB).^2))';
  phi = (xA_in_world(3,:)-z)';
end
