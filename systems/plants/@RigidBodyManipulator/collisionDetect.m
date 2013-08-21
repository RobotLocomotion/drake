function [pos,vel,normal,mu] = collisionDetect(obj,contact_pos)
% for each column of contact_pos, find the closest point in the world
% geometry, and it's (absolute) position and velocity, (unit) surface
% normal, and coefficent of friction.
% @ingroup Collision

% Note: only implements collisions with the obj.terrain so far
[z,normal] = getHeight(obj.terrain,contact_pos(1:2,:));
pos = [contact_pos(1:2,:);z];

n = size(contact_pos,2);
vel = zeros(3,n); % static world assumption (for now)
mu = ones(1,n);

end
