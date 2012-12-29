function [pos,vel,normal,mu] = collisionDetect(obj,contact_pos)
% for each column of contact_pos, find the closest point in the world
% geometry, and it's (absolute) position and velocity, (unit) surface
% normal, and coefficent of friction.

% for now, just implement a ground height at y=0
n = size(contact_pos,2);
pos = [contact_pos(1:2,:);zeros(1,n)];
vel = zeros(3,n); % static world assumption (for now)
normal = [zeros(2,n); ones(1,n)];
mu = ones(1,n);

end
