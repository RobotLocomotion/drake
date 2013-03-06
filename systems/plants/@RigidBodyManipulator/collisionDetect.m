function [pos,vel,normal,mu] = collisionDetect(obj,contact_pos)
% for each column of contact_pos, find the closest point in the world
% geometry, and it's (absolute) position and velocity, (unit) surface
% normal, and coefficent of friction.

% for now, give the point on the heightmap directly below (or above) contact_pos
%
%  IMPORTANT NOTE:  This is not necessarily the closest point.  But it's
%  the simplest thing to implement first.
%

% flip to terrain coordinates
flipY = diag([1,-1,1,1]);
contact_pos = homogTransMult(flipY*obj.T_world_to_terrain,contact_pos);

%   ^ y
%  c|
%   |
%   |
%    -------> x
%   a     b
% a,b,c are all (effectively) integers
a = floor(contact_pos(1:2,:));
b = a; b(1,:)=b(1,:)+1;
c = a; c(2,:)=c(2,:)+1;

%   ^ y
%  c|\  |a    if beyond the diagonal, then flip a to the opposite corner
%   | \*|
%   |  \|
%    -------> x
%       b
tmp = contact_pos(1:2,:) - c;
to_flip = -tmp(1,:)<tmp(2,:);
a(:,to_flip) = a(:,to_flip)+1; % same as + repmat([1;1],1,sum(to_flip));

% compute barycentric coordinates
% http://en.wikipedia.org/wiki/Barycentric_coordinate_system_%28mathematics%29
% with r1=a,r2=b,r3=c
T_noflip = [-1 -1; 1 0];
T_flip = [1 1; 0 -1];
lambda=a;
lambda(:,~to_flip) = T_noflip*tmp(:,~to_flip);
lambda(:,to_flip) = T_flip*tmp(:,to_flip);
lambda(3,:) = 1-lambda(1,:)-lambda(2,:);

% todo: handle case where i'm off the mesh?  (with z = -inf?)
% if it happens now, the code below will error with "bad index"
s = size(obj.terrain_height);
az = obj.terrain_height(sub2ind(s,a(1,:)+1,a(2,:)+1));
bz = obj.terrain_height(sub2ind(s,b(1,:)+1,b(2,:)+1));
cz = obj.terrain_height(sub2ind(s,c(1,:)+1,c(2,:)+1));

z = lambda(1,:).*az + lambda(2,:).*bz + lambda(3,:).*cz;

n = size(contact_pos,2);
pos = [contact_pos(1:2,:);z];
vel = zeros(3,n); % static world assumption (for now)

normal = cross([b;bz]-[a;az],[c;cz]-[a;az]);  %[zeros(2,n); ones(1,n)];
normal(:,to_flip) = -normal(:,to_flip);

mu = ones(1,n);

% convert back to world coordinates
pos = homogTransMult(obj.T_terrain_to_world*flipY,pos);
normal = homogTransMult(obj.T_terrain_to_world*flipY,normal)-repmat(homogTransMult(obj.T_terrain_to_world,zeros(3,1)),1,size(contact_pos,2));

% normalize normals
normal = normal./repmat(sqrt(sum(normal.^2,1)),3,1);

end
