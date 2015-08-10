%function extractPatch

% todo:  move this logic into RigidBodyMesh when it works

x_axis=[1;0;0]; y_axis=[0;0;1]; view_axis=[0;1;0];
Rview = [x_axis, y_axis, view_axis]';

clf
mesh = RigidBodyMesh('../meshes/GliderFuselage.STL');

[pts,ind,normals] = loadFile(mesh);

n=4;  % only handle triangle meshes to starts
assert(n==max(diff(find(ind==0))));
assert(n==min(diff(find(ind==0))));

ind = reshape(ind,n,[]); ind(end,:)=[];
%ind = ind(:,normals'*view_axis<0);  % throw away faces with normals pointing into the page
%pts2 = pts(:,ind);

x = pts(

patch(pts(1,:)',pts(2,:)',pts(3,:)',.7*[1 1 1]);
%plot3(pts(1,:)',pts(2,:)',pts(3,:)','.');

%end

