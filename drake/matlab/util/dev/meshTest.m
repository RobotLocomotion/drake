clf;
load testmesh.mat;

pts=T*[pts(1:3,:);ones(1,size(pts,2))];

n=max(diff(find(ind==0)));
if (min(diff(find(ind==0)))~=n), error('need to handle this case'); end
ind = reshape(ind,n,[]); ind(end,:)=[]; 

%pts(3,:)=0; 
%pts=pts'; ind=ind';
%[pts,ind] = meshReduce(pts,ind);
%drawMesh(pts,ind);

%% collapse unique pts (most likely due to the flattening into 2D)
%[pts,ia,ic]=unique(pts','rows');
%ind=ic(ind);
pts=pts';

%x = pts(ind(:,1),1); y = pts(ind(:,1),2);
%[x,y] = poly2cw(x,y);
%for i=2:size(ind,2)
%  [x2,y2] = poly2cw(pts(ind(:,i),1),pts(ind(:,i),2));
%  [x,y] = polybool('union',x,y,x2,y2);
%  patch(x,y,.7*[1 1 1]); axis equal;
%  drawnow;
%end

%patch(x,y,.7*[1 1 1]);
%axis equal;
%return;

patchinfo = mesh2patch(pts,ind',[0;0;1]);
patchinfo.FaceColor=.7*[1 1 1];

subplot(2,1,2);
h=patch(patchinfo);
axis equal;

%% merge co-planar triangles which share an edge
%ind = sort(ind,1);


%% remove any polygons that will always be hidden (given the view axis)


%% plot

subplot(2,1,1);
x=pts(:,1); y=pts(:,2); z=pts(:,3);
h=patch(x(ind),y(ind),z(ind),.7*[1 1 1]);%,'FaceLighting','gouraud');
axis equal;

%h=patch(x,y,z,.7*[1 1 1]);
% remove now repeated triangles (note: i haven't seen this doing much):
%ind = unique(ind','rows')';

% extract the external edges:
%edges = [[ind(1,:);ind(2,:)],[ind(2,:);ind(3,:)],[ind(1,:);ind(3,:)]]';
%edges = [min(edges,[],2),max(edges,[],2)];  % put them in ascending order
%edges = sortrows(edges);
%dind=find(~any(diff(edges),2));  dind=dind(2:end);  % spare the first vertex
%edges(unique([dind;dind-1]),:)=[]; % zap every edge that has a double

%line(x(edges'),y(edges'),'Color','k');
%line(x(edges(dind,:)'),y(edges(dind,:)'),'Color','r');

%edges = computeMeshEdges(ind')';


