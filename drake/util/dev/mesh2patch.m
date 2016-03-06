function patchinfo=mesh2patch(vertices,faces,view_axis)

% Takes a 3D triangulated mesh, and returns a set of x,y,z points that can
% be handed to patch for 2D visualization.  The primary operations are
%
%  (a) removing faces with normals orthogonal to the viewing axis
%
%  (b) joining coplanar triangles into a single polygon (so that patch does
%  not draw the edge)
% 
%  (c) removing polygons that are completely covered (so will never be
%  seen)
%
% @param vertices an Nx3 double vector of 3D points
% @param faces an Mx3 integer vector of vertex indices
% @param view_axis a 3x1 vector specifying the vector out of the screen.
% 
% @retval x,y,z matrices of polygons suitable for use as in
%    patch(x,y,z,color)
%

N = size(vertices,1);
sizecheck(vertices,[N,3]);

M = size(faces,1);
sizecheck(faces,[M,3]);

sizecheck(view_axis,[3 1]);

coplanar_tolerance = 1e-10; 

%% remove faces that are out-of-plane
a=vertices(faces(:,2),:)-vertices(faces(:,1),:);
b=vertices(faces(:,3),:)-vertices(faces(:,1),:);
normal=cross(a,b)./repmat(sum(a.*a,2).*sum(b.*b,2),1,3);
faces(abs(normal*view_axis)<coplanar_tolerance,:)=[];

%% collapse overlapping points
[vertices,ia,ic]=unique(vertices,'rows');
faces=ic(faces);

%% remove unused vertices
[f,ia,ic]=unique(faces(:));
vertices=vertices(f,:); 
N = size(vertices,1);
h(f)=1:N;  faces=h(faces);
M = size(faces,1);

patchinfo.Vertices=vertices;
patchinfo.Faces=faces;

%% join coplanar polygons with a common edge
edges = [[faces(:,1),faces(:,2)];[faces(:,2),faces(:,3)];[faces(:,3),faces(:,1)]];
edgeflip = edges(:,1)>edges(:,2);
edges(edgeflip,:) = edges(edgeflip,[2 1]);  % put them in ascending order

[sedges,edgeind] = sortrows(edges); faceind=mod(edgeind-1,M)+1; %edgeind=floor(ind/M)+1;
doublesind=find(~any(diff(sedges),2));  % find the doubles
doublesind=doublesind(2:end);  % the first edges scores diff=0, but it doesn't count

  % make sure the edge e(1) comes directly before e(2) in both f1 and f2
  function [f,a,b]=reorder(f,e)
    a=find(f==e(1),1); b=find(f==e(2),1); L=length(f);
    if (a>b && ~(b==1 && a==L)) f=fliplr(f); a=L-a+1; b=L-b+1;
    elseif (a==L && b==1), f=f([end,1:end-1]); a=1;b=2;
    elseif (b==L && a==1), f=f([1,end:-1:2]); a=1;b=2; end
  end

for i=1:length(doublesind)
  face1 = faceind(doublesind(i));
  face2 = faceind(doublesind(i)+1);
  if (face1==face2) 
    f1 = faces(face1,:); f1 = f1(f1>0);  if isempty(f1), error('oops!'); end
%    clf;
%    subplot(2,1,1);
%    patch(vertices(f1,1),vertices(f1,2),vertices(f1,3),[1 0 0]); 

    e=edges(edgeind(doublesind(i)),:);
    [f1,a1,b1]=reorder(f1,e);
    % zap one instance of edge:
    newface=f1([1:a1-1,b1+1:end]);
    
%    subplot(2,1,2);
%    patch(vertices(newface,1),vertices(newface,2),vertices(newface,3),.7*[1 1 1]);
%    hold on;
%    plot(vertices(e,1),vertices(e,2),'b+');
    
    if length(newface)>60 pause; end
    
    faces(face1,1:length(newface)) = newface;
  else
    f1 = faces(face1,:); f1 = f1(f1>0);  if isempty(f1), error('oops!'); end
    f2 = faces(face2,:); f2 = f2(f2>0);  if isempty(f2), error('oops!'); end

%    clf;
%    subplot(2,1,1);
%    patch(vertices(f1,1),vertices(f1,2),vertices(f1,3),[1 0 0]);
%    patch(vertices(f2,1),vertices(f2,2),vertices(f2,3),[0 1 0]);
    
    % check if the two faces are coplanar
    % http://mathworld.wolfram.com/Coplanar.html
    v = vertices(unique([f1,f2],'rows'),:);
    if (size(v,1)<4) error('something went wrong here'); end
    % check the first 4 points:
    if (abs(det([v(1:4,:),ones(4,1)]))>coplanar_tolerance), continue; end
    % check the rest
    if any(abs(cross(v(2,:)-v(1,:),v(3,:)-v(1,:))*(v(5:end,:)-repmat(v(1,:),size(v,1)-4,1))')>coplanar_tolerance), continue; end
    % if i get here, then they are coplanar
    
    % put the two faces together, and remove the overlapping edges.
    %   e1 = edgeind(doublesind(i));
    %   e2 = edgeind(doublesind(i)+1);
    %   if edgeflip(e1), f1 = fliplr(f1); end
    %   if edgeflip(e2), f2 = fliplr(f2); end
    e=edges(edgeind(doublesind(i)),:);
    
    % make sure the edge e(1) comes directly before e(2) in both f1 and f2
    [f1,a1,b1]=reorder(f1,e);
    [f2,a2,b2]=reorder(f2,e);
    
    newface = [f1(1:a1),f2([a2-1:-1:1,end:-1:b2+1]),f1(b1:end)];
    
    %% debugging

    %  axis equal;
%    subplot(2,1,2);
%    patch(vertices(newface,1),vertices(newface,2),vertices(newface,3),.7*[1 1 1]);
%    hold on;
%    plot(vertices(e,1),vertices(e,2),'b+');
%    %  axis equal;
%    i,newface
%    drawnow;
%    if length(newface)>60 pause; end
    
    % write newface back into faces
    if (length(newface)>size(faces,2))
      faces(1,length(newface))=0;  % extends the entire matrix
    end
    faces(face1,1:length(newface)) = newface;
    faces(face2,:) = 0;
    
    % update faceind
    faceind(faceind==face2)=face1;
  end
end

%% handle zeros
faces(all(faces==0,2),:)=[]; % remove rows with all zeros

% close all of the faces (by copying the last element in each row to the
% front)
faces=[faces(sub2ind(size(faces),(1:size(faces,1))',sum(faces~=0,2))),faces];

%vertices(N+1,:)=nan;
%faces(faces==0)=N+1;
faces(faces==0)=nan;

patchinfo.Vertices=vertices;
patchinfo.Faces=faces;

return;

%% return values
x = vertices(:,1); x = x(faces');
y = vertices(:,2); y = y(faces');
z = vertices(:,3); z = z(faces');

end
