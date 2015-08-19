function writeMeshOBJ(filename,x,y,Z)

% writes the mesh to an alias wavefront file (e.g. for the viewers to
% parse)
% adapted from http://www.aleph.se/Nada/Ray/saveobjmesh.m
% @param filename string indicating where to write the file
% @param x,y are *vectors* 
% @param Z is a matrix of size [numel(y),numel(x)]
%  (note the somewhat flipped order of y and x)
%  which mirrors the syntax of mesh(x,y,Z);

typecheck(filename,'char');
assert(isvector(x));
assert(isvector(y));
assert(all(size(Z)==[numel(y),numel(x)]));
Z=Z';  % back to ndgrid vs meshgrid (for sanity + compatibility w/ barycentric code)

[X,Y] = ndgrid(x,y);

% compute normals
N = numel(X);
[indices,coefs,dcoefs] = barycentricInterpolation({x,y},[X(:)';Y(:)']);
fx = sum(Z(indices).*reshape(dcoefs(:,1),size(indices,1),N));
fy = sum(Z(indices).*reshape(dcoefs(:,2),size(indices,1),N));
%normals = [-fx;-fy;ones(1,N)];
nx = reshape(-fx,size(X));
ny = reshape(-fy,size(X));
nz = ones(size(X));
% end compute normals

l=size(X,1); h=size(X,2);

n=zeros(l,h);

[path,name,ext] = fileparts(filename);
if isempty(ext), ext='.obj'; end
filename = fullfile(path,[name,ext]);

fid=fopen(filename,'w');

n=reshape(1:(l*h),l,h);
[i,j]=ndgrid(linspace(0,1,l),linspace(0,1,h));
fprintf(fid,'v %f %f %f\n',[X(:)';Y(:)';Z(:)']);
fprintf(fid,'vt %f %f\n',[i(:)';j(:)']);
fprintf(fid,'g mesh\n');
fprintf(fid,'f %d/%d %d/%d %d/%d %d/%d\n',[reshape(n(1:end-1,1:end-1),1,[]);reshape(n(1:end-1,1:end-1),1,[]);reshape(n(2:end,1:end-1),1,[]);reshape(n(2:end,1:end-1),1,[]);reshape(n(2:end,2:end),1,[]);reshape(n(2:end,2:end),1,[]);reshape(n(1:end-1,2:end),1,[]);reshape(n(1:end-1,2:end),1,[])]);
fprintf(fid,'g\n\n');
fclose(fid);

end