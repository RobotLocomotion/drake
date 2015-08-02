function [vertex,face] = read_tet(filename)

% read_tet - read data from TET file.
%
%   [vertex,face] = read_test(filename);
%
%   'vertex' is a '3 x nb.vert' array specifying the position of the vertices.
%   'face' is a '4 x nb.face' array specifying the connectivity of the tet mesh.
%
%   Copyright (c) 2008 Gabriel Peyre


fid = fopen(filename,'r');
if( fid==-1 )
    error('Can''t open the file.');
    return;
end

[nvert,cnt] = fscanf(fid,'%d vertices', 1);
[nface,cnt] = fscanf(fid,'%d tets', 1);


% str = fgets(fid);
% [a,str] = strtok(str); nvert = str2num(a);
% [a,str] = strtok(str); nface = str2num(a);


[A,cnt] = fscanf(fid,'%f %f %f', 3*nvert);
if cnt~=3*nvert
    warning('Problem in reading vertices.');
end
vertex = reshape(A, 3, cnt/3);
% read Face 1  1088 480 1022
[A,cnt] = fscanf(fid,'%d %d %d %d\n', 5*nface);
if cnt~=5*nface
    warning('Problem in reading faces.');
end
A = reshape(A, 5, cnt/5);
face = A(2:5,:)+1;

fclose(fid);


% rescale
vertex = vertex - repmat(min(vertex,[],2),[1 nvert]);
vertex = vertex ./ repmat(max(vertex(:)),[3 nvert]);