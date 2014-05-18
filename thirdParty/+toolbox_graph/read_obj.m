function [vertex,faces] = read_obj(filename)

% read_obj - load a .obj file.
%
%   [vertex,face,normal] = read_obj(filename);
%
%   faces    : list of facesangle elements
%   vertex  : node vertexinatates
%   normal : normal vector list
%
%   Copyright (c) 2008 Gabriel Peyre

fid = fopen(filename);
if fid<0
    error(['Cannot open ' filename '.']);
end

frewind(fid);
a = fscanf(fid,'%c',1);
if strcmp(a, 'P')
    % This is the montreal neurological institute (MNI) specific ASCII facesangular mesh data structure.
    % For FreeSurfer software, a slightly different data input coding is
    % needed. It will be provided upon request.
    fscanf(fid,'%f',5);
    n_points=fscanf(fid,'%i',1);
    vertex=fscanf(fid,'%f',[3,n_points]);
    normal=fscanf(fid,'%f',[3,n_points]);
    n_faces=fscanf(fid,'%i',1);
    fscanf(fid,'%i',5+n_faces);
    faces=fscanf(fid,'%i',[3,n_faces])'+1;
    fclose(fid);
    return;
end

frewind(fid);
vertex = [];
faces = [];
while 1
    s = fgetl(fid);
    if ~ischar(s), 
        break;
    end
    if ~isempty(s) && strcmp(s(1), 'f')
        % face
        s = regexprep(s, '//[0-9]*', '');
        faces(:,end+1) = sscanf(s(3:end), '%d %d %d');
    end
    if ~isempty(s) && strcmp(s(1), 'v') && strcmp(s(2), ' ')
        % vertex
        vertex(:,end+1) = sscanf(s(3:end), '%f %f %f');
    end
end
fclose(fid);

