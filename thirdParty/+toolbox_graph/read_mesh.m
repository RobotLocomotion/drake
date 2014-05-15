function [vertex,face,normal, uv, sphparam] = read_mesh(file)

% read_mesh - read data from OFF, PLY, WRL, OBJ, TET, M or GIM file.
%
%   [vertex,face] = read_mesh(filename);
%   [vertex,face] = read_mesh;      % open a dialog box
%
%   'vertex' is a 'nb.vert x 3' array specifying the position of the vertices.
%   'face' is a 'nb.face x 3' array specifying the connectivity of the mesh.
%
%   Supported file extensions are: .off, .ply, .wrl, .obj, .tet, .m, .gim.
%
%   Copyright (c) 2007 Gabriel Peyre

import toolbox_graph.*;

switch file
    case {'triangle' 'square' 'square1' 'L' 'L1' 'tetra' 'oct' 'ico' 'rand'}
        [vertex,face] = compute_base_mesh(file);
        [vertex,face] = check_face_vertex(vertex,face);
        return;
end

if nargin==0
    [f, pathname] = uigetfile({'*.off;*.ply;*.wrl;*.obj;*.tet;*.m;*.gim','*.off,*.ply,*.wrl,*.obj,*.tet,*.m,*.gim Files'},'Pick a file');
    file = [pathname,f];
end

supported_ext = {'.off' '.ply' '.wrl' '.obj' '.tet' '.m' '.gim'};
[p,name,ext]=fileparts(file);

if isempty(ext)
  % try to determine extension
  for i=1:length(supported_ext)
    if exist([file supported_ext{i}])==2
      ext = supported_ext{i};
      disp(['Loading file with extension ',ext]);
      break;
    end
  end
  if isempty(ext)
    error('File not found with matching extension.');
  end
  file = fullfile(p,[name,ext]);
end

switch lower(ext)
    case '.off'
        [vertex,face] = read_off(file);
    case '.ply'
        [vertex,face] = read_ply(file);
    case '.smf'
        [vertex,face] = read_smf(file);
    case '.wrl'
        [vertex,face] = read_wrl(file);
    case '.obj'
        [vertex,face] = read_obj(file);
    case '.tet'
        [vertex,face] = read_tet(file);
    case '.m'
        if isfield(options, 'type')
            type = options.type;
        else
            type = 'gim';
        end
        [vertex,face,normal, uv, sphparam] = read_mfile(file, type);
    case '.gim'
        sub_sample = 1;
        [M,Normal] = load_gim(name, options);
        [vertex,face] = convert_gim2mesh(M, sub_sample);
        normal = convert_gim2mesh(Normal, sub_sample);
        
    otherwise
        error('Unknown extension.');
end

if strcmp(name, 'mannequin')
    vertex = -vertex;
end
