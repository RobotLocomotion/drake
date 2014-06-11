function [vertex,face] = read_smf(filename)

% read_smf - read data from SMF file.
%
%   [vertex,face] = read_smf(filename);
%
%   'vertex' is a 'nb.vert x 3' array specifying the position of the vertices.
%   'face' is a 'nb.face x 3' array specifying the connectivity of the mesh.
%
%   Copyright (c) 2004 Gabriel Peyré

fid = fopen(filename,'r');
if( fid==-1 )
    error('Can''t open the file.');
    return;
end

vertex = [];
face = [];

str = 0;
while ( str ~= -1)
    str = fgets(fid);   % -1 if eof
    if str(1)=='v'
        [a,str] = strtok(str);
        [a,str] = strtok(str); x = str2num(a);
        [a,str] = strtok(str); y = str2num(a);
        [a,str] = strtok(str); z = str2num(a);
        vertex = [vertex;[x y z]];
    elseif str(1)=='t' || str(1)=='f'
        [a,str] = strtok(str);
        [a,str] = strtok(str); x = str2num(a);
        [a,str] = strtok(str); y = str2num(a);
        [a,str] = strtok(str); z = str2num(a);
        face = [face;[x y z]];
    end
end

fclose(fid);