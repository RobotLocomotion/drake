function M = read_gim(filename)

% read_gim - load a GIM from a .gim file
%
%   M = load_gim(filename);
%
%   Copyright (c) 2004 Gabriel Peyré

ext = filename(end-2:end);
ext = lower(ext);

if strcmp(ext, 'gim')
    fid = fopen(filename,'rb');
    if fid<0
        error(['No file ' filename, '.']);
    end
    M = fread(fid,'float');
    n = sqrt(length(M)/3);
    M = reshape(M,n,n,3);
    fclose(fid);
elseif strcmp(ext, 'png') || strcmp(ext, 'jpg') || strcmp(ext, 'bmp')
    M = double( imread(filename) );    
else
    error('Unknown format.');
end