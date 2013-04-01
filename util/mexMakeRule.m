function mexMakeRule(sourcefile,additionalargs,additionaldeps)

% equivalent to a makefile line
%   source.mexext : sourcefile additionaldeps{:}
%         mex sourcefile additionalargs{:}
% where source.mexext is the sourcefile name with the extension replaced

if (nargin<2) additionalargs={}; end
if (nargin<3) additionaldeps={}; end

typecheck(sourcefile,'char');
typecheck(additionalargs,'cell');
typecheck(additionaldeps,'cell');

[path,file,ext] = fileparts(sourcefile);
cmd=['mex ',sourcefile,' ',sprintf('%s ',additionalargs{:})];
makeRule(fullfile(path,[file,'.',mexext]),horzcat(sourcefile,additionaldeps),cmd);
