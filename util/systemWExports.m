function [status,result] = systemWExports(command,objects)

% My overloaded implementation of the matlab system command, which calls
% ldd (or otool -L on mac) to find the required shared library directories,
% adds them to the LD_LIBRARY_PATH (or DYLD_LIBRARY_PATH) and then
% executes.
%
% @param command the same command that you would have sent to system
% @param optional object files to run ldd on to build the list
% of dependencies.  for example:
%   if you want to run epstopdf, which is a perl script, you should call
%     systemWExports('epstopdf','gs')
%   since gs (ghostscript) is the real dependency

if nargin<2, objects={command}; 
elseif ischar(objects), objects = {objects}; end
typecheck(objects,'cell');
for i=1:length(objects)
  [~,objects{i}] = system(['which ',strtok(objects{i})]);
end

libs = [];
if (ismac)
  LIBRARY_PATH = 'DYLD_LIBRARY_PATH';
  LDD = 'otool -L';
else
  LIBRARY_PATH = 'LD_LIBRARY_PATH';
  LDD = 'ldd';
end

for i=1:length(objects)
  [s,r] = system([LDD,' ',objects{i}]);
  [~,r] = strtok(r,':'); r=r(2:end);
  libs = [libs,r];
end

%libs

libdirs = {};
while true
  [lib,libs ] = strtok(libs,char(10));
  if isempty(lib) break; end
  libdirs = unique(vertcat(libdirs,strtrim(fileparts(lib))));
%  libdirs
end
if length(libdirs)>0
  libstr = libdirs{1};
  for i=2:length(libdirs)
    libstr = [libstr,':',libdirs{i}];
  end
  disp(['export ',LIBRARY_PATH,'=',libstr,'; ',command]);
  [status,result] = system(['export ',LIBRARY_PATH,'=',libstr,'; ',command]);
else
  [status,result] = system(command);
end
