function val = getCMakeParam(param)
% Checks the cmake cache and extracts the stored parameter

cache_file_name = fullfile(getDrakePath,'pod-build','CMakeCache.txt');
if ~exist(cache_file_name,'file')
  val = [];  % fail silently
  return;
end

txt = fileread(fullfile(getDrakePath,'pod-build','CMakeCache.txt'));
tokens = regexp(txt,[param,':\w+=(.*)'],'tokens','dotexceptnewline');
if isempty(tokens)
  val = [];
else
  val = strtrim(tokens{1}{1});
end

% Note: This was the old way of doing it.  But we found it less robust,
% because the matlab library path might not even allow one to run cmake.

%[retval,val] = system(['cmake -L -N ',fullfile(getDrakePath,'pod-build'),' | grep ', param,' | cut -d "=" -f2']);
%if (retval)
%  val=[];
%else
%  val = strtrim(val);
%end

end
