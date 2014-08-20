function val = getCMakeParam(param)
% Checks the cmake cache and extracts the stored parameter

txt = fileread(fullfile(getDrakePath,'pod-build','CMakeCache.txt'));
if isempty(txt)
  error('Couldn''t find the cmake cache.  Did you run make?');
end
tokens = regexp(txt,[param,':\w+=(.*)'],'tokens','dotexceptnewline');
if isempty(tokens)
  val = [];
else
  val = tokens{1}{1};
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
