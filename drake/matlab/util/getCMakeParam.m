function val = getCMakeParam(param)
% Checks the cmake cache and extracts the stored parameter

cache_file_name = fullfile(get_drake_binary_dir(),'CMakeCache.txt');
if ~exist(cache_file_name, 'file')
  error('Could not find CMakeCache.txt.')
end

txt = fileread(cache_file_name);
tokens = regexp(txt,[param,':\w+=(.*)'],'tokens','dotexceptnewline');
if isempty(tokens)
  val = [];
else
  val = strtrim(tokens{1}{1});
end
end
