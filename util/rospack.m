function path = rospack(package)

[info,path]=system(['rospack find ',package]);
if info==0  % success!
  path = regexprep(path,'\n','');
else
  error(path);
end

