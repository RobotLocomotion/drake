function sout=githubAPI(url,sin)
% struct in, struct out

mystack = dbstack;
path_to_this_mfile = fileparts(which(mystack(1).file));
addpath(fullfile(path_to_this_mfile,'jsonlab'));

if nargin>1 && ~isempty(sin),
  for f=fieldnames(sin)
    % handle json special characters
    str = sin.(f{1});
    str(str<9 | (str>10 & str<32))=[]; % zap special characters
    str = strrep(str,'\','\\');
    str = regexprep(str,'\n','\\n');
    str = regexprep(str,'\t','\\t');
    str = strrep(str,'"','\"');  % double quotes
    sin.(f{1}) = str;
  end

  j = savejson('',sin);
  j = strrep(j,'\\n','\n');
  j = strrep(j,'\\t','\t');
  %    j = strrep(j,char(134),'\"');
  j(j==139)=39;

  if length(j)>5000
    j = [j(1:4500),'\n\n**This post was going to be too long and has been truncated**"}'];
  end

  fname = tempname;
  fptr = fopen(fname,'w');
  fprintf(fptr,'%s',j);
  fclose(fptr);
  if ispc
    [~,fname]=system(['cygpath -u ',fname]);
    fname = strtrim(fname);
  end
  response = curl(['-d @',fname,' ''',url,''''])
else
  response = curl(['''',url,'''']);
end

if isempty(response)
  sout=struct();
  return;
end
try
  sout = loadjson(response);
catch
  if nargin>1
    disp(j);
  end
  disp(response)
  sout=struct();
  if exist('fname'), fname, end
end

end % githubAPI

function response = curl(command)

command = ['curl -s ',command];
if ~ispc
  command = ['export LD_LIBRARY_PATH=""; export DYLD_LIBRARY_PATH="";', command];
end
[exitval,response]=system(command);

if isempty(response)
  exitval
  command
end

end
