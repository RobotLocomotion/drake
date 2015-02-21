function urdfs = allURDFs(rootdir)
% @retval urdfs a cell matrix with the names of all of the URDFs in found under the root directory
%
% This is useful, for instance, for unit tests which want to test some
% basic operation on every urdf (e.g. in drake)

if nargin<1, rootdir = getDrakePath(); end

urdfs = {};
[info,p] = system(['find -L ',rootdir,' -iname "*.urdf" | grep -v "/dev/" | grep -v "irb_140_convhull"']);

if info==0
  while ~isempty(p)
    [pt,p]=strtok(p);
%    pt=fileparts(pt);
    p = regexprep(p,[pt,'.*\n'],'','dotexceptnewline');
    urdfs=vertcat(urdfs,pt);
  end
else  % if find fails for some reason (windows?), then do it the hard way...
  error('Drake:MissingDependency:find','still need to implement a version that works without find');
end

