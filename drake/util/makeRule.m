function makeRule(left,right,varargin)

% MakeRule - basic make utility for matlab functions
%
%  Usage:  makeRule(left, right, command1, command2, ...)
%
%  Compares the file dates of the files in left with the filedates of files
%  in right.  If right is newer, then executes (and displays) command.
%  
%  @param left is a string, or cell array of strings
%  @param right is a string, or cell array of strings
%  @param command1, command2, ... are strings

if (~iscell(left)) left={left}; end
if (~iscell(right)) right={right}; end

for i=1:length(left), ldate(i) = fdate(left{i}); end
for i=1:length(right), rdate(i) = fdate(right{i}); end

if (max(rdate)>min(ldate))
  for i=1:length(varargin)
    disp(varargin{i});
    eval(varargin{i});
  end
end

end

function d = fdate(filename,bErr)

fi = dir(filename);
if (isempty(fi))
  if (nargin>1 && bErr) error('couldn''t find %s',filename); end
  d = 0;
else d = datenum(fi.date); end

end