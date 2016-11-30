function codeCheck(options)

% run mlint and a series of style checks on all available m files 
% in the current directory tree
%
% @option mlint_ignore_id cell array of mlint IDs to safely ignore @default {}
%
% see
% http://www.mathworks.com/help/matlab/matlab_prog/check-code-for-errors-and-warnings.html#brqxeeu-167 
% for information about suppressing warnings

% todo in doc
% @option interactive @default false
%
% reference code style guide

if nargin<1, options=struct(); end

[~,filestr] = system('find . -iname "*.m" | grep -v /dev/');
files = strread(filestr,'%s\n');

printInstructions;

if isfield(options,'mlint_ignore_id')
  typecheck(options.mlint_ignore_id,{'cell','char'});
  if ~iscell(options.mlint_ignore_id)
    options.mlint_ignore_id = {options.mlint_ignore_id};
  end
  mlint_ignore_id = upper(options.mlint_ignore_id);
else
  mlint_ignore_id = {'STOUT','MANU','INUSD','INUSL'}; 
end

for i=1:numel(files)
  m = mlint('-id','-fullpath','-struct',files{i});
  
  skipfile = false;
  nextline = 1;
  j=1;
  while j<=numel(m)
    if (m(j).line < nextline) || any(strcmp(m(j).id,mlint_ignore_id))
      j=j+1;
      continue;
    end
    nextline=m(j).line;
    opentoline(files{i},m(j).line,m(j).column(1));
    m(j).message=strrep(m(j).message,'\','\\');
    while (1)
      reply = input(sprintf('Line %d, %s: %s ([]/c/l/f/i)? ',m(j).line,m(j).id,m(j).message),'s');
      switch lower(reply)
        case ''
          % rerun mlint to see if things got fixed and accomodate possible
          % line changes
          m = mlint('-id','-fullpath','-struct',files{i});
          j=1;
          break;
        case 'c'
          j=j+1;
          break;
        case 'l'
          disp('Skipping the remainder of this line.');
          nextline = m(j).line+1;
          j=j+1;
          break;
        case 'f'
          disp('Skipping the remainder of this file.');
          skipfile = true;
          j=j+1;
          break;
        case 'i'
          fprintf(1,'Ignoring mlint ID %s for the remainder of this execution\n',m(j).id);
          mlint_ignore_id = vertcat(mlint_ignore_id,m(j).id);
          j=j+1;
          break;
        otherwise
          disp('Unknown response');
          printInstructions;
      end
    end
    if skipfile, break; end
  end
end

% todo: enforce camel-cased etc.  
% todo: function/classnames should only use dictionary words (keep local drake dictionary)
% todo: write simple refactor tool use opentoline

end

function printInstructions

fprintf(1,'\nKeyboard interaction:\n RETURN=I''ve fixed it\n c=continue to next suggestion\n l=skip the rest of this line\n f=skip the rest of this file\n i=skip this id for the remainder\n\n');

end
