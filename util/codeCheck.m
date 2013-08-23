function codeCheck(options)

% run mlint and a series of style checks on all available m files 
% in the current directory tree
%
% @option mlint_ignore_id cell array of mlint IDs to safely ignore @default {}

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
  mlint_ignore_id = {}; 
end

for i=1:numel(files)
  m = mlint('-id','-fullpath','-struct',files{i});
  
  skipfile = false;
  for j=1:numel(m)
    if strcmp(m(j).id,mlint_ignore_id)
      continue;
    end
    opentoline(files{i},m(j).line,m(j).column(1));
    while (1)
      reply = input(sprintf('%s: %s ([c]/f/i)? ',m(j).id,m(j).message),'s');
      switch lower(reply)
        case {'','c'}
          % pass through to the next item
          break;
        case 'f'
          disp('Skipping the remainder of this file.');
          skipfile = true;
          break;
        case 'i'
          fprintf(1,'Ignoring mlint ID %s for the remainder of this execution\n',m(j).id);
          vertcat(mlint_ignore_id,m(j).id);
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

fprintf(1,'\nKeyboard interaction:\n c=continue to next suggestion\n f=skip the rest of this file\n i=skip this id for the remainder\n\n');

end
