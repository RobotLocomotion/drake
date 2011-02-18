function force_close_system(sys)

if (nargin<1) sys = 'all'; end

if iscell(sys)
  for i=1:length(sys)
    force_close_system(sys{i});
  end
elseif ischar(sys)
  if (strcmpi(sys,'all'))
    force_close_system(find_system('SearchDepth',0));
  else
    try 
      close_system(sys,0);
    catch       
      feval(sys,[],[],[],'term');
      close_system(sys,0);
    end
  end
else
  error('sys must be a model name or cell array of model names, or the string ''all''');
end

