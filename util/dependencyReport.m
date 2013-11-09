function dependencyReport()

% run this after running the profiler...

  stats = profile('info');
  
  ind = find( ~cellfun(@isempty,strfind({stats.FunctionTable.FileName},[filesep,'toolbox',filesep])));
  toolboxes = regexp({stats.FunctionTable(ind).FileName},'/toolbox/(\w*)','tokens');
  [toolboxes,ia] = unique(cellfun(@(a) a{1},toolboxes));
  disp('======= Toolbox Dependency Report ======= ');
  for i=1:length(toolboxes)
    t = ver(toolboxes{i});
    if ~isempty(t)
      fprintf('  <a href="matlab:profview(''%s'')">%s</a>\n',stats.FunctionTable(ind(ia(i))).FunctionName,t.Name);
    end
  end
  disp('Click on the link above to see the first call to the toolbox');
  disp('Call profile(''viewer'') to see more information');
  disp('======= End of Toolbox Dependency Report ======= ');
end