function src_files = findDependencySource(toolbox)

% after running codeCheck(struct('dependency_report',true)), you might be
% curious about where the dependencies are coming from.  This is a much
% more expensive operation, but it calls the dependency tool repeatedly to
% find the source

[~,filestr] = system('find . -iname "*.m" | grep -v /dev/');
files = strread(filestr,'%s\n');
num_files = numel(files);
h = waitbar(0,'Searching for dependency source');


if (0) % linear search

  for i=1:length(files)
    waitbar(i/num_files,h);
    toolboxes = dependencies.toolboxDependencyAnalysis(files(i));
    if ~isempty(toolboxes) && any(strcmp(toolbox,toolboxes))
      disp(files{i})
    end
  end

else % grouped search
  
  N=40; 
  for i=1:N:length(files)
    waitbar(i/num_files,h);
    toolboxes = dependencies.toolboxDependencyAnalysis(files(i:min(i+N-1,num_files)));
    if ~isempty(toolboxes) && any(strcmp(toolbox,toolboxes))
      for j=i:min(i+N-1,num_files)
        toolboxes = dependencies.toolboxDependencyAnalysis(files(j));
        if ~isempty(toolboxes) && any(strcmp(toolbox,toolboxes))
          disp(files{j})
        end
      end
    end
  end

end

close(h)