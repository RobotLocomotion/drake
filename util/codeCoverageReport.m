function codeCoverageReport()

  % run this after running the profiler.  displays a list of 
  % files, methods, and lines in the current directory (+ subdirectories,
  % skipping /dev/ and /test/ subdirectories) that we not touched by any
  % method during the most recent invocationo of the profiler.
  %
  % the most common way to run this is running unitTest with the 'coverage'
  % option set to true

  stats = profile('info');

  [~,filestr] = system('find . -iname "*.m" | grep -v /dev/ | grep -v /test/');
  files = strread(filestr,'%s\n');
%  files = cellfun(@(a) [pwd,a(2:end)],files,'UniformOutput',false); 
  current_path = pwd;

  disp('======= Code Coverage Report =======');
  for file_index=1:length(files)
    current_filename = [current_path,files{file_index}(2:end)];
    function_table_indices = find(strcmp(current_filename,{stats.FunctionTable.FileName}));
    if isempty(function_table_indices)
      fprintf(' Untouched file: <a href="matlab:edit(''%s'')">%s</a>\n',files{file_index},files{file_index});
    else
      executed_lines = vertcat(stats.FunctionTable(function_table_indices).ExecutedLines);
      executed_lines = executed_lines(:,1);
      
      if isempty(executed_lines)
        fprintf(' The file <a href="matlab:edit(''%s'')">%s</a> was called but did not execute (presumably do to parse errors)',files{file_index},files{file_index});
      else
        executable_lines = callstats('file_lines',current_filename)';
        missed_lines = setdiff(executable_lines,executed_lines);
        if ~isempty(missed_lines)
          call_info = getcallinfo(current_filename,'-v7.8');
          executed_function_names = cellfun(@(a) regexp(a,'(\w+)$','tokens','once'), {stats.FunctionTable(function_table_indices).FunctionName},'UniformOutput',false);
          
          for function_info = call_info
            function_table_index = function_table_indices(strcmp(function_info.name,executed_function_names));
            
            if isempty(function_table_index)
              [path,file,ext] = fileparts(files{file_index});
              fprintf('  Untouched method <a href="matlab:matlab.desktop.editor.openAndGoToFunction(''%s'',''%s'');">%s>%s</a>\n',current_filename,function_info.name,file,function_info.name);
            else
              executable_lines_in_function = executable_lines(executable_lines>=function_info.firstline & executable_lines<=function_info.lastline);
              executed_lines_in_function = stats.FunctionTable(function_table_index).ExecutedLines;
              executed_lines_in_function = executed_lines_in_function(:,1);
              missed_lines_in_function = setdiff(executable_lines_in_function,executed_lines_in_function);
              if ~isempty(missed_lines_in_function)
                fprintf('  Untouched lines in <a href="matlab:profview(''%s'')">%s</a>\n',stats.FunctionTable(function_table_index).FunctionName,stats.FunctionTable(function_table_index).FunctionName);
%                disp(missed_lines_in_function')
              end
            end
          end
        end
      end
    end
  end
  disp('======= End of Code Coverage Report =======');
end
