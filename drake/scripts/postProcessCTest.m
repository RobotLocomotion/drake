function postProcessCTest()
% processes the output of ctest.
% this should be run from the drake root directory

mystack = dbstack;
path_to_this_mfile = fileparts(which(mystack(1).file));

testpath = 'pod-build/Testing/';
d = dir([testpath,'20*']);
d = d(end);

testfile = xmlread(fullfile(testpath,d.name,'/Test.xml'));
if ispc, python=strtrim(fileread('.python')); end

tests = testfile.getElementsByTagName('Test');

num_tests = 0;       % total number of tests
num_passed = 0;      % passed
num_cancelled = 0;   % cancelled because of (acceptable) missing dependencies
num_timed_out = 0;    % timeout
num_failed = 0;    % failed (known or unknown)
num_failed_unknown = 0;  % just unknown
missing_dependencies = containers.Map();
known_issues = containers.Map('KeyType','double','ValueType','double');
known_issue_titles = containers.Map('KeyType','double','ValueType','char');

teststr='';
error_output_str = '';
for i=0:(tests.getLength()-1)
  [this_str,this_error_output_str] = parseTest(tests.item(i));
  teststr = [teststr,this_str];
  error_output_str = [error_output_str, this_error_output_str];
end

missing_dep_str = '';
for f=keys(missing_dependencies)
  missing_dep_str = [missing_dep_str,sprintf(' %s:%d,',f{1}, missing_dependencies(f{1}))];
end
str = sprintf('%d tests run:\n %d passed\n %d cancelled (with missing dependencies %s)\n %d timed out\n %d failed\n',num_tests,num_passed,num_cancelled,missing_dep_str(1:end-1),num_timed_out,num_failed);
for f=keys(known_issues)
  str = [str,sprintf('  %d <a href="https://github.com/RobotLocomotion/drake/issues/%d">known issue #%d</a>: %s\n',known_issues(f{1}),f{1},f{1},known_issue_titles(f{1}))];
end
str = [str,sprintf('  %d unknown\n\n',num_failed_unknown)];

str = [str, teststr, sprintf('\n\nDetailed test output:\n\n'), error_output_str];

disp(str);

function [str,error_output_str] = parseTest(node)
  str='';
  error_output_str='';
  if ~node.hasAttribute('Status'), return; end

  num_tests = num_tests+1;

  % extract test name
  name_node = node.getElementsByTagName('Name').item(0);
  testname = char(name_node.getTextContent());

  % extract test output
  output='';
  n = node.getElementsByTagName('Results').item(0);
  n = n.getElementsByTagName('Measurement').item(0);
  n = n.getElementsByTagName('Value').item(0);
  if (n.hasChildNodes())
    output=char(n.getTextContent());
    if (n.hasAttribute('compression'))
      if ispc
        [~,output]=system([python,' ',path_to_this_mfile,'/decode.py ',output]);
      else
        [~,output]=system(['export LD_LIBRARY_PATH=; export DYLD_LIBRARY_PATH=; python ',path_to_this_mfile,'/decode.py ',output]);
      end
    end
    if (strncmp('sh:',output,3))  % strip bash output if the program crashed
      first_line_feed = find(output==char(10),1);
      output=output(first_line_feed+2:end);
    end
  end

  if strcmp(char(node.getAttribute('Status')),'failed')
    error_id = '';
    n2 = node.getElementsByTagName('NamedMeasurement');
    for j=1:length(n2)
      named_measurement = n2.item(j-1);
      if strcmpi(char(named_measurement.getAttribute('name')),'Exit Code')
        value = named_measurement.getElementsByTagName('Value').item(0);
        if strcmpi(char(value.getTextContent()),'Timeout')
          str = sprintf('%s (timed out)\n',testname);
          num_timed_out = num_timed_out+1;
          error_id = 'timeout';
        end
      end
    end

    num_failed = num_failed+1;

    is_segfault = ~isempty(strfind(output,'malloc_error_break')) || ~isempty(strfind(output,'Segmentation violation'));

    tokens = regexp(output,'<error_id>(.*)</error_id>','tokens');
    if ~isempty(tokens), error_id = tokens{1}{1}; end

    error_message = '';
    tokens = regexp(output,'<error_message>(.*)</error_message>','tokens');
    if ~isempty(tokens), error_message = tokens{1}{1}; end

    [github_issue_num,github_issue_title] = knownIssue(testname,error_id,error_message,is_segfault,output);

    if ~isempty(github_issue_num) % then it's a known issue
%      str = sprintf('%s (known issue: <a href="https://github.com/RobotLocomotion/drake/issues/%d">#%d</a>)\n\n',testname,github_issue_num,github_issue_num);
      if isKey(known_issues,github_issue_num)
          known_issues(github_issue_num)=known_issues(github_issue_num)+1;
      else
          known_issues(github_issue_num)=1;
          known_issue_titles(github_issue_num)=github_issue_title;
      end
      return;
    else
      if is_segfault
        str = sprintf('%s (segfault)\n', testname);
      else
        str = sprintf('%s (error_id = %s)\n', testname, error_id);
      end
      fprintf('unknown failure: %s', str);

      error_output_str = sprintf('%s (unknown issue)\n```\nerror_id = %s\nerror_message = %s\nis_segfault = %d\n```\n```Matlab\n%s\n```\n\n',testname,error_id,error_message,is_segfault,output);
    end
    num_failed_unknown=num_failed_unknown+1;
  else % then the ctest said it passed
    if isempty(strfind(output,'Drake:MissingDependency:'))
      num_passed = num_passed+1;
    %    str = sprintf('%s (passed)\n\n',testname);
    else
      num_cancelled = num_cancelled+1;
      tokens = regexp(output,'<error_id>Drake:MissingDependency:(.*)</error_id>','tokens');
      dependency = tokens{1}{1};
%      str = sprintf('%s (cancelled: missing %s)\n\n',testname, dependency);
      if isKey(missing_dependencies,dependency)
        missing_dependencies(dependency) = missing_dependencies(dependency)+1;
      else
        missing_dependencies(dependency) = 1;
      end
    end
  end
end


end
