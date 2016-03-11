function [github_issue_num,github_issue_title] = knownIssue(test_name,error_id,error_message,is_segfault,test_output)

persistent issue;

if isempty(issue)
  % crawl the open issues and find <matlab_test> tags
  issue = [];
  page = 1;
  while (1)
    github_issues = githubAPI(['https://api.github.com/repos/RobotLocomotion/drake/issues?page=',num2str(page)]);
    if isempty(github_issues)
      break;
    end
    for i=1:length(github_issues)
      tokens = regexp(github_issues{i}.body,'<matlab_test>(.*)</matlab_test>','tokens');
      if ~isempty(tokens)
%        fprintf('#%d: %s\n',github_issues{i}.number,tokens{1}{1});
        issue = vertcat(issue,struct('number',github_issues{i}.number,'test',tokens{1}{1},'title',github_issues{i}.title));
      end
    end
    page = page+1;
  end
end

if strcmp(error_id,'Simulink:SFunctions:SFcnErrorStatus')
  % then try to extract the original identifier in the message
  ids = regexp(error_message,'error (.+) in MATLAB callback','tokens');
  if ~isempty(ids), error_id = ids{1}{1}; end
end

github_issue_num = [];
github_issue_title = '';

persistent already_reported

for i=1:length(issue)
  hit = false;
  try
    hit = eval(issue(i).test);
  catch ex
    if isempty(already_reported)
      mysendmail('drake-build@mit.edu','[Drake] known issue logic is broken', sprintf('Issue: <a href="https://github.com/RobotLocomotion/drake/issues/%d">#%d</a>\n%s',issue(i).number,issue(i).number,getReport(ex)));
      disp(getReport(ex));
      disp(issue(i).test);
    end
  end
  if (hit)
    github_issue_num = issue(i).number;
    github_issue_title = issue(i).title;
    return;
  end
end

  already_reported = true;

end

function varargout = getOutput(outputNo,func,varargin)
  varargout = cell(max(outputNo),1);
  [varargout{:}] = func(varargin{:});
  varargout = varargout(outputNo);
end
