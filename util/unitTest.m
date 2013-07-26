function gui_tree_or_command_line_data =unitTest(options)

% Opens a gui for running unit tests
%
% A test function is any valid .m file which calls error() if the test
% fails, and returns without error if the test passes.
%
% By default, it recurses through the current directory, locating all 
% testable methods/functions (not scripts)/class static methods .   
%
% This default behavior can be changed in a number of ways:
%
% If any directory that is being crawled through has a file name .NOTEST,
% then that entire directory is skipped.  In addition if any function file
% contains the string NOTEST, then it will be skipped; or if any static
% class method contains the string NOTEST, then that method will be
% skipped.
%
% If a directory that is being crawled through has a file named .UNITTEST,
% then the entire directory is not searched.  Instead this script parses
% the contents of the .UNITTEST file, expecting one filename/directory name
% entry per line.  A line with a directory name on it can optionally have a 
% second token (ALL_FILES or TEST_DIRS_ONLY) which tell this script to 
% continue crawling into this directory recursively using the modified policy.
%
%     test1.m
%     test2.m
%     testdir
%     systems TEST_DIRS_ONLY
%     examples ALL
%
%
% @option autorun if true, starts running the tests immediately.  @default
% false
% @option test_dirs_only if true, starts the crawling procedure in
% test_dirs_only mode (but this could be overwritten by a downstream
% .UNITTEST file)
% @option gui if true, shows the gui. set to false for the text only, non-interactive version 
% @default true
% @option logfile file name for writing information about FAILED tests only
%
% All unit tests should always be run (and all tests should pass) before 
% anything is committed back into the Drake repository.

% todo: figure out a good way to clear classes before/after runTest,
% without doing the clear all that zaps the gui content, too.

warning('on');
warning('off','MATLAB:uitree:DeprecatedFunction');
warning('off','MATLAB:uitreenode:DeprecatedFunction');

if (nargin<1) options=struct(); end
if ~isfield(options,'autorun') options.autorun = false; end
if ~isfield(options,'ignore_vrml') options.ignore_vrml = false; end
if ~isfield(options,'gui') options.gui = true; end
if ~isfield(options,'logfile') options.logfile = ''; end
if ~isfield(options,'test_dirs_only') options.test_dirs_only = false; end
if ~isempty(options.logfile)
  if (options.gui)
    warning('At the moment, writing to logfile is only implemented for gui=false.  Disabling logfile.');
    % fixing this would simply be a matter of finding the right (best) way
    % to pass the logfileptr variable through the tree structure to
    % runTest()
  end
  options.logfileptr = fopen(options.logfile,'w');
else
  options.logfileptr = -1;
end

if (options.gui)
  warning('off','MATLAB:uitree:DeprecatedFunction');
    
  h=figure(1302);
  clear global runNode_mutex command_line_data;
  set(h,'HandleVisibility','on');
  clf;

  data=struct('pass',0,'fail',0,'wait',0,'path',pwd);
  root = uitreenode('v0','All Tests',sprintf('<html>All Tests &nbsp;&nbsp;<i>(passed:%d, failed:%d, not yet run:%d)</i></html>',data.pass,data.fail,data.wait),[matlabroot, '/toolbox/matlab/icons/greenarrowicon.gif'],false);
  set(root,'UserData',data);
else
  global command_line_data;
  command_line_data=struct('pass',0,'fail',0);
  root = [];
end

crawlDir('.',root,options.test_dirs_only,options);

if (options.gui)
  [tree,treecont] = uitree('v0','Root',root);
  expandAll(tree,root);

  hb = [];
  hb = [hb,uicontrol('String','Dock','callback',@dock)]; if strcmp(get(h,'WindowStyle'),'docked'), set(hb(end),'String','Undock'); end
  hb = [hb,uicontrol('String','Reload (keep history)','callback',@(h,env) cleanup(h,env,tree),'BusyAction','cancel')];
  hb = [hb,uicontrol('String','Reload (flush history)','callback',@(h,env) reloadGui(h,env,tree),'BusyAction','cancel')];
  htext = uicontrol('Style','text','String','Click triangles to run tests.  Shift-click to edit test file.','BackgroundColor',[1 1 1]);

  set(tree,'NodeSelectedCallback', @runSelectedNode);
  set(treecont,'BusyAction','queue');
  resizeFcn([],[],treecont,hb,htext);
  set(h,'MenuBar','none','ToolBar','none','Name','Drake Unit Tests','NumberTitle','off','ResizeFcn',@(src,ev)resizeFcn(src,ev,treecont,hb,htext));
  set(h,'HandleVisibility','off');%,'WindowStyle','docked'%,'CloseRequestFcn',[]);

  jtree = handle(tree.getTree,'CallbackProperties');
  set(jtree,'KeyPressedCallback', @keyPressedCallback);
  set(jtree,'KeyReleasedCallback', @keyReleasedCallback);

  if (options.autorun)
    tree.setSelectedNode(root);
  end
  
  gui_tree_or_command_line_data = tree;
else
  gui_tree_or_command_line_data = command_line_data;
end


if ~isempty(options.logfile)
  fclose(options.logfileptr);

  % strip the html tags out of the logfile (which are irrelevant outside of matlab and make it difficult to read):
  system(['sed -i -e "s/''[^'']*''//g;s/<[^>]*>//g" ',options.logfile]);
end

end

function dock(h,env)

% todo: replace string with the dock / undock icons?

if (strcmp(get(1302,'WindowStyle'),'docked'))
  set(1302,'WindowStyle','normal')
  set(h,'String','Dock');
else
  set(1302,'WindowStyle','dock')
  set(h,'String','Undock');
end

end

function keyPressedCallback(hTree,eventData)
  global shift_down;
  if (eventData.getKeyCode()==16)
    shift_down = true;
  end
end

function keyReleasedCallback(hTree,eventData)
  global shift_down;
  if (eventData.getKeyCode()==16)
    shift_down = [];
  end
end


function tree=reloadGui(h,env,tree)
  d = get(tree.getRoot,'UserData');  
  cd(d.path);
  megaclear;
  tree=unitTest;
end

function cleanup(h,env,tree)
  saveTree(tree);
  tree=reloadGui(h,env,tree);
  loadTree(tree);
end

function saveTree(tree)
  p=pwd;
  d = get(tree.getRoot,'UserData');  
  cd(d.path);
  backupname = '.unitTestData.mat';
  userdata={};
  iter = tree.getRoot.breadthFirstEnumeration;
  while iter.hasMoreElements;
    node = iter.nextElement;
    if (node.isLeaf)
      d = get(node,'UserData');
      if ~isfield(d,'test'), continue; end  % false leaf
      userdata{end+1} = get(node,'UserData');
    end
  end
  backupname = '.unitTestData.mat';
  save(backupname,'userdata');
  cd(p);
end

function loadTree(tree)
  p=pwd;
  cd(getDrakePath);
  backupname = '.unitTestData.mat';
  load(backupname);
  
  iter = tree.getRoot.breadthFirstEnumeration;
  while iter.hasMoreElements;
    node = iter.nextElement;
    if (node.isLeaf)
      d = get(node,'UserData');
      if ~isfield(d,'test'), continue; end % false leaf
      pathmatches = userdata(strcmp(d.path,cellfun(@(a) getfield(a,'path'),userdata,'UniformOutput',false)));
      if (~isempty(pathmatches))
        testmatch = pathmatches(strcmp(d.test,cellfun(@(a) getfield(a,'test'),pathmatches,'UniformOutput',false)));
        if (~isempty(testmatch))
          set(node,'UserData',testmatch{1});
          switch (testmatch{1}.status)
            case 0
              % do nothing, already set to wait
            case 1
              updateParentNodes(node.getParent,'wait',-1);
              updateParentNodes(node.getParent,'pass',1);
              node.setName(['<html><font color="green">[PASSED]</font> ',d.test,'</html>']);
            case 2
              updateParentNodes(node.getParent,'wait',-1);
              updateParentNodes(node.getParent,'fail',1);
              node.setName(['<html><font color="red"><b>[FAILED]</b></font> ',d.test,'&nbsp;&nbsp;<font color="red"><i>',testmatch{1}.errmsg,'</i></font></html>']);
          end
          tree.reloadNode(node);
        end
      end
    end
  end
  cd(p);
end

function saveMutex()
  p=pwd;
  cd(getDrakePath);
  backupname = '.drake_unit_test_mutex.mat';
  global runNode_mutex;
  node_data = cellfun(@(a) get(a,'UserData'),runNode_mutex,'UniformOutput',false);
  save(backupname,'node_data');
  cd(p);
end

function loadMutex(tree)
  p=pwd;
  cd(getDrakePath);
  backupname = '.drake_unit_test_mutex.mat';
  global runNode_mutex;
  load(backupname);
  if ~isempty(node_data)
    runNode_mutex = cellfun(@(a) findNode(tree,a),node_data,'UniformOutput',false);
  end
  cd(p);
end

function node=findNode(tree,data)
  iter = tree.getRoot.breadthFirstEnumeration;
  while iter.hasMoreElements
    node = iter.nextElement;
    d = get(node,'UserData');
    if strcmp(d.path,data.path)
      if isfield(data,'test') && isfield(d,'test') && strcmp(d.test,data.test)
        return;
      elseif ~isfield(data,'test') && ~isfield(d,'test') % matching directory node
        return;
      end
    end
  end
  node=[];
end

function resizeFcn(src,ev,treecont,hb,htext)
  pos = get(1302,'Position');
  for i=1:length(hb)
    set(hb(i),'Position',[(i-1)*pos(3)/length(hb),pos(4)-20,pos(3)/length(hb),20]);
  end
  set(htext,'Position',[0,-1,pos(3),18]);
  set(treecont,'Position',[0,17,pos(3),pos(4)-38]);
end

function expandAll(tree,node)
  tree.expand(node);
  iter = node.breadthFirstEnumeration;
  while iter.hasMoreElements
    tree.expand(iter.nextElement);
  end
end

function pnode = crawlDir(pdir,pnode,only_test_dirs,options)
  p = pwd;
  cd(pdir);
  disp(['crawling ',pwd]);
%  if strcmp(pdir,'.')
%    [~,pdir]=fileparts(pwd);
%  end
  if exist('.NOTEST','file')
    cd(p);
    return;
  end
  
  if (options.gui)
    data=struct('pass',0,'fail',0,'wait',0,'path',pdir);
    htmldir = strrep(pdir,'/','&#47');
    node = uitreenode('v0',htmldir,['<html>',htmldir,sprintf(' &nbsp;&nbsp;<i>(passed:%d, failed:%d, not yet run:%d)</i></html>',data.pass,data.fail,data.wait)],[matlabroot, '/toolbox/matlab/icons/greenarrowicon.gif'],false);
    set(node,'UserData',data);
    pnode.add(node);
  else
    node=[p,'/',pdir];
  end
  
  if exist('.UNITTEST','file')
    files=dir('');
    fid=fopen('.UNITTEST');
    if (fid<0) return; end  % couldn't open the file.  skip it.
    while true  % check the file for the strings
      tline = fgetl(fid);
      if (~ischar(tline))
        break;
      end
      [name,param] = strtok(tline);  
      name=strtrim(name);
      param=strtrim(param);
      if exist(name,'dir')
        if strcmpi(param,'test_dirs_only')
          crawlDir(name,node,true,options);
        elseif strcmpi(param,'all')
          crawlDir(name,node,false,options);
        elseif ~isempty(param)
          error([fullfile(pwd,'.UNITTEST'),' has a an unknown tag: ',param]);
        else
          crawlDir(name,node,only_test_dirs && ~strcmpi(name,'test'),options);
        end
      elseif exist(name,'file')
        files=vertcat(files,dir(name));
      end
    end
    fclose(fid);
  else
    files=dir('.');
  end 
  
  for i=1:length(files)
    if (files(i).name(1)=='@')  % handle class directories
      files(i).name = fullfile(files(i).name,[files(i).name(2:end),'.m']);
      files(i).isdir = false;
    end
    
    if (files(i).isdir)
      % then recurse into the directory
      if (files(i).name(1)~='.' && ~any(strcmpi(files(i).name,{'dev','slprj','autogen-matlab','autogen-posters'})))  % skip . and dev directories
        crawlDir(files(i).name,node,only_test_dirs && ~strcmpi(files(i).name,'test'),options);
      end
      continue;
    end
    if (only_test_dirs) continue; end

    if (~strcmpi(files(i).name(end-1:end),'.m')) continue; end
    if (strcmpi(files(i).name,'Contents.m')) continue; end
    
    testname = files(i).name;
    ind=find(testname=='.',1);
    testname=testname(1:(ind-1));
    
    isClass = checkFile(files(i).name,'classdef',false);
    if (isClass)
      if (checkFile(files(i).name,'NOTEST'))
        continue; 
      end
      [m,dc] = staticMethods(testname);
      for j=1:length(m)
        fname = which(['@',fullfile(dc{j},m{j})]);
        if (checkMethod(fname,m{j},'NOTEST'))
          continue;
        end
        
        if (options.gui)
          node = addTest(node,[testname,'.',m{j}]);
        else
          runCommandLineTest(node,[testname,'.',m{j}],options);
        end
      end
      
    else
      % reject if there is a notest
      if (checkFile(files(i).name,'NOTEST'))
        continue;
      end
      % reject if it is a script
      if (~checkFile(files(i).name,'function',false))
        continue;
      end
      if (options.gui)
        node = addTest(node,testname);
      else
        runCommandLineTest(node,testname,options);
      end
    end
    
  end
  
  if (options.gui)
    data = get(node,'UserData');
    if (data.wait==0)  % then there was nothing underneath me
      pnode.remove(node);
    end
  end
  cd(p);
end

function node = addTest(node,testname)
  data=struct('path',pwd,'test',testname,'status',0,'errmsg','');
  n = uitreenode('v0',testname,['<html>',testname,'</html>'],[matlabroot, '/toolbox/matlab/icons/greenarrowicon.gif'],true);
  set(n,'UserData',data);
  node.add(n);
  
  updateParentNodes(node,'wait',1);
end

function updateParentNodes(node,field,delta)
  data = get(node,'UserData');
  data = setfield(data,field,getfield(data,field)+delta);
  set(node,'UserData',data);
  v = get(node,'Value');
  if (data.fail>0)
    node.setName(['<html>',v,sprintf(' &nbsp;&nbsp;<i>(passed:%d, <font color="red"><b>failed:%d</b></font>, not yet run:%d)</i></html>',data.pass,data.fail,data.wait)]);
  else
    node.setName(['<html>',v,sprintf(' &nbsp;&nbsp;<i>(passed:%d, failed:%d, not yet run:%d)</i></html>',data.pass,data.fail,data.wait)]);
  end
  
  if ~isempty(node.getParent)
    updateParentNodes(node.getParent,field,delta);
  end
end

function runSelectedNode(tree,ev)
  nodes = tree.getSelectedNodes;
  if (~isempty(nodes))
    node=nodes(1);
    tree.setSelectedNode([]);
    global shift_down;
    if ~isempty(shift_down) && shift_down
      editNode(node);
      shift_down = []; % because I'm about to move focus away and will miss the keyRelease event
    else
      tree=runNode(tree,node);
    end
  end
end

function editNode(node)
  data=get(node,'UserData');
  if isfield(data,'test')
    p = pwd;
    cd (data.path);
    edit(data.test);
    cd(p);
  else
    cd(data.path); 
  end
end

function tree=runNode(tree,node)
  data = get(node,'UserData');
  if (isfield(data,'test'))
    global runNode_mutex;
    if ~isempty(runNode_mutex)
      node.setName(['<html><font color="gray">[WAITING]</font> ',data.test,'</html>']);
      tree.reloadNode(node);
      runNode_mutex{end+1}=node;
      saveMutex();
      return;
    end
    runNode_mutex{1}=node;
    saveMutex();
    
    node.setName(['<html><font color="blue">[RUNNING]</font> ',data.test,'</html>']);
    tree.reloadNode(node);
    
    saveTree(tree);
    pass = runTest(data.path,data.test);
    if ~isa(tree,'javahandle_withcallbacks.com.mathworks.hg.peer.UITreePeer')
      % somebody did a close all
      tree=reloadGui([],[],tree);
      loadTree(tree);
    end
    if ~exist('runNode_mutex')
      % somebody did a clear all. 
      global runNode_mutex;
      loadMutex(tree);
      node=findNode(tree,data);
    end
    
    switch(data.status)
      case 0
        updateParentNodes(node.getParent,'wait',-1);
      case 1
        updateParentNodes(node.getParent,'pass',-1);
      case 2
        updateParentNodes(node.getParent,'fail',-1);
    end
      
    if (pass)
      node.setName(['<html><font color="green">[PASSED]</font> ',data.test,'</html>']);
      data.status = 1;
      updateParentNodes(node.getParent,'pass',1);
      set(node,'UserData',data);
    else
      data.status = 2;
      data.errmsg = lasterr;
      node.setName(['<html><font color="red"><b>[FAILED]</b></font> ',data.test,'&nbsp;&nbsp;<font color="red"><i>',data.errmsg,'</i></font></html>']);
      updateParentNodes(node.getParent,'fail',1);
      set(node,'UserData',data);
    end
    tree.reloadNode(node);
    torun = runNode_mutex(2:end);
    runNode_mutex = [];
    
    for i=1:length(torun)
      tree=runNode(tree,torun{i});
    end
  else
    iter = node.depthFirstEnumeration;
    while iter.hasMoreElements
      n=iter.nextElement;
      d=get(n,'UserData');
      n=findNode(tree,d);  % in case I had to reload
      if isfield(d,'test')
        tree=runNode(tree,n);
      end
    end
  end
end

function pass = runCommandLineTest(path,test,options)
global command_line_data;
fprintf(1,'%-40s ',test);
pass = runTest(path,test,options.logfileptr);
if (pass)
  command_line_data.pass = command_line_data.pass+1;
  fprintf(1,'[PASSED]\n');
else
  command_line_data.fail = command_line_data.fail+1;
  fprintf(1,'\n[FAILED]\n');  % \n because runTest will print things
end
end

function pass = runTest(runpath,test,logfileptr)
p=pwd;
cd(runpath);
%  disp(['running ',path,'/',test,'...']);

if (exist('rng'))
    rng('shuffle'); % init rng to current date
else  % for older versions of matlab
    rand('seed',sum(100*clock));
end
force_close_system();
close all;
clearvars -global -except runNode_mutex command_line_data;
clear Singleton;

s=dbstatus; oldpath=path();
% useful for debugging: if 'dbstop if error' is on, then don't use try catch.
if any(strcmp('error',{s.cond})) ||any(strcmp('warning',{s.cond}))
    feval_in_contained_workspace(test);
else
    try
        feval_in_contained_workspace(test);
    catch ex
        if ((strcmp(ex.identifier,'Drake:MissingDependency:vrml_enabled')))
            % we're running headless and don't care about 3D stuff
            pass=true;
        else
            
            pass = false;
            cd(p);
            disp(' ');
            disp(' ');
            disp('*******************************************************');
            disp(['* error in ',runpath,'/',test]);
            disp('*******************************************************');
            disp(getReport(ex,'extended'));
            disp('*******************************************************');
            
            if (nargin>2 && logfileptr>0)
                fprintf(logfileptr,'*******************************************************\n');
                fprintf(logfileptr,['* error in ',runpath,'/',test,'\n']);
                fprintf(logfileptr,'*******************************************************\n');
                fprintf(logfileptr,'\n');
                fprintf(logfileptr,getReport(ex,'extended'));
                fprintf(logfileptr,'\n');
                fprintf(logfileptr,'*******************************************************\n');
            end
        end
        lasterr(ex.message,ex.identifier);
        %    rethrow(ex);
        path(oldpath);
        return;
    end
end
path(oldpath);

a=warning;
if (~strcmp(a(1).state,'on'))
    error('somebody turned off warnings on me!');  % see bug
end

force_close_system();
close all;
clearvars -global -except runNode_mutex command_line_data;

t = timerfind;
if (~isempty(t)) stop(t); end

pass = true;
cd(p);
end

function feval_in_contained_workspace(f) % helps with scripts
  feval(f);
end

function bfound = checkFile(filename,strings,search_comments)
% opens the file and checks for the existence of the string (or strings)

if nargin<3, search_comments=true; end

if ~iscell(strings), strings = {strings}; end

bfound = false;
fid=fopen(filename);
if (fid<0) return; end  % couldn't open the file.  skip it.
while true  % check the file for the strings
  tline = fgetl(fid);
  if (~ischar(tline))
    break;
  end
  if ~search_comments
    tline = strtok(tline,'%');  % remove text after comments
  end
  for i=1:length(strings)
    if (~isempty(strfind(tline,strings{i})))
      fclose(fid);
      bfound = true;
      return;
    end
  end
end
fclose(fid);

end

function bfound = checkMethod(filename,methodname,strings)

if ~iscell(strings), strings = {strings}; end
strings = lower(strings);

bfound = false;
bInMethod = false;
endcount = 0;
fid=fopen(filename);
if (fid<0) return; end  % couldn't open the file.  skip it.
while true  % check the file for the strings
  tline = fgetl(fid);
  if (~ischar(tline))
    break;
  end
  tline = lower(tline);
  if (~bInMethod && ~isempty(keywordfind(tline,'function')))
    if (~isempty(strfind(tline,lower(methodname))))
      bInMethod = true;
      endcount=0;
    end
  end
  if (bInMethod)
    endcount = endcount + length(keywordfind(tline,{'for','while','switch','try','if'}));
    endcount = endcount - length(keywordfind(tline,'end'));
    
    for i=1:length(strings)
      if (~isempty(strfind(tline,strings{i})))
        fclose(fid);
        bfound = true;
        return;
      end
    end
%    disp([num2str(endcount,'%2d'),': ',tline]);
    if endcount<0
      fclose(fid);
      return;
    end
  end
end
fclose(fid);

end


function inds = keywordfind(line,strs)

if (~iscell(strs)) strs={strs}; end

commentind = strfind(line,'%');
if (~isempty(commentind)) line = line(1:commentind(1)-1); end

inds=[];
for i=1:length(strs)
  s = strs{i};
  a = strfind(line,s);
  % check that it is bracketed by a non-letter
  j=1;
  while j<=length(a)
    if (a(j)~=1 && isletter(line(a(j)-1)))
      a(j)=[];
      continue;
    end
    if (a(j)+length(s)<=length(line) && isletter(line(a(j)+length(s))))
      a(j)=[];
      continue;
    end
    inds=[inds,a(j)];
    j=j+1;
  end
end  

end
