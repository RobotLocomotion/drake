function tree=unitTest(autorun)

% Opens a gui for running unit tests
%
% Recurses through the Drake directories, locating all test scripts 
% (in test subdirectories) and all testable methods/scripts/class static methods 
% in the examples directory.
%
% A test function is any valid .m file which calls error() if the test
% fails, and returns without error if the test passes. 
%
% crawls static methods, looks for NOTEST, ...
%
% All unit tests should always be run (and all tests should pass) before 
% anything is committed back into the Drake repository.

% todo: figure out a good way to clear classes before/after runTest,
% without doing the clear all that zaps the gui content, too.

h=figure(1302);   
clear global runNode_mutex;
set(h,'HandleVisibility','on');
clf;

data=struct('pass',0,'fail',0,'wait',0,'path',pwd);
root = uitreenode('v0','All Tests',sprintf('<html>All Tests &nbsp;&nbsp;<i>(passed:%d, failed:%d, not yet run:%d)</i></html>',data.pass,data.fail,data.wait),[matlabroot, '/toolbox/matlab/icons/greenarrowicon.gif'],false);
set(root,'UserData',data);
crawlDir('examples',root,false);

[tree,treecont] = uitree('v0','Root',root);
expandAll(tree,root);
%crawlDir('.',root,true);

hb = [];
hb = [hb,uicontrol('String','Dock','callback',@dock)];
hb = [hb,uicontrol('String','Reload (keep history)','callback',@(h,env) cleanup(h,env,tree),'BusyAction','cancel')];
hb = [hb,uicontrol('String','Reload (flush history)','callback',@(h,env) reloadGui(h,env,tree),'BusyAction','cancel')];
htext = uicontrol('Style','text','String','Click to run tests.  Shift-click to edit test file.','BackgroundColor',[1 1 1]);

set(tree,'NodeSelectedCallback', @runSelectedNode);
set(treecont,'BusyAction','queue');
resizeFcn([],[],treecont,hb,htext);
set(h,'MenuBar','none','ToolBar','none','Name','Drake Unit Tests','NumberTitle','off','ResizeFcn',@(src,ev)resizeFcn(src,ev,treecont,hb,htext));
set(h,'HandleVisibility','off');%,'WindowStyle','docked'%,'CloseRequestFcn',[]);

jtree = handle(tree.getTree,'CallbackProperties');
set(jtree,'KeyPressedCallback', @keyPressedCallback);
set(jtree,'KeyReleasedCallback', @keyReleasedCallback);

if (nargin>0 && autorun)
  tree.setSelectedNode(root);
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
  megaclear;
  load drake_config;
  cd(conf.root);
  tree=unitTest;
end

function cleanup(h,env,tree)
  saveTree(tree);
  tree=reloadGui(h,env,tree);
  loadTree(tree);
end

function saveTree(tree)
  load drake_config;
  cd(conf.root);
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
end

function loadTree(tree)
  load drake_config;
  cd(conf.root);
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
end

function saveMutex()
  load drake_config;
  cd(conf.root);
  backupname = '.drake_unit_test_mutex.mat';
  global runNode_mutex;
  node_data = cellfun(@(a) get(a,'UserData'),runNode_mutex,'UniformOutput',false);
  save(backupname,'node_data');
end

function loadMutex(tree)
  load drake_config;
  cd(conf.root);
  backupname = '.drake_unit_test_mutex.mat';
  global runNode_mutex;
  load(backupname);
  if ~isempty(node_data)
    runNode_mutex = cellfun(@(a) findNode(tree,a),node_data,'UniformOutput',false);
  end
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
  set(htext,'Position',[0,pos(4)-40,pos(3),18]);
  set(treecont,'Position',[0,0,pos(3),pos(4)-38]);
end

function expandAll(tree,node)
  tree.expand(node);
  iter = node.breadthFirstEnumeration;
  while iter.hasMoreElements
    tree.expand(iter.nextElement);
  end
end

function pnode = crawlDir(pdir,pnode,only_test_dirs)

  p = pwd;
  cd(pdir);
  
  data=struct('pass',0,'fail',0,'wait',0,'path',pdir);
  node = uitreenode('v0',pdir,['<html>',pdir,sprintf(' &nbsp;&nbsp;<i>(passed:%d, failed:%d, not yet run:%d)</i></html>',data.pass,data.fail,data.wait)],[matlabroot, '/toolbox/matlab/icons/greenarrowicon.gif'],false);
  set(node,'UserData',data);
  pnode.add(node);
  files=dir('.');
  
  for i=1:length(files)
    if (files(i).isdir)
      % then recurse into the directory
      if (files(i).name(1)~='.' && ~any(strcmpi(files(i).name,{'dev','slprj','autogen-matlab','autogen-posters'})))  % skip . and dev directories
        crawlDir(files(i).name,node,only_test_dirs && ~strcmpi(files(i).name,'test'));
      end
      continue;
    end
    if (only_test_dirs) continue; end

    if (~strcmpi(files(i).name(end-1:end),'.m')) continue; end
    if (strcmpi(files(i).name,'Contents.m')) continue; end
    
    testname = files(i).name;
    ind=find(testname=='.',1);
    testname=testname(1:(ind-1));
    
    isClass = checkFile(files(i).name,'classdef');
    if (isClass)
      if (checkClass(files(i).name,'NOTEST'))
        continue; 
      end
      m = staticMethods(testname);
      for j=1:length(m)
        if (checkMethod(files(i).name,m{j},'NOTEST'))
          continue;
        end
        
        node = addTest(node,[testname,'.',m{j}]);
      end
      
    else
      % reject if there is a notest
      if (checkFile(files(i).name,'NOTEST'))
        continue;
      end
      node = addTest(node,testname);
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

function pass = runTest(path,test)
  p=pwd;
  cd(path);
%  disp(['running ',path,'/',test,'...']);

  if (exist('rng'))
    rng('shuffle'); % init rng to current date
  else  % for older versions of matlab
    rand('seed',sum(100*clock));
  end
  force_close_system();
  close all;
  clearvars -global -except runNode_mutex;

  s=dbstatus;
  % useful for debugging: if 'dbstop if error' is on, then don't use try catch. 
  if any(strcmp('error',{s.cond})) ||any(strcmp('warning',{s.cond}))
    feval_in_contained_workspace(test);
  else
    try
      feval_in_contained_workspace(test);
    catch ex
      pass = false;
      cd(p);
      disp(' ');
      disp(' ');
      disp('*******************************************************');
      disp(['* error in ',path,'/',test]);
      disp('*******************************************************');
      disp(getReport(ex,'extended'));
      disp('*******************************************************');
      lasterr(ex.message,ex.identifier);
      %    rethrow(ex);
      return;
    end
  end
  
  a=warning;
  if (~strcmp(a(1).state,'on'))
    error('somebody turned off warnings on me!');  % see bug
  end
  
  force_close_system();
  close all;
  clearvars -global -except runNode_mutex;
  
  t = timerfind;
  if (~isempty(t)) stop(t); end
  
  pass = true;
  cd(p);
end

function feval_in_contained_workspace(f) % helps with scripts
  feval(f);
end

function bfound = checkFile(filename,strings)
% opens the file and checks for the existence of the string (or strings)

if ~iscell(strings), strings = {strings}; end

bfound = false;
fid=fopen(filename);
if (fid<0) return; end  % couldn't open the file.  skip it.
while true  % check the file for the strings
  tline = fgetl(fid);
  if (~ischar(tline))
    break;
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

function bfound = checkClass(filename,strings)

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
  commentind = strfind(tline,'%');
  if (~isempty(commentind)) tline = tline(1:commentind(1)-1); end
  if (~bInMethod && ~isempty(strfind(tline,'function')))
    bInMethod = true;
    endcount=0;
  end
  if (bInMethod)
    strings={'for','while','switch','try','if'};
    endcount = endcount + length(keywordfind(tline,strings));
    endcount = endcount - length(keywordfind(tline,'end'));
%    disp([num2str(endcount,'%2d'),': ',tline]);
    if endcount<0
      bInMethod=false;
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
