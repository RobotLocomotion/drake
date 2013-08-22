function count = runmlint(optsfile,searchfiles)
%RUNMLINT Runs MLint on a batch of files and filters the results
%
% count = runmlint(optsfile,searchfiles)
%
% Both arguments are optional, and can be omitted or empty.
%
% optsfile is the file containing the filter rules.  If unspecified,
%   all results are displayed
% searchfiles is the list of MATLAB-files to be processed.  If unspecified,
%   all M-files in the current directory and its subdirectories are
%   processed
%
% count is the number of messages from MLint which were not filtered out.
%
% Each line in the options file is of the form:
%   <action> <argument>
% 
% Actions are:
%   IgnoreAllID <message-id>
%   IgnoreLineID <message-id> <filename> <line>
%   IgnoreFileID <message-id> <filename>
%   IgnoreAllFile <filename>
%   IgnoreAllLine <filename> <line>
%   # Comment
%
% Message IDs are those returned by MLint.  All file names must be specified
% relative to the current MATLAB directory.

% Copyright 2006-2010 The MathWorks, Inc.

count = 0;

outfilename = 'mlint_output.html';

if nargin && ~isempty(optsfile)
    opts = textread(optsfile,'%s','delimiter','\n');
else
    disp('No options file specified.  Displaying all messages');
    opts = {};
end

id_filters = [];
line_filters = [];
line_id_filters = [];
file_filters = [];
file_id_filters = [];

for i=1:numel(opts)
    x = strread(opts{i},'%s');
    if numel(x) && x{1}(1)=='#'
        continue; % Comment
    else
        switch lower(x{1})
            case 'ignoreallid'
                % Just the ID
                if numel(x)~=2
                    warning('RunMLint:MalformedOptions',...
                        'Line %d in options file is malformed',i);
                    continue;
                end
                id_filters = i_add_id_filter(id_filters, x{2});
            case 'ignorelineid'
                % Message ID, File name, line number
                if numel(x)~=4
                    warning('RunMLint:MalformedOptions',...
                        'Line %d in options file is malformed',i);
                    continue;
                end
                fn = i_resolve(x{3});
                ln = str2double(x{4});
                if ~i_isinteger(ln)
                    warning('RunMLint:MalformedOptions',...
                        'Line %d in options file is malformed',i);
                end
                line_id_filters = i_add_line_id_filter(line_id_filters, x{2}, fn, ln);
            case 'ignorefileid'
                % Message ID, File name
                if numel(x)~=3
                    warning('RunMLint:MalformedOptions',...
                        'Line %d in options file is malformed',i);
                    continue;
                end
                fn = i_resolve(x{3});
                file_id_filters = i_add_file_id_filter(file_id_filters, x{2}, fn);
            case 'ignoreallfile'
                % Just the file name
                if numel(x)~=2
                    warning('RunMLint:MalformedOptions',...
                        'Line %d in options file is malformed',i);
                    continue;
                end
                fn = i_resolve(x{2});
                file_filters = i_add_file_filter(file_filters,fn);
            case 'ignoreallline'
                % File name, line number
                if numel(x)~=3
                    warning('RunMLint:MalformedOptions',...
                        'Line %d in options file is malformed',i);
                    continue;
                end
                fn = i_resolve(x{2});
                ln = str2double(x{3});
                if ~i_isinteger(ln)
                    warning('RunMLint:MalformedOptions',...
                        'Line %d in options file is malformed',i);
                    continue;
                end
                line_filters = i_add_line_filter(line_filters, fn, ln);
            otherwise
                warning('RunMLint:MalformedOptions',...
                        'Unknown action: %s',x{1});
        end
    end
end

if nargin<2
    % Returns absolute file names
    searchfiles = i_filesearch(pwd);
else
    % searchfiles = fullfile(pwd,searchfiles)
    searchfiles = strcat(pwd,filesep,cellstr(searchfiles));
end

searchfiles = i_apply_file_filters(file_filters,searchfiles);

fid = fopen(outfilename,'wt');
%fid = 1;

fprintf(fid,'<html>\n<head><title>MLint Report for %s</title>\n<body>\n',pwd);
fprintf(fid,'<h3>MLint Report for %s</h3>\n',pwd);

i_print_id_filters(id_filters,fid);
i_print_file_filters(file_filters,fid);
i_print_line_filters(line_id_filters,fid);
i_print_file_id_filters(file_id_filters,fid);
i_print_line_id_filters(line_id_filters,fid);

fprintf(fid,'<p><table border="1">\n');
fprintf(fid,'<tr><th>File and line</th><th>ID</th><th>Message</th>\n');

for i=1:numel(searchfiles)
    % Run mlint on this file
    fn = searchfiles{i};
    %fprintf(fid,'Running %s\n',fn);
    m = mlint('-id','-fullpath','-struct',fn);
    
    m = i_apply_id_filters(id_filters,m);
    m = i_apply_file_id_filters(file_id_filters,fn,m);
    m = i_apply_line_filters(line_filters,fn,m);
    m = i_apply_line_id_filters(line_id_filters,fn,m);
    
    for k=1:numel(m)
        if strncmpi(pwd,fn,numel(pwd))
            fn = fn(numel(pwd)+2:end);
        end
        if numel(m(k).line)>1
            m(k).line = m(k).line(1);
        end
        fprintf(fid,'<tr><td><a href="matlab:opentoline(''%s'',%d)">%s:%d</a></td>\n',...
            fn, m(k).line, fn, m(k).line);
        fprintf(fid,'<td>%s</td>\n',m(k).id);
        fprintf(fid,'<td>%s</td></tr>\n',m(k).message);
        count = count + 1;
    end
end

fprintf(fid,'</table><p>Complete.\n%d warnings.\n',count);

fprintf(fid,'</body></html>\n');

web(outfilename);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Line Filters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function filters = i_add_line_filter(filters,file,line)

if isempty(filters)
    filters = struct('file',file,'line',line);
else
    filters(end+1).file = file;
    filters(end).line = line;
end

%%%%%%%%%%%%%%%%%
function messages = i_apply_line_filters(filters,file,messages)

if isstruct(filters) && isstruct(messages)
    ff = { filters.file };
    inthisfile = strcmpi(ff,file);
    if any(inthisfile)
        fl = [ filters.line ];
        ml = [ messages.line ];
        ignorelines = fl(inthisfile);
        remove = ismember(ml,ignorelines);
        messages = messages(~remove);
    end
end

%%%%%%%%%%%%%%%%%%
function i_print_line_filters(filters,fid)

if isstruct(filters)
    fprintf(fid,'Ignoring all messages on the following lines:<ul>\n');
    for i=1:numel(filters)
        f = filters(i);
        fprintf(fid,'<li>%s:%d</li>\n', f.file, f.line);
    end
    fprintf(fid,'</ul>\n');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Line ID Filters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% id & file are strings. line is a number
function filters = i_add_line_id_filter(filters,id,file,line)

if isempty(filters)
    filters = struct('id',id,'file',file,'line',line);
else
    filters(end+1).id = id;
    filters(end).file = file;
    filters(end).line = line;
end

%%%%%%%%%%%%%%%%%
% file is a string
function messages = i_apply_line_id_filters(filters,file,messages)

if isstruct(filters) && isstruct(messages)
    ff = { filters.file };
    inthisfile = strcmpi(ff,file);
    if any(inthisfile)
        fl = [ filters.line ];
        ignorelines = fl(inthisfile);
        fid = { filters.id };
        ignoreids = fid(inthisfile);
        
        remove = zeros(size(messages));
        for i=1:numel(messages)
            ind = find(messages(i).line==ignorelines);
            for k=1:numel(ind)
                if strcmpi(ignoreids{k},messages(i).id)
                    remove(i) = 1;
                end
            end
        end
        messages = messages(~remove);
    end
end

%%%%%%%%%%%%%%%%%%
function i_print_line_id_filters(filters,fid)

if isstruct(filters)
    fprintf(fid,'Ignoring specific messages on the following lines:<ul>\n');
    for i=1:numel(filters)
        f = filters(i);
        fprintf(fid,'<li><tt>%s</tt>, %s:%d</li>\n',f.id, f.file, f.line);
    end
    fprintf(fid,'</ul>\n');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% File Filters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function filters = i_add_file_filter(filters,file)

filters = i_cellcat(filters,{file});

%%%%%%%%%%%%%%%%%
function files = i_apply_file_filters(filters,files)

if iscell(filters)
    cf = lower(files);
    filters = lower(filters);
    files = files( ~ismember(cf,filters) );
end

%%%%%%%%%%%%%%%%%%
function i_print_file_filters(filters,fid)

if iscell(filters)
    fprintf(fid,'Ignoring all messages in the following files:<ul>\n');
    fprintf(fid,'<li>%s</li>\n',filters{:});
    fprintf(fid,'</ul>\n');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% ID Filters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function filters = i_add_id_filter(filters,id)

filters = i_cellcat(filters,{id});

%%%%%%%%%%%%%%%%%
function messages = i_apply_id_filters(filters,messages)

if isstruct(messages) && iscell(filters)
    ids = lower({ messages.id });
    filters = lower(filters);
    messages = messages( ~ismember(ids,filters) );
end

%%%%%%%%%%%%%%%%%%
function i_print_id_filters(filters,fid)

if iscell(filters)
    fprintf(fid,'Ignoring all messages with the following IDs:<ul>\n');
    fprintf(fid,'<li><tt>%s</tt></li>\n',filters{:});
    fprintf(fid,'</ul>\n');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% File ID Filters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function filters = i_add_file_id_filter(filters,id,file)

if isempty(filters)
    filters = struct('id',id,'file',file);
else
    filters(end+1).id = id;
    filters(end).file = file;
end

%%%%%%%%%%%%%%%%%
function messages = i_apply_file_id_filters(filters,file,messages)

if isstruct(filters) && isstruct(messages)
    ff = { filters.file };
    inthisfile = strcmpi(ff,file);
    if any(inthisfile)
        fid = { filters.id };
        mid = lower({ messages.id });
        ignoreids = fid(inthisfile);
        remove = ismember(mid,ignoreids);
        messages = messages(~remove);
    end
end

%%%%%%%%%%%%%%%%%%
function i_print_file_id_filters(filters,fid)

if isstruct(filters)
    fprintf(fid,'Ignoring specific messages in the following files:<ul>\n');
    for i=1:numel(filters)
        f = filters(i);
        fprintf(fid,'<li><tt>%s</tt>, %s</li>\n',f.id, f.file);
    end
    fprintf(fid,'</ul>\n');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function c = i_cellcat(varargin)
%I_CELLCAT Joins cell arrays into a single vertical cell array
%    c = i_cellcat(a,b,c...)
% Shorthand for c = [ a(:) ; b(:) ; c(:) ... ];

n = sum(cellfun('prodofsize',varargin));
c = cell(n,1);
count = 1;
for i=1:nargin
    n = numel(varargin{i});
    c(count:count+n-1) = varargin{i}(:);
    count = count + n;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
function b = i_isinteger(v)
%I_ISINTEGER Returns true if the specified value is an integral scalar
%   b = i_isinteger(v)
b = ~isempty(v) && isnumeric(v) && numel(v)==1 && floor(v)==v;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function foundfiles = i_filesearch(startpath)
%I_FILESEARCH Finds M-files inside a specified directory
%
%  Searches a specified path (recursively or non-recursively) for M-files
%  Results are returned as a cell array of strings
%
% files = i_filesearch(startpath)
%

foundfiles = {};
%Create string of recursive directories/subdirectories
paths = genpath(startpath);
paths = strread(paths,'%s','delimiter',pathsep);
for i=1:numel(paths)
    tempfiles = i_rundir(paths{i});
    foundfiles = i_cellcat(foundfiles,tempfiles);
end

%%%%%%%%%%%%%%
% Helper functon for i_filesearch
function foundfiles = i_rundir(dirpath)

% First check for the existence of class or private directories,
% which won't have been picked up by genpath.
w = dir(dirpath);
ind = logical([w.isdir]);
subdirs = { w(ind).name };
match = regexp(subdirs,'^@'); % identify class directories
process_dir = ~cellfun('isempty',match);
% Also process the private directory if there is one
process_dir = process_dir | strcmp(subdirs,'private');
methods = {};
for i = find(process_dir)
    classname = subdirs{i};
    methods = i_cellcat(methods,i_rundir(fullfile(dirpath,classname)));
end
% Now find M-files in this directory
w = dir(fullfile(dirpath,'*.m'));
foundfiles = cell(numel(w),1);
for i=1:numel(foundfiles)
    foundfiles{i} = fullfile(dirpath,w(i).name);
end
foundfiles = [foundfiles ; methods];

%%%%%%%%%%%%%%%%%%
function f = i_resolve(f)

f = fullfile(pwd,f);
if ~exist(f,'file')
    warning('RunMLint:FileNotFound','File not found: %s',f);
end

