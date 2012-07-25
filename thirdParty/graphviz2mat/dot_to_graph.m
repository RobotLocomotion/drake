function [Adj, labels, x, y] = dot_to_graph(filename)

% [Adj, labels, x, y] = dot_to_graph(filename)
% Extract an adjacency matrix, node labels, and layout (nodes coordinates) 
% from GraphViz file       http://www.research.att.com/sw/tools/graphviz
%
% INPUT:  'filename' - the file in DOT format containing the graph layout.
% OUTPUT: 'Adj' - an adjacency matrix with sequentially numbered edges; 
%     'labels'  - a character array with the names of the nodes of the graph;
%          'x'  - a row vector with the x-coordinates of the nodes in 'filename';
%          'y'  - a row vector with the y-coordinates of the nodes in 'filename'.
%
% NOTEs: not guaranted to parse ANY GraphViz file. Debugged on undirected 
%       sample graphs from GraphViz(Heawood, Petersen, ER, ngk10_4, process). 
%       Complaines about RecursionLimit on huge graphs.
%       Ignores singletons (disjoint nodes). Handles loops (arc to self).          
% Sample DOT code "ABC.dot", read by [Adj, labels, x, y] = dot_to_graph('ABC.dot')
% Plot by    draw_graph(adj>0, labels, zeros(size(x,2),1), x, y);  % from BNT
% digraph G {
%       A [pos="28,31"];
%       B [pos="74,87"];
%       A -- B [pos="e,61,71 41,47 46,53 50,58 55,64"];
% }
%                                                last modified: 24 Feb 2004
% by Dr. Leon Peshkin: pesha @ ai.mit.edu | http://www.ai.mit.edu/~pesha 
%  & Alexi Savov:  asavov @ wustl.edu |  http://artsci.wustl.edu/~azsavov

       %  UNCOMMENT, but beware -- SLOW CHECK !!!! 
%if ~exist(filename)                % Checks whether the specified file exists.
%   error('* * * File does not exist or could not be found. * * *');     return;
%end;     

lines = textread(filename,'%s','delimiter','\n','commentstyle','c');  % Read file into cell array of lines
dot_lines = strvcat(lines);                                           % ignoring C-style comments

if isempty(findstr(dot_lines(1,:), 'graph '))                  % Is this a DOT file ?
   error('* * * File does not appear to be in valid DOT format. * * *');    return;
end;

Nlns = size(dot_lines,1);             % The number of lines;
labels = {};
unread = 1:Nlns;             % 'unread' list of lines which has not been examined yet
edge_id = 1;
for line_ndx = 1:Nlns          % This section sets the adjacency matrix entry A(Lnode,Rnode) = edge_id.
    line = dot_lines(line_ndx,:);
    Ddash_pos = strfind(line, ' -- ') + 1;  % double dash positions
    arrow_pos = strfind(line, ' -> ') + 1;  % arrow  dash positions
    tokens = strread(line,'%s','delimiter',' "');
    left_bound = 1;
    for dash_pos = [Ddash_pos arrow_pos];  % if empty - not a POS line
        Lnode = sscanf(line(left_bound:dash_pos -2), '%s');
        Rnode = sscanf(line(dash_pos +3 : length(line)-1),'%s',1);
        Lndx = strmatch(Lnode, labels, 'exact');
        if isempty(Lndx)         % extend our list of labels 
            labels{end+1} = Lnode;
            Lndx = length(labels);
        end
        Rndx = strmatch(Rnode, labels, 'exact');
        if isempty(Rndx)
            labels{end+1} = Rnode;
            Rndx = length(labels);
        end
        Adj(Lndx, Rndx) = edge_id;;
        if  ismember(dash_pos, Ddash_pos)  % The edge is undirected, A(Rndx,LndxL) is also set to 1;
            Adj(Rndx, Lndx) = edge_id;
        end
        edge_id = edge_id + 1; 
        left_bound = dash_pos + 3;
        unread = my_setdiff(unread, line_ndx); 
    end
end
Nvrt = length(labels);  % number of vertices we found  [Do we ever have singleton vertices ???]
% labels = strvcat(labels); % could convert to the searchable array
x = zeros(1, Nvrt); 
y = zeros(1, Nvrt);
lst_node = 0;
        % Find node's position coordinates if they are contained in 'filename'.
for line_ndx = unread        % Look for node's coordiantes among the 'unread' lines.
    line = dot_lines(line_ndx,:);
    bra_pos  = strfind(line, '[');      % has to have "[" if it has the lable
    pos_pos = strfind(line, 'pos');     % position of the "pos"
    for node = 1:Nvrt     % look through the list of labels 
        %  THE NEXT STATEMENT we assume no label is substring of any other label
        lbl_pos = strfind(line, labels{node});
        if ((~isempty(lbl_pos)) & (~isempty(bra_pos)) & (x(node) == 0))  % make sure we have not seen it 
            if (lbl_pos(1) < bra_pos(1))  % label has to be to the left of braket
                lst_node = node;
            end
        end
    end
    if (~isempty(pos_pos) & lst_node)   % this line contains SOME position  
        [node_pos] = sscanf(line(pos_pos:length(line)), ' pos  = "%d,%d"')';
        x(lst_node) = node_pos(1);
        y(lst_node) = node_pos(2);
        lst_node = 0;   %  not to assign position several times 
    end
end

if (isempty(find(x)) & (nargout > 2))   % If coordinates were requested, but not found in 'filename'.
    warning('File does not contain node coordinates.');
else
    x = .9*(x-min(x))/range(x)+.05;  % normalise and push off margins 
    if range(y) == 0, y = .5*ones(size(y)); else, y = .9*(y-min(y))/range(y)+.05; end
end;
if ~(size(Adj,1)==size(Adj,2))           % Make sure Adj is a square matrix. ? 
    Adj = eye(max(size(Adj)),size(Adj,1))*Adj*eye(size(Adj,2),max(size(Adj)));
end;
