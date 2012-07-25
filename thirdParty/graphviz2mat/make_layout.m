function [x, y] = make_layout(adj)
%  [x, y] = make_layout(adj)  Creates a layout from an adjacency matrix
%
% INPUT:   adj - adjacency matrix (source, sink)
% OUTPUT: x, y - Positions of nodes
%
% WARNING: Uses some very simple heuristics, any algorithm would do better 

N = size(adj,1);
tps = toposort(adj);
if ~isempty(tps),       % is directed ?
    level = zeros(1,N);
    for i=tps,
        idx = find(adj(:,i));
        if ~isempty(idx),
            l = max(level(idx));
            level(i)=l+1;
        end;
    end;
else
    level = poset(adj,1)'-1;  
end;
y = (level+1)./(max(level)+2);
y = 1 - y;
x = zeros(size(y));
for i = 0:max(level),
    idx = find(level==i);
    offset = (rem(i,2)-0.5)/10;
    x(idx) = (1:length(idx))./(length(idx)+1)+offset;
end;

function [depth] = poset(adj, root)
% [depth] = poset(adj, root)   Identify a partial ordering among the nodes of a graph
% INPUT:   adj  -  adjacency matrix
%          root -  node to start with
% OUTPUT:  depth - Depth of the node
% Note     : All Nodes must be connected
adj = adj + adj';
N = size(adj,1);
depth = zeros(N,1);
depth(root) = 1;
queue = root;
while 1,
    if isempty(queue),
        if all(depth), 
            break; 
        else
            root = find(depth==0); 
            root = root(1);
            depth(root) = 1;
            queue = root;
        end
    end
    r = queue(1); queue(1) = [];
    idx = find(adj(r,:));
    idx2 = find(~depth(idx));
    idx = idx(idx2);
    queue = [queue idx];
    depth(idx) = depth(r) + 1;
end;

function [seq] = toposort(adj)
% [seq] = toposort(adj)  A Topological ordering of nodes in a directed graph
% INPUT:  adj  -  adjacency matrix 
% OUTPUT: seq  -  a topological ordered sequence of nodes
%                 or an empty matrix if graph contains cycles
N = size(adj);
indeg = sum(adj,1);
outdeg = sum(adj,2);
seq = [];
for i = 1:N,
    idx = find(indeg==0);    % Find nodes with indegree 0
    if isempty(idx),   % If can't find than graph contains a cycle
        seq = [];
        break;
    end;
    [dummy idx2] = max(outdeg(idx)); % Remove the node with the max number of connections
    indx = idx(idx2);
    seq = [seq, indx];
    indeg(indx) = -1;
    idx = find(adj(indx,:));
    indeg(idx) = indeg(idx)-1;
end 