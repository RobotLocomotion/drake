function image_filename = drawGraph(adj,node_labels,varargin)

% Draws a graph using graphviz
%
% @param adj Adjacency matrix (source,sink).  If A is numeric, then
% A(i,j)~=0 indicates there is an edige from i to j.  If A is a cell array,
% then ~isempty(A{i,j}) implies that there is an edge, and A{i,j} is
% the edge label.
%
% @param node_labels is a cell array of strings with the node labels
%


dotfile = fullfile(tempdir,'matlabgraph.gv');
fptr=fopen(dotfile,'w');

fprintf(fptr,'digraph matlabgraph {\n');
fprintf(fptr,'  node []; \n');

n = max([length(node_labels),size(adj)]);

% set empty node_labels to be their index
if length(node_labels)<n, node_labels{n}={}; end
for i=1:n
    if isempty(node_labels{i})
        node_labels{i}=num2str(i);
    end
    fprintf(fptr,'    "%s";\n',node_labels{i});
end

for i=1:size(adj,1)
    for j=1:size(adj,2)
        if isnumeric(adj)
            if adj(i,j)
                fprintf(fptr,'    "%s" -> "%s";\n',node_labels{i},node_labels{j});
            end
        else
            if ~isempty(adj{i,j})
                if isnumeric(adj{i,j})
                    adj{i,j} = num2str(adj{i,j});
                end
                fprintf(fptr,'    "%s" -> "%s" [ label = "%s" ];\n',node_labels{i},node_labels{j},adj{i,j});
            end
        end
    end
end

fprintf(fptr,'}\n');
fclose(fptr);

%drawDot(dotfile,gca,varargin{:});
systemWCMakeEnv(['dot -Tpng -O ',dotfile]);
image_filename = [dotfile,'.png'];
if nargout<1
  disp = getenv('DISPLAY');
  if (strcmp(disp, ''))
    warning('You do not appear to have a valid DISPLAY, so I am not going to attempt to show the image.')
  else
    image(imread(image_filename));
    axis image; axis off
  end
end