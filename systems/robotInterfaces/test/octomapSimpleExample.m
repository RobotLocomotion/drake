function octomapSimpleExample
% matlab implementation of octomap/octomap/src/simple_example.cpp

tree = OcTree(0.1);

% insert some measurements of occupied cells
[x,y,z]=ndgrid(-20:20,-20:20,-20:20);
tree.updateNode(.05*[x(:)';y(:)';z(:)'],true);

% insert some measurements of free cells
[x,y,z]=ndgrid(-30:30,-30:30,-30:30);
tree.updateNode(.02*[x(:)';y(:)';z(:)']-1,false);

tree.search([0;0;0])
tree.search([-1;-1;-1])
tree.search([1;1;1])
