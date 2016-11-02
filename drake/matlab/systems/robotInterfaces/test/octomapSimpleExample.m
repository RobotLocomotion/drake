function octomapSimpleExample
% matlab implementation of octomap/octomap/src/simple_example.cpp

tree = OcTree(0.1);

% insert some measurements of occupied cells
[x,y,z]=ndgrid(-20:20,-20:20,-20:20);
tree.updateNode(.05*[x(:)';y(:)';z(:)'],true);

% insert some measurements of free cells
[x,y,z]=ndgrid(-30:30,-30:30,-30:30);
tree.updateNode(.02*[x(:)';y(:)';z(:)']-1,false);

% insert random point cloud block scan
% tree.insertPointCloud([3xN Pointcloud, 3xN point3d (sensor origin)])
% changing the sensor origin does not seem to have an effect on the point
% cloud insertion... will look further into this.

tree.insertPointCloud(10*rand(3,1000),[0;0;0]);
tree.insertPointCloud(10*(rand(3,1000)-[ones(1,1000);zeros(1,1000);zeros(1,1000)]),[0;0;0]);
tree.insertPointCloud(10*(rand(3,1000)-[zeros(1,1000);ones(1,1000);zeros(1,1000)]),[0;0;0]);
tree.insertPointCloud(10*(rand(3,1000)-[ones(1,1000);ones(1,1000);zeros(1,1000)]),[0;0;0]);
% % perform some queries
% tree.search([0;0;0])
% tree.search([-1;-1;-1])
% tree.search([1;1;1])

% display the tree (note: an lcmgl visualizer needs to be open already)
tree.enableLCMGL('octree');
tree.publishLCMGL();
