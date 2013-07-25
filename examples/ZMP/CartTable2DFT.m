function fr = CartTable2DFT
% coordinate frame for the cart-table force/torque sensor 
   
coords{1}=['force_x'];
coords{2}=['force_z'];
coords{3}='torque';

fr = SingletonCoordinateFrame('CartTable2DFT',3,'f',coords);
