classdef CartTable2DVisualizer < PlanarRigidBodyVisualizer
  % Inputs: CartTable2DState (x,z,p of the table, x of the cart + derivatives)
  
  methods
    function obj = CartTable2DVisualizer
      r = PlanarRigidBodyManipulator('CartTable.urdf',struct('floating',true,'view','right'));
      obj = obj@PlanarRigidBodyVisualizer(r);
    end
  end
  
end