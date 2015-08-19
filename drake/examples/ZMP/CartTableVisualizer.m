classdef CartTableVisualizer < RigidBodyWRLVisualizer
  % Inputs: CartTable2DState (x,z,p of the table, x of the cart + derivatives)
  
  methods
    function obj = CartTableVisualizer
      r = RigidBodyManipulator('CartTable.urdf',struct('floating',true));
      obj = obj@RigidBodyWRLVisualizer(r);
    end
  end
  
end