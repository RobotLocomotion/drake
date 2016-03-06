function quat = singleBodyQuatPropagation(parent_quat,tree_quat,joint_axis,theta_cos,theta_sin)
% For a RigidBody, given the orientation of its parent, and the joint
% angles of the link that connects the parent link to this link, return the
% orientation of this link
% @param parent_quat    The quaternion of the parent link
% @parem tree_quat      This is the quaternion of Ttree stored in RigidBody
% @param joint_axis   The axis of the joint in the body frame
% @param theta_cos   The cosine of the joint angle
% @param theta_sin   The sine of the joint angle
quat = quatProduct(parent_quat,quatProduct(tree_quat,[theta_cos;joint_axis*theta_sin]));
end