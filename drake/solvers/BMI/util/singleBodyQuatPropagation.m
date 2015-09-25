function quat = singleBodyQuatPropagation(parent_quat,tree_quat,body2joint_quat,theta_cos,theta_sin)
% For a RigidBody, given the orientation of its parent, and the joint
% angles of the link that connects the parent link to this link, return the
% orientation of this link
% @param parent_quat    The quaternion of the parent link
% @parem tree_quat      This is the quaternion of Ttree stored in RigidBody
% @param body2joint_quat   The quaternion of the body to joint
% @param theta_cos   The cosine of the joint angle
% @param theta_sin   The sine of the joint angle
joint2body_quat = quatConjugate(body2joint_quat);
quat = quatProduct(parent_quat,quatProduct(tree_quat,quatProduct(joint2body_quat,quatProduct([theta_cos;0;0;theta_sin],body2joint_quat))));
end