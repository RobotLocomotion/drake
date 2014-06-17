function [J, vIndices, dJdq] = geometricJacobian(obj, kinsol, base, endEffector, expressedIn)
%GEOMETRICJACOBIAN Computes the geometric Jacobian from base to endEffector
% expressed in frame attached to expressedIn
%   The general contract of this method is that for joint velocity vector
%   v, the twist of endEffector with respect to base, expressed in
%   expressedIn, can be computed as J * (v(vIndices)).
%   In addition, vIndices is guaranteed to be `in order' in the following
%   two ways:
%   1) if the joint associated with `body' is on the path between base and
%   endEffector, then the velocity indices in body.velocity_num will appear
%   in the same order in vIndices
%   2) if body1 precedes body2 in the joint path going from base to
%   endEffector, then body1.velocity_num will precede body2.velocity_num in
%   vIndices.
%   Note that this guarantees that the last
%   length(endEffector.velocity_num) indices of vIndices are exactly
%   body.velocity_num

compute_gradient = nargout > 2;

[~, jointPath, signs] = findKinematicPath(obj, base, endEffector);
vIndices = vertcat(obj.body(jointPath).velocity_num);

if isempty(jointPath)
  J = zeros(6,0);
  dJdq = zeros(0, obj.num_positions); % TODO: test
  return;
end

JBlocks = cellfun(@times, kinsol.J(jointPath), num2cell(signs'), 'UniformOutput', false);
J = [JBlocks{:}];
% J = cell2mat(JBlocks); % doesn't work with TaylorVar

if compute_gradient
  dJdqBlocks = cellfun(@times, kinsol.dJdq(jointPath), num2cell(signs'), 'UniformOutput', false);
  dJdq = vertcat(dJdqBlocks{:});
end

if expressedIn ~= 1
  % change frame from world to expressedIn
  T = kinsol.T{expressedIn};
  TInv = inv(T);
  if compute_gradient
    dTInvdq = dinvT(T, kinsol.dTdq{expressedIn});
    dJdq = dAdHTimesX(TInv, J, dTInvdq, dJdq);
  end
  J = transformTwists(TInv, J);
end

end
