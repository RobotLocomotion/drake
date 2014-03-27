function [contact_pos,J,dJ] = contactPositions(obj,kinsol,body_idx,body_contacts)
error('The contactPositions function has been deprecated. See contactConstraints and collisionDetect')
% % @param body_idx is an array of body indexes
% % @param body_contacts is a cell array of vectors containing contact point indices
% % 
% % @retval p(:,i)=[px;py;pz] is the position of the ith contact point 
% % @retval J is dpdq
% % @retval dJ is ddpdqdq
% % @ingroup Collision
%       
% if ~isstruct(kinsol)  
%   % treat input as contactPositions(obj,q)
%   kinsol = doKinematics(obj,kinsol,nargout>2);
% end
% 
% if nargin<3
%   body_idx = 1:length(obj.body);
% end
% 
% if nargin<4
%   n_contact_pts = size([obj.body(body_idx).contact_pts],2);
%   body_contacts = [];
% else
%   if isa(body_contacts,'cell')
%     n_contact_pts = sum(cellfun('length',body_contacts));
%   else
%     n_contact_pts = length(body_contacts);
%     body_contacts = {body_contacts};
%   end
% end
% 
% d=3; %obj.dim;  % 2 for planar, 3 for 3D
% contact_pos = zeros(d,n_contact_pts)*kinsol.q(1);
% 
% if (nargout>1) 
%   J = zeros(n_contact_pts*d,obj.num_q)*kinsol.q(1); 
% end
% if (nargout>2)
%   dJ = sparse(n_contact_pts*d,obj.num_q^2);
% end
% 
% count=0;
% for i=1:length(body_idx)
%   if isempty(body_contacts)
%     nC = size(obj.body(body_idx(i)).contact_pts,2);
%     pts_idx = 1:nC;
%   else
%     pts_idx = body_contacts{i};
%     nC = length(pts_idx);
%   end
%   if nC>0
%     if (nargout>2)
%       [contact_pos(:,count+(1:nC)),J(d*count+(1:d*nC),:),dJ(d*count+(1:d*nC),:)] = forwardKin(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts(:,pts_idx));
%     elseif (nargout>1)
%       [contact_pos(:,count+(1:nC)),J(d*count+(1:d*nC),:)] = forwardKin(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts(:,pts_idx));
%     else
%       contact_pos(:,count+(1:nC)) = forwardKin(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts(:,pts_idx));
%     end
%     count = count + nC;
%   end
% end
% 
% end
