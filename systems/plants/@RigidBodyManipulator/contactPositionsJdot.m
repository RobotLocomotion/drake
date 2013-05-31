function [contact_pos,J,Jdot] = contactPositionsJdot(obj,kinsol,body_idx,body_contacts)
% @retval p(:,i)=[px;py;pz] is the position of the ith contact point 
% @retval J is dpdq
% @retval Jdot is d(dpdq)dt
      
typecheck(kinsol,'struct');

if nargin<3
  body_idx = 1:length(obj.body);
end

if nargin<4
  n_contact_pts = size([obj.body(body_idx).contact_pts],2);
  body_contacts = [];
else
  n_contact_pts = sum(cellfun('length',body_contacts));
end

d=length(obj.gravity);  % 2 for planar, 3 for 3D
contact_pos = zeros(d,n_contact_pts)*kinsol.q(1);

if (nargout>1) 
  J = zeros(n_contact_pts*d,obj.num_q); 
end
if (nargout>2)
  Jdot = zeros(n_contact_pts*d,obj.num_q); 
end

count=0;
for i=1:length(body_idx)
  if isempty(body_contacts)
    nC = size(obj.body(body_idx(i)).contact_pts,2);
    pts_idx = 1:nC;
  else
    pts_idx = body_contacts{i};
    nC = length(pts_idx);
  end
  if nC>0
    if (nargout>2)
      [contact_pos(:,count+(1:nC)),J(d*count+(1:d*nC),:)] = forwardKin(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts(:,pts_idx));
      Jdot(d*count+(1:d*nC),:) = forwardJacDot(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts(:,pts_idx));
    elseif (nargout>1)
      [contact_pos(:,count+(1:nC)),J(d*count+(1:d*nC),:)] = forwardKin(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts(:,pts_idx));
    else
      contact_pos(:,count+(1:nC)) = forwardKin(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts(:,pts_idx));
    end
    count = count + nC;
  end
end

end
