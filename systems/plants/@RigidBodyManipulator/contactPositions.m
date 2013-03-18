function [contact_pos,J,dJ] = contactPositions(obj,kinsol,body_idx)
% @retval p(:,i)=[px;py;pz] is the position of the ith contact point 
% @retval J is dpdq
% @retval dJ is ddpdqdq
      
if ~isstruct(kinsol)  
  % treat input as contactPositions(obj,q)
  kinsol = doKinematics(obj,kinsol,nargout>2);
end

if nargin<3
  body_idx = 1:length(obj.body);
end

d=length(obj.gravity);  % 2 for planar, 3 for 3D
contact_pos = zeros(d,size([obj.body(body_idx).contact_pts],2));  
if (nargout>1) 
  J = zeros(size([obj.body(body_idx).contact_pts],2)*d,obj.num_q); 
end
if (nargout>4)
  dJ = sparse(size([obj.body(body_idx).contact_pts],2)*d,obj.num_q^2);
end

count=0;
for i=1:length(body_idx)
  nC = size(obj.body(body_idx(i)).contact_pts,2);
  if nC>0
    if (nargout>2)
      [contact_pos(:,count+(1:nC)),J(d*count+(1:d*nC),:),dJ(d*count+(1:d*nC),:)] = forwardKin(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts);
    elseif (nargout>1)
      [contact_pos(:,count+(1:nC)),J(d*count+(1:d*nC),:)] = forwardKin(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts);
    else
      contact_pos(:,count+(1:nC)) = forwardKin(obj,kinsol,body_idx(i),obj.body(body_idx(i)).contact_pts);
    end
    count = count + nC;
  end
end

end
