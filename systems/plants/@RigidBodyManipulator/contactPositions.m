function [p,J,dJ] = contactPositions(obj,kinsol)
% @retval p(:,i)=[px;py;pz] is the position of the ith contact point 
% @retval J is dpdq
% @retval dJ is ddpdqdq
      
if ~isstruct(kinsol)  
  % treat input as contactPositions(obj,q)
  kinsol = doKinematics(obj,kinsol,nargout>2);
end

contact_pos = zeros(3,obj.num_contacts); 
if (nargout>1) J = zeros(3*obj.num_contacts,obj.num_q); end
count=0;
for i=1:length(obj.body)
  nC = size(obj.body(i).contact_pts,2);
  if nC>0
    if (nargout>2)
      [contact_pos(:,count+(1:nC)),J(3*count+(1:3*nC),:),dJ(3*count+(1:3*nC),:)] = forwardKin(obj,kinsol,i,obj.body(i).contact_pts);
    elseif (nargout>1)
      [contact_pos(:,count+(1:nC)),J(3*count+(1:3*nC),:)] = forwardKin(obj,kinsol,i,obj.body(i).contact_pts);
    else
      contact_pos(:,count+(1:nC)) = forwardKin(obj,kinsol,i,obj.body(i).contact_pts);
    end
    count = count + nC;
  end
end
p = contact_pos;
end
