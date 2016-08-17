function G = graspTransferMatrix(xc)
% Given the contact point xc, compute the grasp transfer matrix
% G = [I           I          ... I;
%      \hat{xc1}   \hat{xc2}  ... \hat{xcN}]
num_xc = size(xc,2);
skew_row = [2;3;1;3;1;2];
skew_col = [1;1;2;2;3;3];
skew_mat1 = sparse(reshape(bsxfun(@times,(1:3)',ones(1,num_xc)),[],1),(1:3*num_xc)',ones(3*num_xc,1));
skew_mat2 = sparse(reshape(bsxfun(@times,skew_row,ones(1,num_xc)),[],1),reshape(bsxfun(@plus,skew_col,3*(0:num_xc-1)),[],1),reshape([xc(3,:);-xc(2,:);-xc(3,:);xc(1,:);xc(2,:);-xc(1,:)],[],1));
G = [skew_mat1;skew_mat2];
end