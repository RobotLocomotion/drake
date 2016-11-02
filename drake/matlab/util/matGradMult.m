function [y] = matGradMult(A,b,transposeA)
% [y] = matGradMult(A,b)
% If A is the result of dC/dX for matrix C and vector X, as output by
% geval, then return d(C*b)/dX (a matrix).
% @param A the gradient of a matrix C with respect to X as output by geval
% @param b a vector
% @retval y a matrix, the result of d(C*b)/dX

if isempty(A)
  y = A; % output should be empty matrix of the same size as A
  return;
end

if (size(b,2)>1 || ~isnumeric(A) || ~isnumeric(b)) % call my other method
  n = size(b,1);
  m = size(A,2);
  k = size(A,1)/n;
  y = matGradMultMat(sparse(k,n),b,A,sparse(numel(b),m)); % with a minor abuse of notation
  return;
end

if nargin>2 && transposeA
  n = length(b);
  m = size(A,2);
  k = size(A,1)/n;
  y = zeros(k,m);
  for i=1:m,
    y(:,i) = reshape(A(:,i),n,k)'*b;
  end
  return;
end

n = length(b);
m = size(A,2);
k = size(A,1)/n;

B = sparse(repmat((1:k)',n,1),1:k*n,reshape(bsxfun(@times,b',ones(k,1)),k*n,1),k,k*n);
y = B*A;
% B_row=repmat(reshape(diag([b;zeros(k-n-1,1)]),1,(k-1)^2),1,k);
% B =  reshape(B_row(1:k*k*n),k*n,k)';
% y = B*A;
% return

% tensor = reshape(A,k,n,m);  tensor method was actually a little slower
% % (<10% difference)
% y = zeros(k,m); %was mxn-->changed to nxm then changed again
% for i=1:m,  %was n
%   y(:,i) = reshape(A(:,i),k,n)*b;
% %   y(:,i) = tensor(:,:,i)*b;
% end
end
