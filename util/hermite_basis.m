function b = hermite_basis(p)
  p=grlex(p);
  function v = ip(f,g)
    Q = f.*g;
    [~,pw,M] = decomp(Q);
    if isempty(pw), pw = 0 ; end
    % Any odd powers result in zero.
    M(:,any(mod(pw,2),2)) = [];
    pw(any(mod(pw,2),2),:) = [];
    K = 1+max(pw(:))/2;
    gauss_int = zeros(K+1,1);
    for r = 0:K
      gauss_int(r+1,1) = factd(2*r-1);
    end
    v = reshape(M*prod(reshape(gauss_int(1+pw/2),size(pw)),2),size(Q,1),size(Q,2));
  end
  b = orthogonal_basis(p,@ip);
end

% b = chebyshev_basis(p)
% p -- K-by-1 msspoly of linearly independent polynomials.
% b -- K-by-1 msspoly of polynomials orthogonal w.r.t:
%      <f,g> = int_{-1}^1 ... int_{-1}^1 
%              f(x)*g(x)*((1-x1^2)...(1-xn^2))^(-1/2) dx_1 ... dx_n
function b = orthogonal_basis(p,ip)
  b  = p;
  b(1) = b(1)/sqrt(ip(b(1),b(1)));
  for i = 2:length(p)
    b(i) = p(i) - ip(p(i)*ones(i-1,1),b(1:i-1))'*b(1:i-1);
    b(i) = b(i)/sqrt(ip(b(i),b(i)));
  end
end



function m = grlex(p)
  [x,p,M] = decomp(p);
  if any(sum(M==1,2) ~= 1)
    error('p must be monomials');
  end
  [~,I1] = sortrows(fliplr(p));
  [~,I2] = sort(sum(p(I1,:),2));
  m = recomp(x,p(I1(I2),:),speye(size(p,1)));
end



function [f] = factd(n)
%FACTD Double Factorial function = n!! 
%
%usage: f = factd(n)
%
%tested under version 5.3.1
%
%     This function computes the double factorial of N.
%     N may be complex and any size. Uses the included 
%     complex Gamma routine.
%
%     f = n*(n-2)*(n-4)*...*5*3*1 for n odd
%     f = n*(n-2)*(n-4)*...*6*4*2 for n even
%
%see also: Gamma, Fact
%
%Paul Godfrey
%pgodfrey@conexant.com
%8-29-00

[siz]=size(n);
n=n(:);

p=cos(pi*n)-1;

f=2.^((-p+n+n)/4).*pi.^(p/4).*gamma(1+n/2);

p=find(round(n)==n & imag(n)==0 & real(n)>=-1);
if ~isempty(p)
f(p)=round(f(p));
end

p=find(round(n/2)==(n/2) & imag(n)==0 & real(n)<-1);
if ~isempty(p)
f(p)=Inf;
end

f=reshape(f,siz);

return

%a demo of this routine is
n=-10:10;
n=n(:);
f=factd(n);
[n f]

ezplot factd
grid on
return

end
