function y = linspacevec(d1,d2,n)
% function y = linspacevec(d1,d2,n)
% acts like linspace, but d1 and d2 may be column vectors

if nargin == 2
    n = 100;
end
y = repmat(d1,1,n) + (d2-d1)*(0:n-1)/(n-1);

end