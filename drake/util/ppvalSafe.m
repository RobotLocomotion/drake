function yy = ppvalSafe(pp,xx,bLimitStart,bLimitEnd)
% PPVALSAFE(pp,xx,bLimitStart,bLimitEnd)
%    Checks to see if xx is beyond the breaks in pp before evaluating
%    ppval.  If bLimitStart (or End) is true (default), then an error is thrown if
%    the spline is evaluated past its breaks.  If bLimitStart (or End) is
%    false, then the spline is evaluated at the last break (zero-order-hold
%    past the end of the spline.

if (nargin<3) bLimitStart = true; end
if (nargin<4) bLimitEnd = true; end

% update: don't need this anymore, since I added ppval to TaylorVar! 
%xx=double(xx);  % convert to double (e.g., from TaylorVar.  ppval can't handle taylorvars.  and it will warn if gradient data is lost)

sx = max(xx,pp.breaks(1));
if (bLimitStart && any(sx~=xx)) 
  error('attempted to evaluate past the beginning of the spline'); 
end

ex = min(sx,pp.breaks(end));
if (bLimitEnd && any(ex~=sx)) 
  error('attempted to evaluate past the end of the spline'); end

yy = ppval(pp,ex);
