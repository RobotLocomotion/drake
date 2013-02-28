 function [Gvec] = snfindG(iGfun, jGvar, Gfull)
%function [Gvec] = snfindG(iGfun, jGvar, Gfull)
%    Grabs elements in Gfull corresponding to
%    row and column indices (iGfun, jGvar).
%
%    Note: We cannot simply use the Matlab
%    function find(), since it is crucial that
%    the order of Gvec correspond to iGfun,
%    and jGvar.  Furthermore, zero
%    elements in Gfull must not be deleted
%    from Gvec.

Gind = sub2ind(size(Gfull), iGfun, jGvar);
Gvec = Gfull(Gind);

% Avoid Gvec being stored in sparse format:
Gvec  = full(Gvec);
[m,n] = size(Gvec);
if m == 1,
  Gvec = Gvec';
end


