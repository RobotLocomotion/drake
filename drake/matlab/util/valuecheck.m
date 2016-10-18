function [tf,errstr] = valuecheck(val,desired_val,tol)

% VALUECHECK
%   Usage:  valuecheck(val,desired_val)

if (nargin<3 || isempty(tol)) tol=1e-8; end

tf = true;
errstr = '';

if isscalar(desired_val) && ~isempty(val)
  desired_val = repmat(desired_val,size(val));
end

if ((length(size(val))~=length(size(desired_val))) || any(size(val)~=size(desired_val)))
  errstr = ['Wrong size.  Expected ', mat2str(size(desired_val)),' but got ', mat2str(size(val))];
  tf = false;
  if (nargout>0)
    return;
  end
end

if (tf && ~isequal(isnan(val(:)),isnan(desired_val(:))))
  errstr = 'NANs don''t match. ';
  if any(isnan(val(:)))
    val
    [a,b] = ind2sub(find(isnan(val(:))),size(val));
    errstr = [errstr,sprintf('Found NANs at \n'),sprintf('(%d,%d) ',[a;b]),sprintf('\n')];
  else
    errstr = [errstr,'val has no NANs'];
  end
  if any(isnan(desired_val(:)))
    desired_val
    [a,b] = ind2sub(size(desired_val),find(isnan(desired_val(:))));
    errstr = [errstr,sprintf('but desired_val has them at \n'), sprintf('(%d,%d)',[a;b]),sprintf('\n')];
  else
    errstr = [errstr,'but desired_val has no NANs'];
  end
  %    err = desired_val-val;
  %    err(abs(err)<tol)=0;
  %    err=sparse(err)
  
  tf = false;
  if (nargout>0)
    return;
  end  
end

if (tf && any(abs(val(:)-desired_val(:))>tol))
  if (ndims(val)<=2 && length(val)<=6)
    % clean before printing
    desired_val(abs(desired_val)<tol/2)=0;
    val(abs(val)<tol/2)=0;
    errstr = sprintf('Values don''t match.  Expected \n%s\n but got \n%s', mat2str(desired_val), mat2str(val));
  else
    err = desired_val-val;
    errstr = size(desired_val);
    % print sparse-matrix-like format, but support ND arrays:
    ind=find(abs(err(:))>tol);
    a = cell(1,length(errstr));
    [a{:}] = ind2sub(errstr,ind);
    errstr = '';
    for i=1:numel(ind)
      b = cellfun(@(b) b(i),a);
      indstr = ['(',sprintf('%d,',b(1:end-1)), sprintf('%d)',b(end))];
      errstr = [errstr, sprintf('%10s %12f %15f\n',indstr,full(val(ind(i))),full(desired_val(ind(i))))];
    end
    errstr = sprintf('Values don''t match.\n    Index       Value       Desired Value\n   -------     --------    ---------------\n%s', errstr);
  end
  
  tf = false;
  if (nargout>0)
    return;
  end
end

if ~tf, 
  error('Drake:ValueCheck',errstr); 
end
