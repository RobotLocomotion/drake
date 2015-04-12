function generateGradients(fun,order,fname,varargin)
% Generate Gradients
%   Symbolic gradients can be a lot faster if you compute them once and
%   generate a file.
%   Usage: generateGradients(fun,order,filename,a1,a2,...,an[,options])
%
%   @param fun is a function handle to the function you want to provide gradients for
%   @param order is the order of the Taylor expansion (0 is no gradients, 1 is
%       linearization, ...0
%   @param fname is the filename to write result to.  if fname is empty,
%       then just return the symbolic structures
%   @param a1,a2,...  are some nominal inputs, which are just used to provide
%       information about the size and types of the inputs.  If the first
%       argument is a matlab class, then the code will be generated
%       appropriately (using the class properties as parameters when the
%       generated function is evaluated).
%       Note: an (the last input) must not be a struct... or it will be
%       confused with an options struct (if this is a problem, then the
%       remedy is just to add an extra empty options struct on the end)
%   @option bSimplify - boolean (default: true)
%
%   Example:
%     cd examples/Pendulum
%     p=PendulumPlant();
%     generateGradients('dynamics',3,'dynamicsGradients',p,0,randn(2,1),0);

% note: could support multiple input arguments as classes, but then i have
% to worry about duplicate parameter names, etc, etc.

typecheck(fun,{'function_handle','char'});
if (nargout(fun)>1) warning('Your function has multiple outputs, but\nI currently only generate gradients for the first output'); end
  
typecheck(order,'double'); sizecheck(order,[1 1]);
if (order<1) error('order should be >=1'); end

typecheck(fname,'char');
if (length(varargin)<1) error('fun must have at least one input'); end
if (isstruct(varargin{end}))
  options = varargin{end};
  varargin = {varargin{1:end-1}};
else
  options = struct();
end
if (~isfield(options,'simplify')) options.simplify = true; end

reserved_strs={'order','df'};
for o=2:order
  reserved_strs={reserved_strs{:},['d',num2str(o),'f']};
end
for i=1:length(varargin)
  reserved_strs={reserved_strs{:},['a',num2str(i)]};
  for j=1:prod(size(varargin{i}))
    reserved_strs={reserved_strs{:},['a',num2str(i),'_',num2str(j)]};
  end
end

s=[];
for i=1:length(varargin)
  if (i==1 && isobject(varargin{i}))
    a{i}=varargin{i};
    fields = fieldnames(a{i});
    for j=1:length(fields) 
      % check to make sure we don't overlap with the few important
      % variables:
      if (any(strcmp(fields{j},reserved_strs)))
        error([fields{j},' is a reserved string name']);
      end
      
      % make the sym:
      try
        fieldval=getfield(a{i},fields{j});
        if (isscalar(fieldval))
          placeHolder=sym(fields{j},'real');
          a{i}=setfield(a{i},fields{j},placeHolder);
        else
          if (exist('ai')), clear ai; end
          for k=1:prod(size(fieldval))
            ai(k)= sym([fields{j},'_',num2str(k)],'real');
          end
          a{i}=setfield(a{i},fields{j},reshape(ai,size(fieldval)));
        end
      catch exception
        string = ['Variable ' fields{j} ' not symbolicisized'];
        disp(string);
      end
    end
  elseif (isnumeric(varargin{i}))
    if (exist('ai')), clear ai; end
    for j=1:prod(size(varargin{i}))
      ai(j)=sym(['a',num2str(i),'_',num2str(j)],'real');
    end
    a{i}=reshape(ai,size(varargin{i}));
    s=[s;ai(:)];
  else
    if (i==1)
      error('a1 must be an object or numeric');
    else
      error(['a',num2str(i),' must be numeric']);
    end
  end
end

if (isempty(s))
  error('don''t have any numerical inputs to grad gradients with respect to'); 
end

% now call the actual function
f = feval(fun,a{:});
% simplify
if (options.simplify) f = simplify(f); end

m = prod(size(f));
n = length(s);

% differentiate dynamics symbolically
disp('Generating order 1 gradients...');
df{1} = jacobian(reshape(f,m,1),s);
if (options.simplify) df{1} = simplify(df{1}); end    
for o=2:order
  disp(['Generating order ',num2str(o),' gradients...']);
  df{o} = reshape(jacobian(reshape(df{o-1},m*n^(o-1),1),s),m,n^o);
if (options.simplify) df{o} = simplify(df{o}); end            
end
disp('Writing gradients to file...');

%% write file
% strip off .m from filename if it came in that way

ind=find(fname=='.',1);
if (~isempty(ind)) fname=fname(1:(ind-1)); end
mfile = sprintf('%s.m',fname);

% open file and write header
fp = fopen(mfile,'w');
fprintf(fp,'function [df');
for o=2:order
  fprintf(fp,', d%df',o);
end
fprintf(fp,'] = %s(a1',fname);
for i=2:length(varargin)
  fprintf(fp,', a%d',i);
end
fprintf(fp,', order)\n');
fprintf(fp,'%% This is an auto-generated file.\n%%\n%% See <a href="matlab: help generateGradients">generateGradients</a>. \n\n');

% check inputs
fprintf(fp,'%% Check inputs:\n');

if (isobject(varargin{1}))
  fprintf(fp,'typecheck(a1,''%s'');\n',class(varargin{1}));
end
fprintf(fp,'if (nargin<%d) order=1; end\n',length(varargin));
fprintf(fp,'if (order<1) error('' order must be >= 1''); end\n');
for i=1:length(varargin)
  fprintf(fp,'sizecheck(a%d,[%s]);\n',i,num2str(size(varargin{i})));
end

% write symbols
fprintf(fp,'\n%% Symbol table:\n');
v = symvar(f);
for i=1:length(v)
  vi = char(v(i));
  if (any(strcmp(vi,reserved_strs))) % then it's ai_j
    ind=find(vi=='_');  if (isempty(ind)) error('shouldn''t get here'); end
    i=str2num(vi(2:ind-1));
    j=str2num(vi(ind+1:end));
    fprintf(fp,'%s=a%d(%d);\n',vi,i,j);
  else % then it must be a field from the object
      % check to see if it is inside a matrix in a field from the object
      ind=find(vi=='_');
      if (isempty(ind))
          fprintf(fp,'%s=a1.%s;\n',vi,vi);  
      else
          ind = ind(end);
          % this is in a property that is a matrix
          property_name = vi(1:ind-1);
          j=str2num(vi(ind+1:end));
          fprintf(fp,'%s=a1.%s(%d);\n',vi,property_name,j);
      end
  end
end

fprintf(fp,'\n\n%% Compute Gradients:\n');
write_symbolic_matrix(fp,df{1},'df');
for o=2:order
  symbol = ['d',num2str(o),'f'];
  fprintf(fp,'\n%% %s\n',symbol);
  fprintf(fp,'if (order>=%d)\n',o);
  write_symbolic_matrix(fp,df{o},['  ',symbol]);
  fprintf(fp,'else\n  d%df=[];\nend  %% if (order>=%d)\n',o,o);
end

fprintf(fp,'\n\n %% NOTEST\n'); 

% close file
fclose(fp);

% write test file
mkdir('.','test/');
in = varargin;
save(['test/',fname,'.mat'],'fun','in');
fp = fopen(['test/',fname,'Test.m'],'w');
fprintf(fp,'function %sTest()\n',fname);
fprintf(fp,'%% Tests user gradients vs TaylorVar gradients to check\n%% consistency (e.g., should break if you update\n%% the original function you autogenerated gradients for but\n%% forgot to re-generate the gradients.\n\n');
fprintf(fp,'oldpath=addpath(''..'');\n');
fprintf(fp,'load %s.mat;\n',fname);
fprintf(fp,'\nf1=cell(1,%d);\n',order+1);
fprintf(fp,'[f1{:}]=geval(fun,in{:},struct(''grad_method'',''user''));\n');
fprintf(fp,'\nf2=cell(1,%d);\n',order+1);
fprintf(fp,'[f2{:}]=geval(fun,in{:},struct(''grad_method'',''taylorvar''));\n');
fprintf(fp,'\nfor i=1:%d\n',order+1);
fprintf(fp,'  if (any(abs(f1{i}(:)-f2{i}(:))>1e-5))\n');
fprintf(fp,'    path(oldpath);\n');
fprintf(fp,'    error(''gradients don''''t match!'');\n');
fprintf(fp,'  end\n');
fprintf(fp,'end\n\n');
fprintf(fp,'path(oldpath);\n');
fclose(fp);

fprintf(1,'Success!  Gradient code written to %s.\n',mfile);
fprintf(1,'I''ve also written a test code to test/%sTest.m, to check consistency of the autogenerated code (e.g., in case you change the original function without regenerating the gradients).\n',fname);
fprintf(1,'Now add the following lines to the BEGINNING of your existing function (make sure that your input values are not modified before calling this):\n\n');
fprintf(1,'  if (nargout>1)\n');
fprintf(1,'    [df');
for o=2:order
  fprintf(1,',d%df',o);
end
fprintf(1,']= %s(a1',fname);
for i=2:length(varargin)
  fprintf(1,',a%d',i);
end
fprintf(1,',nargout-1);\n  end\n');
fprintf(1,'where you must replace a1 - an with the actual names of your inputs.\n\n');

end

function write_symbolic_matrix(fp,A,symbol)
  [m,n]=size(A);
  fprintf(fp,'%s = sparse(%d,%d);\n',symbol,m,n);
  for i=1:m, for j=1:n, if (A(i,j)~=0) fprintf(fp,'%s(%d,%d) = %s;\n',symbol,i,j,char(A(i,j))); end; end; end
end

