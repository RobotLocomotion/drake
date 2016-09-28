function gradTest(FUN,varargin)

% GRAD TEST
%   allows you to visually check the gradients of a function 
%         [f,df] = my_function(a1,a2,a3,...)
%   where f is the output and df{1} = [dfda1,dfda2,dfda3,...], 
%   df{2..N} would be the second through Nth derivatives 
%   (but are not checked here - at least not yet).
%
%   call this function with
%   gradTest(@my_function,a1_0,a2_0,a3_0,...,options);
%   where ai_0 is the nominal value of ai (about which to linearize).  ai
%   can be a vector.
%
%   Options:
%       options.scale  - cell array, which scale{i} is a vector
%       containing the magnitude to perturb the elements of input i.  the
%       default is .1*ones(size(ai));
%          Example:  
%              struct('scale',{{1,.1*ones(13,1),.1*ones(2,1)}})    
%            for f(t,x(1:13),u(1:2))
%       alternatively, if options.scale is a scalar, then it sets the scale
%       to options.scale*ones(size(ai))
%
%       options.tol - compare gradients to finite differences, and only
%       plot the result for a particulare input/output pair if it 
%       exceeds the tolerance percentage of the true gradient (default: 0 => plot everything)
%
%       options.input_name -  cell array with the name of the input variables for
%       plotting purposes.  
%
%       options.output_name -  name of the output variable for
%       plotting purposes.  
%
%       options.num_samples  - number of samples to plot for each dimension
%       (default is 21).
%
%       options.drawfun - function pointer of the form
%             drawfun(a1,a2,a3,...,f) 
%       which displays the numerical samples (to help in debugging)
%
%   Example: (for the simple pendulum)
%      while(1)
%        t0=0;  x0=randn(2,1); u0=randn;
%        gradTest(@dynamics,t0,x0,u0,struct('input_names',{{'t','x','u'}},'output_name','xdot'));
%        gradTest(@cost,t0,x0,u0,struct('input_names',{{'t','x','u'}},'output_name','g'));
%        gradTest(@finalcost,t0,x0,struct('input_names',{{'t','x'}},'output_name','h'));
%      end
%


if (isstruct(varargin{end}))
  options = varargin{end};
  varargin = {varargin{1:end-1}};
else
  options = struct();
end

if (~isfield(options,'scale'))
  for i=1:length(varargin)
    options.scale{i} = .1*ones(size(varargin{i}));
  end
elseif isscalar(options.scale)
  scale = options.scale;
  options.scale = {};
  for i=1:length(varargin)
    options.scale{i} = scale*ones(size(varargin{i}));
  end
end
%if (~iscell(options.scale)) options.scale = {options.scale}; end
if (~isfield(options,'input_name'))
  for i=1:length(varargin)
    options.input_name{i} = ['a',num2str(i)];
  end
end
if (~isfield(options,'output_name')) options.output_name = 'f'; end
if (~isfield(options,'num_samples')) options.num_samples = 31; end
if (~isfield(options,'tol')) options.tol = 0.01; end

[f,df] = FUN(varargin{1:end});
f = reshape(f,[],1);
if (~iscell(df)) a{1} = df; df=a; clear a; end


input_ind = 0;
for v=1:length(varargin)
  for b=1:19, fprintf(1,' '); end
  x = varargin{v};
  if ~isnumeric(x), continue; end
  
  for i=1:length(x)
    for b=1:19, fprintf(1,'\b'); end
    fprintf(1,'grad %5d of %5d',i,length(x));
    samples = linspace(-options.scale{v}(i),options.scale{v}(i),options.num_samples);
    ind=ceil((options.num_samples-eps)/2);
    for s=1:options.num_samples
      y(:,s) = reshape(FUN(varargin{1:v-1},varargin{v}+[zeros(i-1,1);samples(s);zeros(length(x)-i,1)],varargin{v+1:end}),[],1);
    end
    for j=1:length(f)
      grad = df{1}(j,input_ind+i);
      numgrad(1) = (y(j,ind+1) - y(j,ind))/(samples(ind+1)-samples(ind));
      numgrad(2) = (y(j,ind) - y(j,ind-1))/(samples(ind)-samples(ind-1));
      
      % to better support discontinuities and finite sampling inaccuracies,
      % anything between numgrad(1) and numgrad(2) is considered zero error.
      numgrad = sort(numgrad);
      if (grad<numgrad(1)) numerr=numgrad(1)-grad; 
      elseif (grad>numgrad(2)) numerr=grad-numgrad(2);
      else numerr=0; 
      end
      
      if (numerr>=options.tol)
        sfigure(109); clf
        h=plot(x(i)+samples,y(j,:),'b.-',x(i),f(j),'r*');
        if (iscell(options.input_name{v}))
          xlabel(options.input_name{v}{i})
        else
          xlabel([options.input_name{v},'(',num2str(i),')']);
        end
        if (iscell(options.output_name))
          ylabel(options.output_name{j});
        else
          ylabel([options.output_name, '(',num2str(j),')']);
        end
        h=[h;line(x(i)+options.scale{v}(i)*[-1,1],f(j)+options.scale{v}(i)*numgrad(1)*[-1,1],'color',[0 1 0])];
        h=[h;line(x(i)+options.scale{v}(i)*[-1,1],f(j)+options.scale{v}(i)*numgrad(2)*[-1,1],'color',[0 1 0])];
        h=[h;line(x(i)+options.scale{v}(i)*[-1,1],f(j)+options.scale{v}(i)*df{1}(j,input_ind+i)*[-1,1],'color',[1 0 0])];
%        title(['abs(1-numgrad/grad) = ',num2str(numerr)]);
        axis tight;
        legend(h([1 3 4 5]),'sampled','numgrad1','numgrad2','grad');
        if (isfield(options,'drawfun')) 
          for k=1:options.num_samples
            sfigure(109);
            hold on;
            h=plot(x(i)+samples(k),y(j,k),'go');
            options.drawfun(varargin{1:v-1},varargin{v}+[zeros(i-1,1);samples(k);zeros(length(x)-i,1)],varargin{v+1:end},y(:,k));
            delete(h);
            hold off;
          end
        end
        if (options.tol>0)
          error('gradients do not match to tolerance');
        else
          pause;
        end
      end
    end
    clear y;
  end
  input_ind = input_ind + length(x);
  fprintf(1,'\n');
end


end