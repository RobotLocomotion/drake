classdef Trajectory < DrakeSystem
  
  properties
    dim
    tspan
  end
  
  methods (Abstract=true)
    y = eval(obj,t);
    t = getBreaks(obj); 
  end
 
  methods 
    function obj = Trajectory(dim)
      if (isnumeric(dim))
        numout = prod(dim);
      else
        numout = 1;  % just output nan for now
      end
      obj = obj@DrakeSystem(0,0,0,numout,false,false);
      obj.dim = dim;
    end
    
    function x0 = getInitialState(obj)
      x0 = []; % no state
    end

    function xcdot = dynamics(obj,t,x,u)
      xcdot = []; % no state
    end
    
    function xdn = update(obj,t,x,u)
      xdn = [];  % no state
    end
    
    function y = output(obj,t,x,u)
      if (isnumeric(obj.dim))
        y = reshape(obj.eval(t),[],1);
      else 
        y=nan;
      end
    end
    
    function dtraj = fnder(obj)
      error('not implemented yet');
    end

    function ydot = deriv(obj,t)
      ydot = eval(fnder(obj),t);
    end
    
    function yddot = dderiv(obj,t)
      yddot = eval(fnder(fnder(obj)),t);
    end
    
    function mobj = inFrame(obj,frame)
      if (obj.getOutputFrame==frame)
        mobj=obj;
      else
        tf = findTransform(obj.getOutputFrame,frame);
        if isempty(tf) error('couldn''t find a coordinate transform from the trajectory frame %s to the requested frame %s',obj.getOutputFrame.name,frame.name); end
        mobj = FunctionHandleTrajectory(@(t) tf.output(t,[],obj.eval(t)), frame.dim, obj.getBreaks);
        mobj = setOutputFrame(mobj,frame);
        mobj = setSampleTime(mobj,obj.getSampleTime);
      end
    end
    
    function traj = trajfun(traj,fhandle)
      % analagous to cellfun, this returns a new trajectory which is
      % equivalent to applying fhandle to traj at every t.
      
      error('not implemented yet');  
    end
    
    function mobj = uminus(obj)
      mobj = FunctionHandleTrajectory(@(t)-obj.eval(t),obj.dim,obj.getBreaks,@(t)-obj.deriv(t));
    end
    
    function [a,b,breaks] = setupTrajectoryPair(a,b)
        if (isnumeric(a)) a = ConstantTrajectory(a); end
        typecheck(a,'Trajectory'); 
        if (isnumeric(b)) b = ConstantTrajectory(b); end
        typecheck(b,'Trajectory');
        tspan = [max(a.tspan(1),b.tspan(1)),min(a.tspan(2),b.tspan(2))];
        if (tspan(2)<tspan(1))
          error('Drake:Trajectory:IncompatibleTimesForConcatenation','concatenated trajectories do not overlap in time');
        end
        
        breaks = unique([reshape(a.getBreaks,[],1);reshape(b.getBreaks,[],1)]);
        breaks = breaks(breaks>=tspan(1) & breaks<=tspan(2));
    end
    
    function b = ctranspose(a)
      s = size(a);
      b = FunctionHandleTrajectory(@(t) ctranspose(a.eval(t)),s([end,1:end-1]),a.getBreaks);
    end
    
    function c = vertcat(a,varargin)
      c=a;
      for i=1:length(varargin)
        [c,b,breaks]=setupTrajectoryPair(c,varargin{i});
        if (length(c.dim)~=length(b.dim) || any(c.dim(2:end)~=b.dim(2:end)))
          error('dimensions 2:end must match');
        end
        c = FunctionHandleTrajectory(@(t) vertcat(c.eval(t),b.eval(t)),[c.dim(1)+b.dim(1),c.dim(2:end)],breaks);
        c = setOutputFrame(MultiCoordinateFrame({getOutputFrame(c),getOutputFrame(b)}));
      end
    end
    
    function c = horzcat(a,varargin)
      c=a;
      for i=1:length(varargin)
        [c,b,breaks]=setupTrajectoryPair(c,varargin{i});
        cdim=size(c); bdim=size(b);
        if (length(cdim)~=length(bdim) || any(cdim([1,3:end])~=bdim([1,3:end])))
          error('dimensions 1 and 3:end must match');
        end
        c = FunctionHandleTrajectory(@(t) horzcat(c.eval(t),b.eval(t)),[cdim(1),cdim(2)+bdim(2),cdim(3:end)],breaks);
        c = setOutputFrame(MultiCoordinateFrame({getOutputFrame(c),getOutputFrame(b)}));
      end
    end
    
    function c = power(a,b)
      [a,b,breaks]=setupTrajectoryPair(a,b);
      c = FunctionHandleTrajectory(@(t) a.eval(t).^b.eval(t),max(size(a),size(b)),breaks);
    end
    
    function a = prod(x,dim)
      if (nargin<2)
        dim=find(size(x)>1,'first');
      end
      a = FunctionHandleTrajectory(@(t) prod(x.eval(t),dim), [x.dim(1:dim-1),x.dim(dim+1:end)],x.getBreaks);
    end
    
    function c = times(a,b)
      [a,b,breaks]=setupTrajectoryPair(a,b);
      c = FunctionHandleTrajectory(@(t) a.eval(t).*b.eval(t),size(a),breaks);
    end
    
    function c = mtimes(a,b)
      if (ndims(a)>2 || ndims(b)>2) error('only defined for two-d matrices'); end
      if (size(a,2) ~= size(b,1)) error('dimension mismatch'); end

      [a,b,breaks]=setupTrajectoryPair(a,b);
      c = FunctionHandleTrajectory(@(t) a.eval(t)*b.eval(t),[size(a,1),size(b,2)],breaks);
    end
    
    function c = plus(a,b)
      if (ndims(a) ~= ndims(b)) error('dimension mismatch'); end
      if (~isscalar(a) && ~isscalar(b) && any(size(a)~=size(b))) error('dimension mismatch'); end
      
      [a,b,breaks]=setupTrajectoryPair(a,b);
      c = FunctionHandleTrajectory(@(t) a.eval(t)+b.eval(t),max(size(a),size(b)),breaks);
    end
    
    function a = subsasgn(a,s,b)
      [a,b,breaks]=setupTrajectoryPair(a,b);      
      a = FunctionHandleTrajectory(@(t) subsasgn(a.eval(t),s,b.eval(t)),size(subsasgn(a.eval(breaks(1)),s,b.eval(breaks(1)))),breaks);
    end

    function n=numel(varargin)
      % have to set numel to 1, because calls to subsref 
      % with . and {} automatically ask for numel outputs.
      % note that I *hate* that numel ~= prod(size(obj))
      n=1;
    end
    
    function varargout = subsref(a,s)
      if (length(s)==1 && strcmp(s(1).type,'()'))
        breaks = a.getBreaks();
        varargout=cell(1,nargout);
        [varargout{:}] = FunctionHandleTrajectory(@(t) subsref(a.eval(t),s),size(subsref(a.eval(breaks(1)),s)),breaks);
      elseif nargout>0  % use builtin
        varargout=cell(1,nargout);
        [varargout{:}] = builtin('subsref',a,s);
      end
    end
    
    function obj = shiftTime(obj,offset)
      error('not implemented yet');
    end
    
    function s = size(obj,dim)
      s=obj.dim;
      if (length(s)==1) s=[s,1]; end
      if (nargin>1) s=s(dim); end
    end
    
    function l = length(obj)
      s = size(obj);
      l = max(s);
    end
    
%    function n = numel(obj)
%      n = prod(size(obj));
%    end
    
    function ts = getTimeSpan(obj)
      ts = obj.tspan;
    end
    
    function traj = subTrajectory(obj,ind)
      error('not implemented yet.  use subsref?');
    end
    
    function h=fnplt(obj,plotdims)
      if (nargin>1 && ~isempty(plotdims) && any(plotdims>obj.dim | plotdims<1)) error('plotdims out of range'); end
      breaks=obj.getBreaks();
      m=5; t=linspace(0,1,m)'; n=length(breaks)-1;
      ts = repmat(1-t,1,n).*repmat(breaks(1:end-1),m,1) + repmat(t,1,n).*repmat(breaks(2:end),m,1);
      ts = ts(:);
      pts = obj.eval(ts);
      if (prod(obj.dim)==1)
        h=plot(ts,squeeze(pts),'b.-','LineWidth',1,'MarkerSize',5);
      elseif (nargin>1 && ~isempty(plotdims) && length(plotdims)==1)
        h=plot(ts,squeeze(pts(plotdims,:)),'b.-','LineWidth',1,'MarkerSize',5);
      else
        if (nargin<2 || isempty(plotdims)) plotdims=[1,2]; end
        h=plot(pts(plotdims(1),:),pts(plotdims(2),:),'b-',pts(plotdims(1),[1:m:end,end]),pts(plotdims(2),[1:m:end,end]),'b.','LineWidth',1);% ,'MarkerSize',5);
      end
    end
    
    function [xnear,tnear,dxnear,dtnear] = closestPoint(obj,x,knot_ind)
      % returns the closest point on the trajectory to the sample point x
      %
      % @param x the sample point
      % @param knot_ind (optional) when available, this indicates the index
      % of the closest knot point.  
      % 
      % @retval xnear the closest point on the trajectory
      % @retval tnear the time on the trajectory associated with xnear
      
      t=obj.getBreaks();
      if (nargin<3)
        xt = obj.eval(t);
        xbar = x(:,ones(1,length(t))) - xt;
        d = sum(xbar.^2,1);
        [dmin,knot_ind] = min(d);
      else
        xt = obj.eval(t(knot_ind));
        xbar = x-xt;
        dmin = sum(xbar.^2,1);
      end
      
      % for gradient debugging
%      xnear = obj.eval(t(knot_ind));
%      tnear = t(knot_ind);
%      dxnear = zeros(obj.num_y);
%      dtnear = zeros(1,obj.num_y);
%      return;
      % end debugging;
      
      
      tspan = [t(max(knot_ind-1,1)),t(min(knot_ind+1,length(t)))];
      options = optimset('fminbnd');
      options.TolX = 1e-15;
      [tnear,dmin,exitflag]=fminbnd(@(t) sum((x-obj.eval(t)).^2,1),tspan(1),tspan(end),options);
      
      if (exitflag<1) 
        warning('fminbnd exited with exitflag %d',exitflag);
      end
      xnear = obj.eval(tnear);
      
      if (nargout>2)
        xdmin = obj.deriv(tnear);
        
        if (min(abs(tnear-t([1,end])))<=1e-14) % then I'm at one of the rails (value should be larger than options.TolX above)
          dtnear = zeros(1,obj.num_y);
        else
          xddmin = obj.dderiv(tnear);
          dtnear = xdmin'/(xdmin'*xdmin + (xnear-x)'*xddmin);
        end
        dxnear = xdmin*dtnear;
      end
    end
    
    function d = distance(obj,x)
      xnear = closestPoint(obj,x);
      d = norm(xnear-x);
    end
    
    function [d,dd] = distanceSq(obj,x)
      if (nargout>1)
        [xnear,~,dxnear,~] = closestPoint(obj,x);
        d = (xnear-x)'*(xnear-x);
        dd = 2*(xnear-x)'*(dxnear - eye(obj.num_y));
      else
        xnear = closestPoint(obj,x);
        d = (xnear-x)'*(xnear-x);
      end
    end
    
    function alpha = getParameters(obj)
      error('parameters are not implemented for this type of trajectory'); 
    end
    
    function alpha = setParameters(obj)
      error('parameters are not implemented for this type of trajetory'); 
    end
    
    
  end
end
