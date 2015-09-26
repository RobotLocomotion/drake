classdef (InferiorClasses = {?ConstantTrajectory,?FunctionHandleTrajectory,?PPTrajectory}) DTTrajectory < Trajectory
  
  properties
    tt
    xx
  end
  
  methods
    function obj = DTTrajectory(tt,xx)
      obj = obj@Trajectory(size(xx,1));
      typecheck(tt,'double');
      typecheck(xx,'double');
      if (~isrow(tt)) error('tt should be a 1xn list of times'); end
      if (size(tt,2)~=size(xx,2)) error('xx must have the same number of columns as tt'); end
      if (~issorted(tt)) error('tt should be monotonically increasing'); end

      ts = mean(diff(tt));
      if ts>(tt(end)-tt(end-1))+1e-6    % handle the case where the final time was not a multiple of the sample time
        tt = tt(1:end-1);
        xx = xx(:,1:end-1);
        ts = mean(diff(tt));
      end
      if (max(diff(tt))-ts>1e-6 || ts-min(diff(tt))>1e-6)
        error('tt doesn''t appear to have a fixed sample time'); 
      end
      obj = obj.setSampleTime([ts;0]);
      % touch up times to align perfectly with sample times:
      tt = round(tt/ts)*ts;
      
      obj.tt = tt;
      obj.xx = xx;
      obj.tspan = [min(tt) max(tt)];
    end
    function y = eval(obj,t)  
      ind = find(abs(obj.tt-t)<1e-10);  % only return on exact matches.  For interpolation, you should be using zoh or foh to make PPTrajectories
      y = obj.xx(:,ind);
    end
        
    function t = getBreaks(obj)
      t = obj.tt;
    end
    
    function h=fnplt(obj,plotdims)
      if (prod(obj.dim)==1)
        h=stem(obj.tt,obj.xx,'b');
      else
        if (nargin<2) plotdims=[1,2]; end
        if (length(plotdims)==1)
          h=stem(obj.tt,obj.xx(plotdims,:),'b','MarkerSize',5);
        else
          h=plot(obj.xx(plotdims(1),:),obj.xx(plotdims(2),:),'b.','MarkerSize',5);
        end
      end
    end
    
    function obj = vertcat(varargin)
      % find the first DTTrajectory
      ind = find(cellfun(@(a)isa(a,'DTTrajectory'),varargin),1);
      tt = varargin{ind}.tt;
      xx=[];
      
      fr = {};
      for i=1:length(varargin)
        if (isa(varargin{i},'DTTrajectory'))
          valuecheck(tt,varargin{i}.tt);  % error if they are not at the same sample times (up to numerical errors)
          xx = vertcat(xx,varargin{i}.xx);
        else
          xx = vertcat(xx,eval(varargin{i},tt));
        end
        fr = {fr{:},getOutputFrame(varargin{i})};
      end
      obj = DTTrajectory(tt,xx);
      obj = setOutputFrame(obj,MultiCoordinateFrame(fr));
    end
    
    function obj = horzcat(varargin)
      % find the first DTTrajectory
      ind = find(cellfun(@(a)isa(a,'DTTrajectory'),varargin),1);
      tt = varargin{ind}.tt;
      xx=[];
      
      fr ={};
      for i=1:length(varargin)
        if (isa(varargin{i},'DTTrajectory'))
          valuecheck(tt,varargin{i}.tt);  % error if they are not at the same sample times (up to numerical errors)
          xx = horzcat(xx,varargin{i}.xx);
        else
          xx = horzcat(xx,eval(varargin{i},tt));
        end
        fr = {fr{:},getOutputFrame(varargin{i})};
      end
      obj = DTTrajectory(tt,xx);
      obj = setOutputFrame(obj,MultiCoordinateFrame(fr));
    end

    function c = mtimes(a,b)
      if any([size(a,1) size(b,2)]==0)  % handle the empty case
        c = ConstantTrajectory(zeros(size(a,1),size(b,2)));
        return;
      end
      if isa(a,'ConstantTrajectory') a=double(a); end
      if isa(b,'ConstantTrajectory') b=double(b); end

      if isnumeric(a)  % then only b is a DTTrajectory
        tt = b.tt;
        xx = a*b.xx;  % ok since last dimension of xx is the time dim...
        c = DTTrajectory(tt,xx);
%      elseif isnumeric(b) % then only a is a DTTrajectory
%        c = a;
%        c.xx = a.xx*b;
      else
        error('not implemented (yet)');
      end
        
    end    
    
    function c = plus(a,b)
      if ~isequal(size(a),size(b))
        error('must be the same size');  % should support scalars, too (but don't yet)
      end
      if any(size(a)==0)  % handle the empty case
        c = ConstantTrajectory(zeros(size(a)));
        return;
      end
      if isa(a,'ConstantTrajectory') a=double(a); end
      if isa(b,'ConstantTrajectory') b=double(b); end
      
      if isnumeric(a)  % then only b is a PPTrajectory
        tt = b.tt;
        xx = bsxfun(@plus,a,b.xx);
        c = DTTrajectory(tt,xx);
      elseif isnumeric(b)
        tt = a.tt;
        xx = bsxfun(@plus,a.xx,b);
        c = DTTrajectory(tt,xx);
      else
        error('not implemented (yet)');
      end
    end
  end
end
