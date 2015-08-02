function hOut = axisAnnotation(varargin)

% Implements the annotation command from matlab, but does everything in the
% coordinates of the current axis.
%
% Syntax:
%   axisAnnotation( annotation_type)
%   axisAnnotation('line',x,y)
%   axisAnnotation('arrow',x,y)
%   axisAnnotation('doublearrow',x,y)
%   axisAnnotation('textarrow',x,y)
%   axisAnnotation('textbox',[x y w h])
%   axisAnnotation('ellipse',[x y w h])
%   axisAnnotation('rectangle',[x y w h])
%   axisAnnotation(axis_handle,...)
%   axisAnnotation(...,'PropertyName',PropertyValue,...)
%   anno_obj_handle = axisAnnotation(...)
%
% See documentation for the matlab annotation command for more
% details.  


% get the list of axes to operate upon
if ~isempty(varargin) && all(ishghandle(varargin{1})) && ~isempty(findobj(varargin{1},'type','axes','-depth',0))
    ax = varargin{1};
    varargin=varargin(2:end);
else
    ax = gca;
end

if (~ischar(varargin{1}))
  error('the first argument here should be the annotation type');
end

% transform to figure coordinate
switch varargin{1}
  case {'line','arrow','doublearrow','textarrow'}
    x=varargin{2};
    y=varargin{3};
    hOut=annotation(varargin{1},0*x,0*y,varargin{4:end});
    set(hOut,'parent',gca,'X',x,'Y',y);
  case {'textbox','ellipse','rectangle'}
    dim=varargin{2};
    hOut=annotation(varargin{1},0*dim,varargin{3:end});
    set(hOut,'parent',gca,'position',dim);
  otherwise
    error('don''t know this annotation type');
end



