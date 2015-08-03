classdef Point2DVisualizer < Visualizer
  % adds a point to an existing 2D scene
  
  properties 
    linespec
    props
  end
  
  methods
    function obj = Point2DVisualizer(frame,linespec,varargin)
      typecheck(frame,'CoordinateFrame');
      valuecheck(frame.dim,2);
      if (nargin<2) linespec = 'b*'; end
      typecheck(linespec,'char');
      if (nargin<3) varargin={}; end
      typecheck(varargin,'cell');
      
      obj = obj@Visualizer(frame);
      obj.linespec = linespec;
      obj.props = varargin;
    end
    
    function draw(obj,t,y)
      % draws directly on the current axis
      plot(y(1),y(2),obj.linespec,obj.props{:});
    end
  end
end