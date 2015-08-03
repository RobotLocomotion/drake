classdef MultiVisualizer < Visualizer
  % stack a bunch of visualizers together so that they draw at the same
  % time 
  
  properties 
    viz;  % a cell array of visualizers
  end
  
  methods
    function obj = MultiVisualizer(viz)
      if isa(viz,'Visualizer')
        viz = {viz};
      end
      typecheck(viz,'cell'); 
      viz = {viz{:}}';  % make it a column vector
      
      obj = obj@Visualizer(MultiCoordinateFrame(cellfun(@(a) a.getInputFrame,viz,'UniformOutput',false)));
      obj.viz = viz;
    end
    
    function obj = addVisualizer(viz)
      typecheck(viz,'Visualizer');
      obj.viz = {obj.viz{:},viz};
      obj = setInputFrame(MultiCoordinateFrame(cellfun(@(a) a.getInputFrame,viz,'UniformOutput',false)));
    end
    
    function draw(obj,t,y)
      y = splitCoordinates(obj.getInputFrame,y);
      for i=1:length(obj.viz)
        obj.viz{i}.draw(t,y{i});
      end
    end
      
  end
end