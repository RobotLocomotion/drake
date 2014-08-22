classdef ControlSurface
  
  properties
    name;
    chord;
    span;
    left_edge_position_along_wing;
    
    
  end
  
  
  methods
    
    function obj = ControlSurface(name, chord, span, left_edge_position_along_wing)
      obj.name = name;
      obj.chord = chord;
      obj.span = span;
      obj.left_edge_position_along_wing = left_edge_position_along_wing;
      
    end
    
  end
  
  
end
    