classdef Obstacle
    % Class that describes an obstacle
    % supports
    %   drawing
    %   getting constraints
    
    properties (SetAccess=protected, GetAccess=protected)
        x = 0;
        y = 0;
        z = 0;
        
        func;
        
    end
    
    
    methods (Abstract = true)
        con = getConstraints(obj)
        %move(obj, displacement)
        draw(obj)
    end
    
    methods
        % constructor
        function obj = Obstacle()
            
        end
    end
    
    
end