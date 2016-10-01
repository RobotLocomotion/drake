classdef SoftPaddleControl < HybridDrakeSystem
    properties
    end
    
    methods
        function obj = SoftPaddleControl(plant)
            obj = obj@HybridDrakeSystem(2*plant.in_contact.num_positions,length(plant.in_contact.B));
            obj = setInputFrame(obj, getOutputFrame(plant));
            obj = setOutputFrame(obj, getInputFrame(plant));
        end
    end
end