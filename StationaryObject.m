classdef StationaryObject < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        length;
        breadth;
        xCenter;
        yCenter;
        angle;
        xCoordinates;
        yCoordinates;
        handle;
    end
    
    methods
        function update(obj,currentStep)
            
            obj.xCoordinates(currentStep,:)=[obj.xCoordinates(1,1) obj.xCoordinates(1,2) obj.xCoordinates(1,3) obj.xCoordinates(1,4) obj.xCoordinates(1,1)];
            obj.yCoordinates(currentStep,:)=[obj.yCoordinates(1,1) obj.yCoordinates(1,2) obj.yCoordinates(1,3) obj.yCoordinates(1,4) obj.yCoordinates(1,1)];
        end
    end
    
end

