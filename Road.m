classdef Road
    %ROAD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        widthTrack;
        %central line 
        mainHandle;
         mainXRoadPoints;
        mainYRoadPoints;
        
        % road lines
        innerRoadHandles;
        outerRoadHandles;
        xOuterCurvePoints = [];
        yOuterCurvePoints = [];
        xInnerCurvePoints = [];
        yInnerCurvePoints = [];
        
        % track lines
        innerTrackHandles;
        outerTrackHandles;
        xOuterCurveTrackPoints = [];
        yOuterCurveTrackPoints = [];
        xInnerCurveTrackPoints = [];
        yInnerCurveTrackPoints = [];
       
        % 
        xInnerEndPoints;
        yInnerEndPoints;
        xOuterEndPoints;
        yOuterEndPoints;
      
        % rnd points of central line
        xEndPoints;
        yEndPoints;
        endPointsSlope;
    end
    
    methods
    end
    
end

