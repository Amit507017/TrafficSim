classdef Kreuzung 
    %KREUZUNG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        XRoadPoints;
        YRoadPoints;
        mainRoadHandles;
        trackWidth;
        noRoadToTracks;
        noRoadAwayTracks;
        squareDim;
        squareAngle;
         %%the main points of the kreuzung (Right road)
       
        %%Road Points 
        XRoadToTracksPoints;
        YRoadToTracksPoints;
        XRoadAwayTracksPoints;
        YRoadAwayTracksPoints;
        toTracksHandles;
        awayTrackHandles;
        
        %points of parabola connecting roads of Kreuzung
        xCornerPoints;
        yCornerPoints;
        kreuzungCornerHandles;
        % End Points of Kreuzung roads to connect to other roads
        xEndPoints;
        yEndPoints;
        endPointsSlope;
        
        %%Track Points
        XInnerRoadToTracksPoints;
        YInnerRoadToTracksPoints;
        XInnerRoadAwayTracksPoints;
        YInnerRoadAwayTracksPoints;
        innerRoadToTracksHandles;
        innerRoadAwayTracksHandles;
        
        %% start Points of inner tracks
        xTrackStartPoint;
        yTrackStartPoint;
        
        %Kreuzung inner road points and handles
        xRoadJoiningPoints;
        yRoadJoiningPoints;
        roadJoiningHandles;
        
        % Traffic lights 
        trafficSignal;
        signalTime;
        
    end
    
    methods
    end
    
end

