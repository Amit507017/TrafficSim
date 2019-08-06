function simulateScenario(count)
global vehicleDatabase;
global pedestrianDatabase;
global bicycleDatabase;
global roadDatabase;
global kreuzungDatabase;
global objectDatabase;


size_vehicleDatabase = size(vehicleDatabase);
size_pedestrianDatabase = size(pedestrianDatabase);
size_bicycleDatabase = size(bicycleDatabase);
size_roadDatabase = size(roadDatabase);
size_kreuzungDatabase = size(kreuzungDatabase);
size_objectDatabase=size(objectDatabase);
% for j=1:size_vehicleDatabase(2)
%     if strcmp(vehicleDatabase(1,j).type,'EGO')
%         set(vehicleDatabase(1,j).handle,'Facecolor','k');
%     else
%         set(vehicleDatabase(1,j).handle,'Facecolor','w');
%     end
% end
% 
% for j=1:size_pedestrianDatabase(2)
%     set(pedestrianDatabase(1,j).handle,'Facecolor','k');
% end
% 
% for j=1:size_bicycleDatabase(2)
%     set(bicycleDatabase(1,j).handle,'Facecolor','m');
% end

% set(gcf,'PaperPositionMode','Manual','PaperPosition',[0,0,40,20])
% print(gcf,'-dpdf','1.pdf');

% hold on;
% set(gcf,'Pointer','crosshair','doublebuffer','on');
% %[new_xRoadPoints new_yRoadPoints] = curvefit();
% [x,y] = ginput(2);
%
% [new_xRoadPoints new_yRoadPoints slope_endpt] = cubicSpline(x(1),y(1),x(2),y(2),0.02,0.2);
% plot (new_xRoadPoints,new_yRoadPoints);
Time =0;
timeStep = 0.02;
simulationTime =3;
noSteps = simulationTime/timeStep;
EGOVehicleDrive =0;
pathFound=0;

global collisionTime;
if isempty(collisionTime)
    collisionTime = simulationTime;
end
global isCollisionDetected;

if collisionTime < simulationTime
    isCollisionDetected=1;
else
    isCollisionDetected=0;
end

for i=1:size_vehicleDatabase(2)
    if strcmp(vehicleDatabase(1,i).type,'EGO')
        
        EGOVehicle = i;
        break;
        
    end
end



for step=1:noSteps
    
    
    Time = Time + timeStep;
    %set(handles.currentTime,'String',Time);
    predictionTime = 4;%simulationTime;
    noStepPrediction=predictionTime/timeStep;
    stopPrediction=0;
    breakEGO=0;
    parallelTime = Time;
    collisionObject=0;
    
    if exist('treeDatabase','var') && ~isempty (treeDatabase)
        treeDatabase(1,1).removePastNodes();
    else
        treeDatabase = [];
    end
    
    if step==1
    for predictionStep = 1:noStepPrediction
        parallelTime = parallelTime + timeStep;
        currentPredictionStep = step + predictionStep;
        
        
        %% Bicycle collision prediction
        for i=1:size_bicycleDatabase(2)
            bicycleDatabase(1,i).driveOneTrack(bicycleDatabase(1,i).xTravelPoints,bicycleDatabase(1,i).yTravelPoints,timeStep,currentPredictionStep);
            %     for j=1:size_vehicleDatabase(2)
            %         collBicVeh = ispolygon(bicycleDatabase(1,i).xCoordinates,bicycleDatabase(1,i).yCoordinates,vehicleDatabase(1,j).xCoordinates,vehicleDatabase(1,j).yCoordinates);
            %         len = length(collBicVeh);
            %         for k=1:len
            %             if collBicVeh == 1
            %                 set(handles.messageBox,'String','Collision Detected');
            %             end
            %         end
            %     end
            %
            %     for j=1:size_bicycleDatabase(2)
            %         if i~=j
            %             collBicVeh = ispolygon(bicycleDatabase(1,i).xCoordinates,bicycleDatabase(1,i).yCoordinates,bicycleDatabase(1,j).xCoordinates,bicycleDatabase(1,j).yCoordinates);
            %             len = length(collBicVeh);
            %             for k=1:len
            %                 if collBicVeh == 1
            %                     set(handles.messageBox,'String','Collision Detected');
            %                 end
            %             end
            %         end
            %     end
        end
        
        
        %% Pedestrain collision prediction
        for i=1:size_pedestrianDatabase(2)
            
            pedestrianDatabase(1,i).move(pedestrianDatabase(1,i).xTravelPoints,pedestrianDatabase(1,i).yTravelPoints,timeStep,currentPredictionStep);
            
            if sqrt((pedestrianDatabase(1,i).xPos(currentPredictionStep)-vehicleDatabase(1,EGOVehicle).xCG(currentPredictionStep-1))^2 + ...
                    (pedestrianDatabase(1,i).yPos(currentPredictionStep)-vehicleDatabase(1,EGOVehicle).yCG(currentPredictionStep-1))^2) ...
                    < (vehicleDatabase(1,EGOVehicle).lf+vehicleDatabase(1,EGOVehicle).lr+vehicleDatabase(1,EGOVehicle).lfToFront+vehicleDatabase(1,EGOVehicle).lrToRear)/2
                collPedVeh = inpolygon(pedestrianDatabase(1,i).xCoordinates(currentPredictionStep,:),pedestrianDatabase(1,i).yCoordinates(currentPredictionStep,:),vehicleDatabase(1,EGOVehicle).xCoordinates(currentPredictionStep-1,:),vehicleDatabase(1,EGOVehicle).yCoordinates(currentPredictionStep-1,:));
                len = length(collPedVeh);
                for k=1:len
                    if collPedVeh(k) == 1
                        isCollisionDetected =1;
                        stopPrediction=1;
                        if parallelTime < collisionTime
                            collisionTime=parallelTime;
                        end
%                        set(handles.messageBox,'String','Collision Detected');
                    end
                end
                
            end
        end
        
        %% Vehicle collision detection
        for i=1:size_vehicleDatabase(2)
            if pathFound==1
                predWithConstSpeed=0;
            else
                predWithConstSpeed=1;
            end
            if strcmp(vehicleDatabase(1,i).type,'EGO')
                vehicleDatabase(1,i).drive(vehicleDatabase(1,i).xTravelPoints,vehicleDatabase(1,i).yTravelPoints,timeStep,parallelTime,(currentPredictionStep));
                EGOVehicle = i;
            else
                vehicleDatabase(1,i).drive(vehicleDatabase(1,i).xTravelPoints,vehicleDatabase(1,i).yTravelPoints,timeStep,parallelTime,(currentPredictionStep));
               % vehicleDatabase(1,i).driveOneTrack(vehicleDatabase(1,i).xTravelPoints,vehicleDatabase(1,i).yTravelPoints,timeStep,(currentPredictionStep));
            end
            
            % Bicycle and vehicle
            if~isempty(bicycleDatabase)
                for j=1:size_bicycleDatabase(2)
                    collBicVeh = inpolygon(vehicleDatabase(1,i).xCoordinates(currentPredictionStep,:),vehicleDatabase(1,i).yCoordinates(currentPredictionStep,:),bicycleDatabase(1,j).xCoordinates(currentPredictionStep,:),bicycleDatabase(1,j).yCoordinates(currentPredictionStep,:));
                    len = length(collBicVeh);
                    for k=1:len
                        if collBicVeh(k) == 1
                            isCollisionDetected =1;
                            %    stoPrediction=1;
                            if parallelTime < collisionTime
                                collisionTime=parallelTime;
                            end
            %                set(handles.messageBox,'String','Collision Detected');
                        end
                    end
                end
            end
            
            % Pedestrian and vehicle
            if ~isempty(pedestrianDatabase)
                for j=1:size_pedestrianDatabase(2)
                    if sqrt((pedestrianDatabase(1,j).xPos(currentPredictionStep)-vehicleDatabase(1,EGOVehicle).xCG(currentPredictionStep-1))^2 + ...
                            (pedestrianDatabase(1,j).yPos(currentPredictionStep)-vehicleDatabase(1,EGOVehicle).yCG(currentPredictionStep-1))^2) ...
                            < (vehicleDatabase(1,EGOVehicle).lf+vehicleDatabase(1,EGOVehicle).lr+vehicleDatabase(1,EGOVehicle).lfToFront+vehicleDatabase(1,EGOVehicle).lrToRear)/2
                        collVehPed = inpolygon(vehicleDatabase(1,EGOVehicle).xCoordinates(currentPredictionStep-1,:),vehicleDatabase(1,EGOVehicle).yCoordinates(currentPredictionStep-1,:),...
                            pedestrianDatabase(1,j).xCoordinates(currentPredictionStep,:),pedestrianDatabase(1,j).yCoordinates(currentPredictionStep,:));
                        len = length(collVehPed);
                        for k=1:len
                            if collVehPed(k) == 1
                                isCollisionDetected =1;
                                stopPrediction=1;
                                if parallelTime < collisionTime
                                    collisionTime=parallelTime;
                                end
 %                               set(handles.messageBox,'String','Collision Detected');
                            end
                        end
                    end
                    
                end
            end
            
            % vehicle with other vehicle
            for j=1:size_vehicleDatabase(2)
                % checking collision detection for only EGO vehicle
                if i~=j && strcmp(vehicleDatabase(1,i).type,'EGO')
                    EGOVehicleLength = (vehicleDatabase(1,EGOVehicle).lf+vehicleDatabase(1,EGOVehicle).lr+vehicleDatabase(1,EGOVehicle).lfToFront+vehicleDatabase(1,EGOVehicle).lrToRear)/2000;
                    otherVehicleLength =  (vehicleDatabase(1,j).lf+vehicleDatabase(1,j).lr+vehicleDatabase(1,j).lfToFront+vehicleDatabase(1,j).lrToRear)/2;
                    if sqrt((vehicleDatabase(1,EGOVehicle).xCG(currentPredictionStep-1) - vehicleDatabase(1,j).xCG(currentPredictionStep-1))^2 + ...
                            (vehicleDatabase(1,EGOVehicle).yCG(currentPredictionStep-1) - vehicleDatabase(1,j).yCG(currentPredictionStep-1))^2) < (EGOVehicleLength+otherVehicleLength)
                        collBicVeh = inpolygon(vehicleDatabase(1,i).xCoordinates(currentPredictionStep-1,:),vehicleDatabase(1,i).yCoordinates(currentPredictionStep-1,:),vehicleDatabase(1,j).xCoordinates(currentPredictionStep-1,:),vehicleDatabase(1,j).yCoordinates(currentPredictionStep-1,:));
                        len = length(collBicVeh);
                        for k=1:len
                            if collBicVeh(k) == 1
                                isCollisionDetected =1;
                                collisionObject=j;
                                stopPrediction=1;
                                if parallelTime < collisionTime
                                    collisionTime = parallelTime;
                                end
%                                set(handles.messageBox,'String','Collision Detected');
                            end
                        end
                    end
                end
            end
            
            % vehicle with stationary object
            for j=1:size_objectDatabase(2)
                objectDatabase(1,j).update(currentPredictionStep);
                EGOVehicleLength = (vehicleDatabase(1,EGOVehicle).lf+vehicleDatabase(1,EGOVehicle).lr+vehicleDatabase(1,EGOVehicle).lfToFront+vehicleDatabase(1,EGOVehicle).lrToRear)/2000;
                
                distVehObj=sqrt((vehicleDatabase(1,EGOVehicle).xCG(currentPredictionStep-1) - objectDatabase(1,j).xCenter)^2 + ...
                    (vehicleDatabase(1,EGOVehicle).yCG(currentPredictionStep-1) - objectDatabase(1,j).yCenter)^2);
                if  (distVehObj < (EGOVehicleLength+objectDatabase(1,j).length))|| (distVehObj < (EGOVehicleLength+objectDatabase(1,j).breadth))
                    collVehObj=inpolygon(vehicleDatabase(1,EGOVehicle).xCoordinates(currentPredictionStep-1,:),vehicleDatabase(1,EGOVehicle).yCoordinates(currentPredictionStep-1,:),...
                        objectDatabase(1,j).xCoordinates(currentPredictionStep-1,:),objectDatabase(1,j).yCoordinates(currentPredictionStep-1,:));
                    len = length(collVehObj);
                    for k=1:len
                        if collVehObj(k) == 1
                            isCollisionDetected =1;
                            stopPrediction=1;
                            if parallelTime < collisionTime
                                collisionTime = parallelTime;
                            end
             %               set(handles.messageBox,'String','Collision Detected');
                        end
                    end
                end
            end
            
        end
        if stopPrediction ==1
            %  break;
        end
        
    end
    end
    
    
    % Prediction algorithm end
    
    
    
    %% Collision Avoidance just by breaking
    if isCollisionDetected==1
        stoppingDistance=(vehicleDatabase(1,EGOVehicle).speed(step)^2)/(2*9.81*1); %%%Braking distance wikipedia
        distanceToCollision=(collisionTime-Time) * vehicleDatabase(1,EGOVehicle).speed(step);
        breakEGO=0;
        distMin=1;
        
        if distanceToCollision > (stoppingDistance+5) && collisionObject~=0
            if (abs(vehicleDatabase(1,EGOVehicle).angleDir(step)-vehicleDatabase(1,collisionObject).angleDir(step)))<(pi/4)
                breakEGO=1;
                isCollisionDetected=0;
            end
            %%%collision can be avoided just by braking, check if any other object
            %%%is following
        end
        
        if breakEGO==1
            [~,idxEGO]=min((((vehicleDatabase(1,EGOVehicle).xCG(step) - vehicleDatabase(1,EGOVehicle).xTravelPoints).^2) +...
                (vehicleDatabase(1,EGOVehicle).yCG(step) - vehicleDatabase(1,EGOVehicle).yTravelPoints).^2));
            
            for j=1:size_vehicleDatabase(2)
                if j~=EGOVehicle
                    if vehicleDatabase(1,j).trackHandles==vehicleDatabase(1,EGOVehicle).trackHandles
                        dist=((vehicleDatabase(1,j).xCG(step) - vehicleDatabase(1,EGOVehicle).xTravelPoints).^2 + ...
                            (vehicleDatabase(1,j).yCG(step) - vehicleDatabase(1,EGOVehicle).yTravelPoints).^2);
                        [distOther,idxOther]=min(dist);
                        if distOther < 0.5 && idxOther < idxEGO
                            distEGOOther=(idxEGO-idxOther)*0.2;
                            stoppingDistance=((vehicleDatabase(1,j).speed(step))^2)/(2*9.81*0.8);
                            if distEGOOther < stoppingDistance
                                breakEGO=0;
                                isCollisionDetected=1;
                            end
                        end
                        
                    end
                end
            end
        end
        
    end
    
    
    %% Collision avoidance using RRT
    
    if (isCollisionDetected ==1) && ((collisionTime-Time)<2)
       
       collisionTimeStep=Time;
    
        hold on;
        % the start node
        qInit= Node;
        qInit.xNode = vehicleDatabase(1,EGOVehicle).xCG(step);
        qInit.yNode = vehicleDatabase(1,EGOVehicle).yCG(step);
        qInit.nodeIndex = 1;%'1.1';
        qInit.cost = 0;
        qInit.slope = vehicleDatabase(1,EGOVehicle).angleDir(step);
        qInit.time = Time;
        qInit.delta = vehicleDatabase(1,EGOVehicle).delta_fl(step);
        qInit.speed = vehicleDatabase(1,EGOVehicle).speed(step);
        qInit.ax =vehicleDatabase(1,EGOVehicle).longitudinalAccn(step);
        if qInit.ax<0
            qInit.axType=2;
        elseif qInit.ax>0
            qInit.axType=1;
        else
            qInit.axType=4;
        end
        % the goal node
        distanceToGoal = vehicleDatabase(1,EGOVehicle).speed(step) * 4;%(collisionTime+1-Time);
        idxGoal = floor(distanceToGoal/0.2);
               
        unchanged_xTravelPoints=[] ;
        unchanged_yTravelPoints=[] ;
        %check the position by considering all the roads, necessary because
        %while replanning idx changes as path is different
        for m=1:size_roadDatabase(2)
            size_innerTracks = size(roadDatabase(1,m).innerTrackHandles);
            for innerTracks=1:size_innerTracks(2)
                distToKlot=sqrt(((roadDatabase(1,m).xInnerCurveTrackPoints(innerTracks,:)-vehicleDatabase(1,EGOVehicle).xCG(step)).^2+...
                    (roadDatabase(1,m).yInnerCurveTrackPoints(innerTracks,:)-vehicleDatabase(1,EGOVehicle).yCG(step)).^2));
                [minVal, idxTemp]=min(distToKlot);
                if minVal<2
                    idx=idxTemp;
                    
                    unchanged_xTravelPoints = roadDatabase(1,m).xInnerCurveTrackPoints(innerTracks,(idx+idxGoal):end);
                    unchanged_yTravelPoints = roadDatabase(1,m).yInnerCurveTrackPoints(innerTracks,(idx+idxGoal):end);
                    break;
                end
            end
            
            size_outerTracks = size(roadDatabase(1,m).outerTrackHandles);
            for outerTracks=1:size_outerTracks(2)
                distToKlot=sqrt(((roadDatabase(1,m).xOuterCurveTrackPoints(outerTracks,:)-vehicleDatabase(1,EGOVehicle).xCG(step)).^2+...
                    (roadDatabase(1,m).yOuterCurveTrackPoints(outerTracks,:)-vehicleDatabase(1,EGOVehicle).yCG(step)).^2));
                [minVal, idxTemp]=min(distToKlot);
                if minVal<2
                    idx=idxTemp;
                    
                    unchanged_xTravelPoints = roadDatabase(1,m).xOuterCurveTrackPoints(outerTracks,(idx+idxGoal):end);
                    unchanged_yTravelPoints = roadDatabase(1,m).yOuterCurveTrackPoints(outerTracks,(idx+idxGoal):end);
                    break;
                end
            end
        end
        
      % checking if number of point for idxGoal does not exceed the road
        % end point, which will give an error
        distCG=(((vehicleDatabase(1,EGOVehicle).xTravelPoints - vehicleDatabase(1,EGOVehicle).xCG(step)).^2) + (vehicleDatabase(1,EGOVehicle).yTravelPoints - vehicleDatabase(1,EGOVehicle).yCG(step)).^2);
            [~,idx]=min(distCG);
        size_travelPoints=size(roadDatabase(1,1).mainXRoadPoints);
        if (idx+idxGoal)>size_travelPoints(1)
            idxGoal=size_travelPoints(1)-idx;
        end
        
        new_xRoadPoints=[];
        new_yRoadPoints=[];
        xCenterCollision=[];
        yCenterCollision=[];
        xCenterKreuzung=[];
        yCenterKreuzung=[];
        radiusKreuzung=[];
        
        isInnerTrack=0;
        % points of the other tracks for sampling
        size_trackHandles = size(vehicleDatabase(1,EGOVehicle).trackHandles);
        for track=1:size_trackHandles(2)
            for road=1:size_roadDatabase(2)
                size_innerTracks = size(roadDatabase(1,road).innerTrackHandles);
                for innerTracks=1:size_innerTracks(2)
                    if roadDatabase(1,road).innerTrackHandles(innerTracks)== vehicleDatabase(1,EGOVehicle).trackHandles(track)
                        currentRoad=road;
                        isInnerTrack=1;
%                         for innerTrack=1:size_innerTracks(2)
%                             if roadDatabase(1,road).innerTrackHandles(innerTrack)~= vehicleDatabase(1,EGOVehicle).trackHandles(track)
%                                 innerTrackXPoints=roadDatabase(1,road).xInnerCurveTrackPoints(innerTrack,:);
%                                 new_xRoadPoints = [new_xRoadPoints innerTrackXPoints((idx+30):(idx+idxGoal))];
%                                 innerTrackYPoints=roadDatabase(1,road).yInnerCurveTrackPoints(innerTrack,:);
%                                 new_yRoadPoints = [new_yRoadPoints innerTrackYPoints((idx+30):(idx+idxGoal))];
%                             end
%                         end
                    end
                end
                
                size_outerTracks = size(roadDatabase(1,road).outerTrackHandles);
                for outerTracks=1:size_outerTracks(2)
                    if roadDatabase(1,road).outerTrackHandles(outerTracks)== vehicleDatabase(1,EGOVehicle).trackHandles(track)
                        currentRoad=road;
                        isInnerTrack=0;
%                         for outerTrack=1:size_outerTracks(2)
%                             if roadDatabase(1,road).outerTrackHandles(outerTrack)~= vehicleDatabase(1,EGOVehicle).trackHandles(track)
%                                 outerTrackXPoints=roadDatabase(1,road).xOuterCurveTrackPoints(outerTrack,:);
%                                new_xRoadPoints = [new_xRoadPoints outerTrackXPoints((idx+30):(idx+idxGoal))];
%                                 outerTrackYPoints=roadDatabase(1,road).yOuterCurveTrackPoints(outerTrack,:);
%                               new_yRoadPoints = [new_yRoadPoints outerTrackYPoints((idx+30):(idx+idxGoal))];
%                             end
%                         end
                    end
                end
            end
            
            if ~exist('CurrentRoad','var')
                currentRoad=1;
            end
            
            %sampling uniformly over the width of the road
            if isInnerTrack==1
                noOfTracks=length(roadDatabase(1,currentRoad).innerTrackHandles);
                if noOfTracks==1 % when there is only one track sample on both lanes irrespective of direction of travel on that lane
                    new_xRoadPoints=roadDatabase(1,currentRoad).mainXRoadPoints((idx+30):(idx+idxGoal));
                    new_yRoadPoints=roadDatabase(1,currentRoad).mainYRoadPoints((idx+30):(idx+idxGoal));
                    widthRoad=roadDatabase(1,currentRoad).widthTrack*2;
                else
                    widthRoad=roadDatabase(1,currentRoad).widthTrack*noOfTracks/2;
                    if mod(noOfTracks,2)==0
                        laneNumber=noOfTracks/2;
                        new_xRoadPoints=roadDatabase(1,currentRoad).xInnerCurvePoints(laneNumber,(idx+30):(idx+idxGoal));
                        new_yRoadPoints=roadDatabase(1,currentRoad).yInnerCurvePoints(laneNumber,(idx+30):(idx+idxGoal));
                    else
                        trackNumber=floor(noOfTracks/2)+1;
                        new_xRoadPoints=roadDatabase(1,currentRoad).xInnerCurveTrackPoints(trackNumber,(idx+30):(idx+idxGoal));
                        new_yRoadPoints=roadDatabase(1,currentRoad).yInnerCurveTrackPoints(trackNumber,(idx+30):(idx+idxGoal));
                    end
                end
            else
                noOfTracks=length(roadDatabase(1,currentRoad).outerTrackHandles);
                if noOfTracks==1   % when there is only one track sample on both lanes irrespective of direction of travel on that lane
                    new_xRoadPoints=roadDatabase(1,currentRoad).mainXRoadPoints((idx+30):(idx+idxGoal));
                    new_yRoadPoints=roadDatabase(1,currentRoad).mainYRoadPoints((idx+30):(idx+idxGoal));
                    widthRoad=roadDatabase(1,currentRoad).widthTrack;
                else
                    widthRoad=roadDatabase(1,currentRoad).widthTrack*noOfTracks/2;
                    if mod(noOfTracks,2)==0
                        laneNumber=noOfTracks/2;
                        new_xRoadPoints=roadDatabase(1,currentRoad).xOuterCurvePoints(laneNumber,(idx+30):(idx+idxGoal));
                        new_yRoadPoints=roadDatabase(1,currentRoad).yOuterCurvePoints(laneNumber,(idx+30):(idx+idxGoal));
                    else
                        trackNumber=floor(noOfTracks/2)+1;
                        new_xRoadPoints=roadDatabase(1,currentRoad).xOuterCurveTrackPoints(trackNumber,(idx+30):(idx+idxGoal));
                        new_yRoadPoints=roadDatabase(1,currentRoad).yOuterCurveTrackPoints(trackNumber,(idx+30):(idx+idxGoal));
                    end
                end
            end
        end
            
            % sampling points for kreuzung
            for kreuzung=1:size_kreuzungDatabase(2)
                size_roadJoiningHandles = size(kreuzungDatabase(1,kreuzung).roadJoiningHandles);
                for joiningRoad=1:size_roadJoiningHandles(2)
                    if kreuzungDatabase(1,kreuzung).roadJoiningHandles(joiningRoad)== vehicleDatabase(1,EGOVehicle).trackHandles(track)
%                         xCenterCollision = vehicleDatabase(1,EGOVehicle).xTravelPoints(idx+floor((vehicleDatabase(1,EGOVehicle).speed(step+1)*collisionTime)/0.2));
%                         yCenterCollision = vehicleDatabase(1,EGOVehicle).yTravelPoints(idx+floor((vehicleDatabase(1,EGOVehicle).speed(step+1)*collisionTime)/0.2));
                        xCenterKreuzung= kreuzungDatabase(1,kreuzung).XRoadPoints(1,1)- kreuzungDatabase(1,kreuzung).squareDim*(cos(kreuzungDatabase(1,kreuzung).squareAngle(1,1)));
                        yCenterKreuzung= kreuzungDatabase(1,kreuzung).YRoadPoints(1,1)- kreuzungDatabase(1,kreuzung).squareDim*(sin(kreuzungDatabase(1,kreuzung).squareAngle(1,1)));
                        radiusKreuzung=kreuzungDatabase(1,kreuzung).squareDim-3;
                        
                    end
                end
            end
        end
        
        
        
        
       
    
    for i=1:size_pedestrianDatabase(2)
        %pedestrianDatabase(1,i).move(pedestrianDatabase(1,i).xTravelPoints,pedestrianDatabase(1,i).yTravelPoints,timeStep);
        %pedestrianDatabase(1,i).updatePedestrianPosition(step+1);
    end
    
    
    for i=1:size_vehicleDatabase(2)
        if strcmp(vehicleDatabase(1,i).type,'EGO')
            if EGOVehicleDrive==1      % because prediction is not same, so vehicle model is run again
                vehicleDatabase(1,i).drive(vehicleDatabase(1,i).xTravelPoints,vehicleDatabase(1,i).yTravelPoints,timeStep,Time,(step+1));
            end
           vehicleDatabase(1,i).updateVehiclePosition(step+1);
            EGOVehicle =i;
        else
            vehicleDatabase(1,i).drive(vehicleDatabase(1,i).xTravelPoints,vehicleDatabase(1,i).yTravelPoints,timeStep,Time,(step+1));
            vehicleDatabase(1,i).updateVehiclePosition(step+1);
        end
    end
    
    for i=1:size_bicycleDatabase(2)
        bicycleDatabase(1,i).updateBicyclePosition(step);
    end
    %
    % Traffic signal timing (red to green)
    
    % if vehicle break as no path is found and it collides (intersects) with other object, simulation is stopped with no solution 
    for k=1:size_kreuzungDatabase(2)
        
        if (rem(Time,3.0) <0.001) || (rem(Time,3.0) > 2.9999)
            size_ToTracks = size(kreuzungDatabase(1,k).noRoadToTracks);
            for l=1:size_ToTracks(2)
                if get(kreuzungDatabase(1,k).trafficSignal(l,1),'Color') == [0 1 0]
                    set(kreuzungDatabase(1,k).trafficSignal(l,:),'Color',[1 0 0]);
                    if l==size_ToTracks(2)
                        set(kreuzungDatabase(1,k).trafficSignal(1,:),'Color',[0 1 0]);
                    else
                        set(kreuzungDatabase(1,k).trafficSignal(l+1,:),'Color',[0 1 0]);
                    end
                    break;
                end
            end
        end
    end
    
     pause(1e-50);
end

%% For storing the data    
if 1 %exist('new_treeDatabase1','var')
file=num2str(count);
save(file,'roadDatabase','vehicleDatabase','kreuzungDatabase', 'pedestrianDatabase', 'bicycleDatabase','objectDatabase');
%save(file,'roadDatabase','vehicleDatabase','kreuzungDatabase', 'pedestrianDatabase', 'bicycleDatabase','collisionTime','objectDatabase','new_treeDatabase1','selectedTrajectory','breakEGO','collisionTimeStep','originalXTravelPoints','originalYTravelPoints');
%save(file,'roadDatabase','vehicleDatabase','kreuzungDatabase', 'pedestrianDatabase', 'bicycleDatabase','collisionTime','objectDatabase','new_treeDatabase','selectedTrajectory','breakEGO');
% vehicleDatabase(1,EGOVehicle).xTravelPoints=originalXTravelPoints;
% vehicleDatabase(1,EGOVehicle).yTravelPoints=originalYTravelPoints;
end

