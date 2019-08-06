function [count]=vehicle_para_update(tempVehicleDatabase,pedestrianDatabase,count)
% code for creating defferent scenarios by permuting the initial parameters
% of vehicle and pedestrians
if ~isempty(tempVehicleDatabase)
    % storing the original parameters of the vehicle
    originalxCG=tempVehicleDatabase(1,1).xCG(1);
    originalyCG=tempVehicleDatabase(1,1).yCG(1);
    originalSpeed=tempVehicleDatabase(1,1).speed(1);
    originalAccn=tempVehicleDatabase(1,1).longitudinalAccn(1);
    for i=1:3
        % changing the position
        [~,idx]=min(sqrt((tempVehicleDatabase(1,1).xTravelPoints-tempVehicleDatabase(1,1).xCG(1)).^2+(tempVehicleDatabase(1,1).yTravelPoints-tempVehicleDatabase(1,1).yCG(1)).^2));
        tempVehicleDatabase(1,1).xCG=tempVehicleDatabase(1,1).xTravelPoints(idx+20);
        tempVehicleDatabase(1,1).yCG=tempVehicleDatabase(1,1).yTravelPoints(idx+20);
        tempVehicleDatabase(1,1).angleDir(1)=atan2(tempVehicleDatabase(1,1).yTravelPoints(idx+7)-tempVehicleDatabase(1,1).yTravelPoints(idx+5),(tempVehicleDatabase(1,1).xTravelPoints(idx+7)-tempVehicleDatabase(1,1).xTravelPoints(idx+5)));
        tempVehicleDatabase(1,1).updateVehiclePosition(1);
        for j=1:3
            % changing the velocity
            tempVehicleDatabase(1,1).speed(1)=tempVehicleDatabase(1,1).speed(1)+2;
            
            for k=1:1
                ax=0;
                tempVehicleDatabase(1,1).longitudinalAccn=ax;
                temptempVehicleDatabase=tempVehicleDatabase;
                temptempVehicleDatabase(:,1) = [];
                tempPedestrianDatabase=pedestrianDatabase;
                [count]=vehicle_para_update(temptempVehicleDatabase,tempPedestrianDatabase,count);
               [flag]=0;% intersectionDetection();
               if flag==0 && count>0
             simulateScenario(count);
               end
                count=count+1;
            end
        end
         tempVehicleDatabase(1,1).speed(1)=originalSpeed;
        
    end
elseif ~isempty(pedestrianDatabase)
    % storing the original parameters of the pedestrians
    originalxCG=pedestrianDatabase(1,1).xPos(1);
    originalyCG=pedestrianDatabase(1,1).yPos(1);
    originalSpeed=pedestrianDatabase(1,1).speed(1);
    originalAccn=pedestrianDatabase(1,1).accn(1);
     
     
     for i=1:2
         % changing the position
        [~,idx]=min(sqrt((pedestrianDatabase(1,1).xTravelPoints-pedestrianDatabase(1,1).xPos(1)).^2+(pedestrianDatabase(1,1).yTravelPoints-pedestrianDatabase(1,1).yPos(1)).^2));
        pedestrianDatabase(1,1).xPos=pedestrianDatabase(1,1).xTravelPoints(idx+10);
        pedestrianDatabase(1,1).yPos=pedestrianDatabase(1,1).yTravelPoints(idx+10);
        
        pedestrianDatabase(1,1).angleDir(1)=atan2(pedestrianDatabase(1,1).yTravelPoints(idx+5)-pedestrianDatabase(1,1).yTravelPoints(idx+4),(pedestrianDatabase(1,1).xTravelPoints(idx+5)-pedestrianDatabase(1,1).xTravelPoints(idx+4)));
        set(pedestrianDatabase(1,1).handle,'Position',[pedestrianDatabase(1,1).xPos(1), pedestrianDatabase(1,1).yPos(1),0.8,0.8]);
        for j=1:2
            % changing the velocity
            pedestrianDatabase(1,1).speed(1)=pedestrianDatabase(1,1).speed(1)+0.3;
            for k=1:1
                ax=0;%-0.4+k*0.2;
                pedestrianDatabase(1,1).accn=ax;
                temptempVehicleDatabase=tempVehicleDatabase;
                tempPedestrianDatabase=pedestrianDatabase;
                tempPedestrianDatabase(:,1)=[];
                if ~isempty(tempPedestrianDatabase)
                    vehicle_para_update(temptempVehicleDatabase,tempPedestrianDatabase,count);
                end
                %%% simulate
                %flag=inters2(tempVehicleDatabase,pedestrianDatabase);
               [flag]=0;
               if flag==0 && count>0
           simulateScenario(count);
               end
             
count=count+1;
                
             
            end
        end
         pedestrianDatabase(1,1).speed(1)=originalSpeed;
        
    end
end

if ~isempty(tempVehicleDatabase)
    tempVehicleDatabase(1,1).xCG=originalxCG;
    tempVehicleDatabase(1,1).yCG=originalyCG;
   % tempVehicleDatabase(1,1).speed(1)=originalSpeed;
    tempVehicleDatabase(1,1).longitudinalAccn(1)=originalAccn;
    Phi=atan(tempVehicleDatabase(1,1).yTravelPoints(idx)-tempVehicleDatabase(1,1).yTravelPoints(idx-2))/((tempVehicleDatabase(1,1).xTravelPoints(idx)-tempVehicleDatabase(1,1).xTravelPoints(idx-2)));
    tempVehicleDatabase(1,1).updateVehiclePosition(1);
elseif ~isempty(pedestrianDatabase)
    pedestrianDatabase(1,1).xPos=originalxCG;
    pedestrianDatabase(1,1).yPos=originalyCG;
    pedestrianDatabase(1,1).speed(1)=originalSpeed;
    pedestrianDatabase(1,1).accn(1)=originalAccn;
    set(pedestrianDatabase(1,1).handle,'Position',[originalxCG, originalyCG,0.8,0.8]);
end
