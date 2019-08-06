classdef Bicycle < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % bicycle parameters
        speed ;
        lateralAccn;
        longitudinalAccn;
        vehicleMass;
        heightCG;
        lengthVehicle;
        widthVehicle;
        steeringRatio;
        steeringAngle;
        angleDir;
        handle;
        
        xCG;
        yCG;
        
         %for collision detection
        xCoordinates;
        yCoordinates;
        
        psiRel; %angle difference between vehicle yaw angle and destination yaw angle
        ePsi; % error for yaw angle
        
         %information regarding the path to be travelled by vehicle
        trackHandles;
        xTravelPoints=[];
        yTravelPoints=[];
        
        %Points to be considered for stooping the vehicle (longitudinal
        %controller)
        xTrafficSignalPoints;
        yTrafficSignalPoints;
        trafficSignalHandles;
        xOtherVehiclePoints;
        yOtherVehiclePoints;
    end
    
    methods
        function obj = Bicycle()
             obj.speed=10/3.6;
            obj.lateralAccn=1.2;
            obj.longitudinalAccn=1;
            obj.vehicleMass=500;
            obj.heightCG=0.5;
            obj.lengthVehicle=2000;
            obj.widthVehicle=1000;
            obj.steeringRatio=3;
            obj.steeringAngle = 0;
            obj.angleDir=0;
        end
        
       function driveOneTrack(obj,xPoints,yPoints,T,currentStep)
            
            global vehicleDatabase;
            size_vehicleDatabase = size(vehicleDatabase);
            global bicycleDatabase;
            size_bicycleDatabase  = size(bicycleDatabase);
            size_xPoints = size(xPoints);
            step= size(obj.xCG);
           % currentStep = step(2)+1;
            stopPoint_end=size_xPoints(2);
            stopPoint_vehicle=size_xPoints(2);
            stopPoint_bicycle=size_xPoints(2);
            stopPoint_signal=size_xPoints(2);
            referenceSpeedTarget = 0;
            %             plot(xPoints(200),yPoints(200));
            %predictiondistance = obj.speed *
            
            if obj.speed(currentStep-1) < 15/3.6
                anticipationTime =1;
            elseif obj.speed(currentStep-1) < 50/3.6
                anticipationTime=(0.5-1)/(50-15)*obj.speed(currentStep-1)*3.6+1-(0.5-1)/(50-15)*15;
            else
                anticipationTime=0.5;
            end
            
            anticipationDistance =  obj.speed(currentStep-1) * anticipationTime;
            idxAnticipation =floor( anticipationDistance/0.2);
            
            % find CGPoint index of vehicle and goal index
            dist=(((xPoints - obj.xCG(currentStep-1)).^2) + (yPoints - obj.yCG(currentStep-1)).^2);
            [~,CGPoint]=min(dist);
            
             % find traffic signal or vehicle CG point index for
            % longitudinal controller
%             if obj.signalLongiController==1
%                 if ~isempty(obj.xTrafficSignalPoints)
%                     dist1=(((xPoints - obj.xTrafficSignalPoints(1,1)).^2) + (yPoints - obj.yTrafficSignalPoints(1,1)).^2);
%                     [~,stopPoint_signal]=min(dist1);
%                     
%                     if (get(obj.trafficSignalHandles(1,1),'Color') ==[0 1 0])
%                         stopPoint_signal = size_xPoints(2);
%                         obj.speed(currentStep-1)=21/3.6;
%                     end
%                 end
%             end
            
%                 noTrafficSignal = size(obj.xTrafficSignalPoints);
%                 for k=1:noTrafficSignal(2)
%                     if (xPoints(i)==obj.xTrafficSignalPoints(1,k) && yPoints(i) == obj.yTrafficSignalPoints(1,k)) && (CGPoint < i)
%                         if (get(obj.trafficSignalHandles(1,1),'Color') ==[1 0 0])
%                             if stopPoint_signal > i
%                                 stopPoint_signal = i;
%                             end
%                         elseif (get(obj.trafficSignalHandles(1,1),'Color') ==[0 1 0]) 
%                             stopPoint_signal = size_xPoints(2);
%                             obj.speed(currentStep-1)=10/3.6;
%                         end
%                     elseif (xPoints(i)==obj.xTrafficSignalPoints(1,1) && yPoints(i) == obj.yTrafficSignalPoints(1,1)) && (CGPoint > i)
%                         stopPoint_signal=size_xPoints(2);
%                         obj.speed(currentStep-1)=obj.speed(1);
%                     end
%                 end
           % if obj.otherObjLongiController==1
                for j=1:size_vehicleDatabase(2)
                    if (vehicleDatabase(1,j).handle ~= obj.handle)
                        dist2=(((xPoints - vehicleDatabase(1,j).xCG(end)).^2) + (yPoints - vehicleDatabase(1,j).yCG(end)).^2);
                        [min_dist2,stopPoint_next_vehicle]=min(dist2);

                        if (min_dist2 < 0.25) && (CGPoint < stopPoint_next_vehicle) && (stopPoint_next_vehicle < stopPoint_vehicle)
                            stopPoint_vehicle=stopPoint_next_vehicle;
                            referenceSpeedTarget = vehicleDatabase(1,j).speed(end);
                        end
                    end
                end
            %end
                
            
            % next point for longitudinal controller
             if stopPoint_vehicle < stopPoint_signal && stopPoint_vehicle < stopPoint_end
                stopPoint = stopPoint_vehicle;
             elseif stopPoint_signal < stopPoint_end
                stopPoint = stopPoint_signal;
                referenceSpeedTarget=0;
             else
                 stopPoint = stopPoint_end;
                 referenceSpeedTarget = obj.speed(currentStep-1);
             end
            
            %plot(xPoints(stopPoint),yPoints(stopPoint),'ro');
            %
            %longitudinal dynamic controller
            Kv=0.7;
            Ka=3;
            dDesired =6;
            aEGOmin=-9;
            aEGOmax =1;
             
            
            VRel = referenceSpeedTarget-obj.speed(currentStep-1);
            
            remainingDistance = (stopPoint - CGPoint)*0.2;
            VRelDesired  = -Kv*(remainingDistance-dDesired);
            ev=VRel-VRelDesired;
            TTI=remainingDistance/VRel;
            
           
            
            
%             if TTI <1.5
                obj.longitudinalAccn(currentStep)=min(0,max(aEGOmin,Ka*ev));
%             end
            
            obj.speed(currentStep) = obj.speed(currentStep-1) + T*obj.longitudinalAccn(currentStep-1);
            if obj.speed(currentStep) < 0
                obj.speed(currentStep) = 0;
            end
            %end longitudinal dynamic controller
            %plot(xGoal,yGoal,'yo');
            
            Kp=obj.speed(currentStep)/69.44445;
            Ki=0;
            
            speed_table = [13.899 11 9 7 5 3];
            Kang_table = [0.12 0.14 0.16 0.18 0.20 0.22];
            Kdist_table = [0.01 0.02 0.03 0.04 0.05 0.06];
            Kang=interp1(speed_table,Kang_table,obj.speed(currentStep));%obj.speed(currentStep)/1069;%0.013;max(2.5, (7-8)/(80-50)*obj.speed(currentStep-1)*3.6+8-(7-8)/(80-10)*10);
            Kdist=interp1(speed_table,Kdist_table,obj.speed(currentStep));%obj.speed(currentStep)/6949.5;%0.001;max(0.1, (0.1-0.4)/(100-80)*obj.speed(currentStep)*3.6+0.4-(0.1-0.4)/(100-80)*80);
            
            %             Kang=180/pi*max(2.5, (6-8)/(80-50)*obj.speed(currentStep-1)*3.6+8-(6-8)/(80-50)*50);
%             Kdist=180/pi*max(0.1, (0.1-0.4)/(100-80)*obj.speed(currentStep-1)*3.6+0.4-(0.1-0.4)/(100-80)*80);
            
            %% second P controller
             if (CGPoint + idxAnticipation) < size_xPoints(2)
               x2 = xPoints(CGPoint+idxAnticipation); x1= (xPoints(CGPoint+idxAnticipation-2));
               y2 = yPoints(CGPoint+idxAnticipation); y1= (yPoints(CGPoint+idxAnticipation-2));
%            
%              distToLin = sqrt((yPoints(CGPoint+idxAnticipation) - yPoints(CGPoint))^2 + (xPoints(CGPoint+idxAnticipation) - xPoints(CGPoint))^2);
           else
               x2 = xPoints(CGPoint); x1= (xPoints(CGPoint-2));
               y2 = yPoints(CGPoint); y1= (yPoints(CGPoint-2));
%                obj.psiRel(currentStep)=atan2((yPoints(end) - (yPoints(end-2))),(xPoints(end)- (xPoints(end-2))));
%                distToLin = sqrt((yPoints(end) - yPoints(CGPoint))^2 + (xPoints(end) - xPoints(CGPoint))^2);
           end
           
           if x2 > x1 + 1e-9
               aLin=-(y2-y1)/(x2-x1); bLin=1; cLin=((y2-y1)/(x2-x1))*x1-y1;
               distToLin=([aLin bLin cLin]*[obj.xCG(currentStep - 1); obj.yCG(currentStep - 1); 1])/sqrt(aLin^2+bLin^2);
               obj.psiRel(currentStep)=atan2(-aLin, bLin);
           elseif x1 > x2 + 1e-9
               aLin=-(y2-y1)/(x2-x1); bLin=1; cLin=((y2-y1)/(x2-x1))*x1-y1;
               distToLin=-([aLin bLin cLin]*[obj.xCG(currentStep - 1); obj.yCG(currentStep - 1); 1])/sqrt(aLin^2+bLin^2);
               obj.psiRel(currentStep)=atan2(aLin, -bLin);
           else
               if y2-y1>0
                   distToLin=x2-obj.xCG(currentStep - 1);
                   obj.psiRel(currentStep)=pi/2;
               else
                   distToLin=obj.xCG(currentStep - 1)-x2;
                   obj.psiRel(currentStep)=-pi/2;
               end
           end
           
            %% new position of vehicle
            obj.xCG(currentStep) = obj.xCG(currentStep-1) + T*obj.speed(currentStep)*cos(obj.angleDir(currentStep-1));
            obj.yCG(currentStep) = obj.yCG(currentStep-1) + T*obj.speed(currentStep)*sin(obj.angleDir(currentStep-1));
            %new yaw angle of vehicle
            obj.angleDir(currentStep) = obj.angleDir(currentStep-1) + T*(obj.speed(currentStep)/obj.lengthVehicle)*tan(obj.steeringAngle(currentStep-1));
            %relative yaw angle
           % obj.psiRel(currentStep) = atan2((yGoal-obj.yCG(currentStep-1)),(xGoal-obj.xCG(currentStep-1)));
            %error
            obj.ePsi(currentStep)=obj.psiRel(currentStep)-obj.angleDir(currentStep);
            if obj.ePsi(currentStep)>pi
                obj.ePsi(currentStep)=obj.ePsi(currentStep)-2*pi;
            elseif obj.ePsi(currentStep)< -pi
                obj.ePsi(currentStep)=obj.ePsi(currentStep)+2*pi;
            end
            
            %Integral error
            size_error = size(obj.ePsi);
            integral_error = 0;
            for i=1:size_error(2)
                integral_error = integral_error + obj.ePsi(i)*T;
            end
            %
          
            obj.steeringAngle(currentStep) = sign(obj.ePsi(currentStep))*min((720/13.5)*pi/180, abs(Kang*obj.ePsi(currentStep)+Ki*integral_error-Kdist*distToLin));
            maxGradiantDelta=((180/obj.steeringRatio)/0.5)*pi/180; % maximal steering gradient: 180 degree in 1.5 seconds
            %steering angle
            if abs(obj.steeringAngle(currentStep)-obj.steeringAngle(currentStep-1))/T>maxGradiantDelta
                obj.steeringAngle(currentStep) = obj.steeringAngle(currentStep-1) + sign(obj.ePsi(currentStep))*(maxGradiantDelta*T);
            end
            
            %Current  lateral acceleration
            
            if abs(obj.steeringAngle(currentStep)) > 0.0001
                R(currentStep) = (obj.widthVehicle/1000)/tan(obj.steeringAngle(currentStep));
            else
                R(currentStep) = 1e+5;
            end
            obj.lateralAccn(currentStep) = obj.speed(currentStep)^2/R(currentStep);
            obj.angleDir(currentStep) =obj.angleDir(currentStep-1)+obj.speed(currentStep)*tan(obj.steeringAngle(currentStep))/(obj.lengthVehicle/1000);
            %% vehicle position update
            
            ex=cos(obj.angleDir(currentStep));%cos((str2double(get(handles.vehicleDiretion,'String')))*pi/180);
            ey=sin(obj.angleDir(currentStep));%sin((str2double(get(handles.vehicleDiretion,'String')))*pi/180);
            exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
            eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
            scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
            x_Data = zeros(4,1);
            y_Data = zeros(4,1);
            
             x_Data(3)=obj.xCG(currentStep)+ex*(((obj.lengthVehicle))/2000*scalingUnit)+((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(3)=obj.yCG(currentStep)+ey*(((obj.lengthVehicle))/2000*scalingUnit)+((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(4)=obj.xCG(currentStep)+ex*(((obj.lengthVehicle))/2000*scalingUnit)-((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(4)=obj.yCG(currentStep)+ey*(((obj.lengthVehicle))/2000*scalingUnit)-((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(2)=obj.xCG(currentStep)-ex*(((obj.lengthVehicle))/2000*scalingUnit)+((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(2)=obj.yCG(currentStep)-ey*(((obj.lengthVehicle))/2000*scalingUnit)+((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(1)=obj.xCG(currentStep)-ex*(((obj.lengthVehicle))/2000*scalingUnit)-((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(1)=obj.yCG(currentStep)-ey*(((obj.lengthVehicle))/2000*scalingUnit)-((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            
            obj.xCoordinates(currentStep,:) = [x_Data(1) x_Data(2) x_Data(3) x_Data(4) x_Data(1)];
            obj.yCoordinates(currentStep,:) = [y_Data(1) y_Data(2) y_Data(3) y_Data(4) y_Data(1)];
            
            %set(obj.handle,'xData',x_Data,'yData',y_Data);
            %vehicle path
            plot(obj.xCG,obj.yCG,'r-');
            
            
            
       end
        
       function updateBicyclePosition(obj,currentStep)
            set(obj.handle,'xData',obj.xCoordinates(currentStep,:),'yData',obj.yCoordinates(currentStep,:));
       end
    end
    
end

