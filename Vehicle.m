classdef Vehicle < handle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % vehicle parameters
        type;
        speed ;
        lateralAccn;
        longitudinalAccn;
        vehicleMass;
        heightCG;
        lengthVehicle;
        widthVehicle;
        lf;
        lr;
        lfToFront;
        lrToRear;
        steeringRatio;
        steeringAngle;
        angleDir; %yaw angle
        handle; %vehicle object (patch) handle
        
        steeringAngleDriver;
        
        %for collision detection
        xCoordinates;
        yCoordinates;
        
        
        %Tyre Properties
        cl_fl;
        bl_fl;
        cs_fl;
        bs_fl;
        cl_fr;
        bl_fr;
        cs_fr;
        bs_fr;
        cl_rl;
        bl_rl;
        cs_rl;
        bs_rl;
        cl_rr;
        bl_rr;
        cs_rr;
        bs_rr;
        
        Iz;                                        % MI around Z axis
        xCG;
        yCG;
        %the geometrical center of vehicle
        xCenter;
        yCenter;
        radiusCollisionControl;
        psiRel; %angle difference between vehicle yaw angle and destination yaw angle
        ePsi; % error for yaw angle
        
        %% two track model variables
        slipLongitudinal_fl;
        slipLongitudinal_fr;
        slipLongitudinal_rl;
        slipLongitudinal_rr;
        
        delta_fl;
        delta_fr;
        delta_rl;
        delta_rr;
        
        % Forces on tyre in Z direction
        Fz_fl;
        Fz_fr;
        Fz_rl;
        Fz_rr;
        
        % longitudinal forces on tyre
        Fl_fl;
        Fl_fr;
        Fl_rl;
        Fl_rr;
        
        %
        alpha_fl;
        alpha_fr;
        alpha_rl;
        alpha_rr;
        
        %Sideslip
        slipSide_fl;
        slipSide_fr;
        slipSide_rl;
        slipSide_rr;
        
        %Lateral forces on tyre
        Fs_fl;
        Fs_fr;
        Fs_rl;
        Fs_rr;
        
        %sideslip angle
        beta;
        dotPhi;
        
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
        
        %driver model
        signalLongiController=1;
        otherObjLongiController=1;
    end
    
    methods
        function obj= Vehicle()

            obj.type = 'Other';
            obj.speed=50/3.6;
            obj.lateralAccn=0;
            obj.longitudinalAccn=0;
            obj.vehicleMass=2485;
            obj.heightCG=400;
            obj.lengthVehicle=4943;
            obj.widthVehicle=1874;
            obj.lf=1500;%1567;
            obj.lr=1410;%1346;
            obj.lfToFront=930;
            obj.lrToRear=1100;
            obj.steeringRatio=13.5;
            obj.angleDir=0;
            obj.cl_fl=1.8;%1.4;
            obj.bl_fl=19;%13;
            obj.cs_fl=0.8055;%1.25;
            obj.bs_fl=7.2;%17.2;
            obj.cl_fr=1.8;
            obj.bl_fr=19;
            obj.cs_fr=0.9;
            obj.bs_fr=7.2;
            obj.cl_rl=1.8;
            obj.bl_rl=19;
            obj.cs_rl=0.9;
            obj.bs_rl=7.2;
            obj.cl_rr=1.8;
            obj.bl_rr=19;
            obj.cs_rr=0.9;
            obj.bs_rr=7.2;
            obj.steeringAngle = 0;
            obj.steeringAngleDriver = 0;
            obj.Iz =2000;
            %obj.desiredSpeed=NaN;
            
            %% two track model variables
            obj.slipLongitudinal_fl=0;
            obj.slipLongitudinal_fr=0;
            obj.slipLongitudinal_rl=0;
            obj.slipLongitudinal_rr=0;
            
            obj.delta_fl=0;
            obj.delta_fr=0;
            obj.delta_rl=0;
            obj.delta_rr=0;
            
            % Forces on tyre in Z direction
            obj.Fz_fl=0;
            obj.Fz_fr=0;
            obj.Fz_rl=0;
            obj.Fz_rr=0;
            
            % longitudinal forces on tyre
            obj.Fl_fl=0;
            obj.Fl_fr=0;
            obj.Fl_rl=0;
            obj.Fl_rr=0;
            
            %
            obj.alpha_fl=0;
            obj.alpha_fr=0;
            obj.alpha_rl=0;
            obj.alpha_rr=0;
            
            %Sideslip
            obj.slipSide_fl=0;
            obj.slipSide_fr=0;
            obj.slipSide_rl=0;
            obj.slipSide_rr=0;
            
            %Lateral forces on tyre
            obj.Fs_fl=0;
            obj.Fs_fr=0;
            obj.Fs_rl=0;
            obj.Fs_rr=0;
            
            obj.beta = 0;
            obj.dotPhi=0;
            
        end
        
        %% Kinematic model
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
            if obj.signalLongiController==1
                if ~isempty(obj.xTrafficSignalPoints)
                    dist1=(((xPoints - obj.xTrafficSignalPoints(1,1)).^2) + (yPoints - obj.yTrafficSignalPoints(1,1)).^2);
                    [~,stopPoint_signal]=min(dist1);
                    
                    if (get(obj.trafficSignalHandles(1,1),'Color') ==[0 1 0])
                        stopPoint_signal = size_xPoints(2);
                        obj.speed(currentStep-1)=21/3.6;
                    end
                end
            end
            
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
            if obj.otherObjLongiController==1
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
            end
                
            
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
            scalingUnit =1;% 	(get(handles.scalingUnit,'String'));
            x_Data = zeros(4,1);
            y_Data = zeros(4,1);
            
            x_Data(3)=obj.xCG(currentStep)+ex*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)+((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(3)=obj.yCG(currentStep)+ey*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)+((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(4)=obj.xCG(currentStep)+ex*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)-((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(4)=obj.yCG(currentStep)+ey*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)-((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(2)=obj.xCG(currentStep)-ex*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)+((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(2)=obj.yCG(currentStep)-ey*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)+((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(1)=obj.xCG(currentStep)-ex*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)-((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(1)=obj.yCG(currentStep)-ey*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)-((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            
            obj.xCoordinates(currentStep,:) = [x_Data(1) x_Data(2) x_Data(3) x_Data(4) x_Data(1)];
            obj.yCoordinates(currentStep,:) = [y_Data(1) y_Data(2) y_Data(3) y_Data(4) y_Data(1)];
            
            set(obj.handle,'xData',x_Data,'yData',y_Data);
%             %vehicle path
           % plot(obj.xCG,obj.yCG,'r-','Linewidth',1.25);
            
            
            
        end
        
        
        
        function drive(obj,xPoints,yPoints,T,totalTime,currentStep)
            
%            tic
            g=9.81;                                         % accn due to gravity
            
            factorMu=1; % 0.6; %Friction coefficients
            mul_f=factorMu*1.0;
            mul_r=factorMu*1.0;
            mus_f=factorMu*0.9;
            mus_r=factorMu*0.9;
            
            mul_fl=mul_f;
            mus_fl=mus_f;
            mul_fr=mul_f;
            mus_fr=mus_f;
            mul_rl=mul_r;
            mus_rl=mus_r;
            mul_rr=mul_r;
            mus_rr=mus_r;
            
            ell=(obj.lf+obj.lr);
            size_xPoints = size(xPoints);
            
            %currentStep = step(2)+1;
            
            global vehicleDatabase;
            size_vehicleDatabase = size(vehicleDatabase);
            
            
            stopPoint_end=size_xPoints(2);
            stopPoint_vehicle=size_xPoints(2);
            stopPoint_signal=size_xPoints(2);
            referenceSpeedTarget = 0;
            %desired distance between two vehicle for longitudinal control
            dDesired = 6;
            
            %PID gains
            Kang=180/pi*max(2.5, (6-8)/(80-50)*obj.speed(currentStep-1)*3.6+8-(6-8)/(80-50)*50);
            Kdist=180/pi*max(0.1, (0.1-0.4)/(100-80)*obj.speed(currentStep-1)*3.6+0.4-(0.1-0.4)/(100-80)*80);
            Ki =0;
            Kv=0.7;
            
            
            %Anticipation time 
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
%           
            % find traffic signal or vehicle CG point index for
            % longitudinal controller
            if obj.signalLongiController==1
                if ~isempty(obj.xTrafficSignalPoints)
                    dist1=(((xPoints - obj.xTrafficSignalPoints(1,1)).^2) + (yPoints - obj.yTrafficSignalPoints(1,1)).^2);
                    [~,stopPoint_signal]=min(dist1);
                    
                    if (get(obj.trafficSignalHandles(1,1),'Color') ==[0 1 0])
                        stopPoint_signal = size_xPoints(2);
                        obj.speed(currentStep-1)=21/3.6;
                    end
                end
            end
            
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
            if obj.otherObjLongiController==1
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
            end
                
            
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
            
            VRel = referenceSpeedTarget-obj.speed(currentStep-1);
            
            remainingDistance = (stopPoint - CGPoint)*0.2;
            VRelDesired  = -Kv*(remainingDistance-dDesired);
            ev=VRel-VRelDesired;
            
            
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
           
           % error calculation
            %obj.psiRel(currentStep)=atan2((yPoints(CGPoint+idxAnticipation) - (yPoints(CGPoint))),(xPoints(CGPoint+idxAnticipation)-(xPoints(CGPoint))));
             obj.ePsi(currentStep) = obj.psiRel(currentStep) - obj.angleDir(currentStep-1);
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
            
           
             maxSteervelocity=180/0.5;
            obj.steeringAngleDriver(currentStep) = (Kang*obj.ePsi(currentStep)+Ki*integral_error-Kdist*distToLin); 
            if abs( obj.steeringAngleDriver(currentStep) -  obj.steeringAngleDriver(currentStep-1))/T >  maxSteervelocity
                obj.steeringAngleDriver(currentStep) = obj.steeringAngleDriver(currentStep-1) + ...
                    sign(obj.steeringAngleDriver(currentStep) -  obj.steeringAngleDriver(currentStep-1))*maxSteervelocity*T;
            end
            
            

            if obj.steeringAngleDriver(currentStep) > 720
                obj.steeringAngleDriver(currentStep)=720;
            elseif obj.steeringAngleDriver(currentStep) < -720
                obj.steeringAngleDriver(currentStep)=-720;
            end
           
%              
            % DETERMINE MANEUVER
            
            
            ka=0.02;
            %
            % Longitudinal
            tSchwell=0.6;  % For initial period longitudinal slip is linear
            
            if  obj.longitudinalAccn(currentStep-1) < 0
                maxMinSlip=min(0,max(-0.1,ka*ev));
            else
                 maxMinSlip=0;
            end
            if totalTime<=tSchwell
                obj.slipLongitudinal_fl(currentStep)=totalTime/tSchwell*maxMinSlip; %here: negative for braking
                obj.slipLongitudinal_fr(currentStep)=totalTime/tSchwell*maxMinSlip; %here: negative for braking
                obj.slipLongitudinal_rl(currentStep)=totalTime/tSchwell*maxMinSlip; %here: negative for braking
                obj.slipLongitudinal_rr(currentStep)=totalTime/tSchwell*maxMinSlip; %here: negative for braking
            else
                obj.slipLongitudinal_fl(currentStep)=maxMinSlip; %here: negative for braking
                obj.slipLongitudinal_fr(currentStep)=maxMinSlip; %here: negative for braking
                obj.slipLongitudinal_rl(currentStep)=maxMinSlip; %here: negative for braking
                obj.slipLongitudinal_rr(currentStep)=maxMinSlip; %here: negative for braking
            end
            
            obj.delta_fl(currentStep)=(obj.steeringAngleDriver(currentStep)/obj.steeringRatio)*pi/180;
            obj.delta_fr(currentStep)=(obj.steeringAngleDriver(currentStep)/obj.steeringRatio)*pi/180;
            obj.delta_rl(currentStep)=0;
            obj.delta_rr(currentStep)=0;
            %
            %
            %
            % VEHICLE DYNAMICS MODEL
            
            if currentStep<=2
                obj.longitudinalAccn(currentStep) =obj.longitudinalAccn(1);
                obj.lateralAccn(currentStep)=obj.lateralAccn(1);
            else
%                  ax(n)=((vForVisual(n)-vForVisual(n-1))/T)*cos(beta_n)-v_n*(((betaForVisual(n)-betaForVisual(n-1))/T+dotPhi_n))*sin(beta_n);
%         ay(n)=((vForVisual(n)-vForVisual(n-1))/T)*sin(beta_n)+v_n*(((betaForVisual(n)-betaForVisual(n-1))/T+dotPhi_n))*cos(beta_n);
                
               obj.longitudinalAccn(currentStep) =((obj.speed(currentStep-1)-obj.speed(currentStep-2))/T)*cos(obj.beta(currentStep-1))-...
                    obj.speed(currentStep-1)*(((obj.beta(currentStep-1)-obj.beta(currentStep-2))/T+obj.dotPhi(currentStep-1)))*sin(obj.beta(currentStep-1));     % Slide page45
                %obj.longitudinalAccn(currentStep) =  min(obj.longitudinalAccn(currentStep),ax);
                obj.lateralAccn(currentStep)=((obj.speed(currentStep-1)-obj.speed(currentStep-2))/T)*sin(obj.beta(currentStep-1))+...
                    obj.speed(currentStep-1)*(((obj.beta(currentStep-1)-obj.beta(currentStep-2))/T)+obj.dotPhi(currentStep-1))*cos(obj.beta(currentStep-1));     % Slide page45
              
            end
            
            obj.Fz_fl(currentStep)=obj.vehicleMass*(obj.lr/ell-obj.heightCG/ell*obj.longitudinalAccn(currentStep)/g)*(g/2-obj.heightCG/obj.widthVehicle*obj.lateralAccn(currentStep));      % Slide page45
            obj.Fz_fr(currentStep)=obj.vehicleMass*(obj.lr/ell-obj.heightCG/ell*obj.longitudinalAccn(currentStep)/g)*(g/2+obj.heightCG/obj.widthVehicle*obj.lateralAccn(currentStep));      % Slide page45
            obj.Fz_rl(currentStep)=obj.vehicleMass*(obj.lf/ell+obj.heightCG/ell*obj.longitudinalAccn(currentStep)/g)*(g/2-obj.heightCG/obj.widthVehicle*obj.lateralAccn(currentStep));      % Slide page45
            obj.Fz_rr(currentStep)=obj.vehicleMass*(obj.lf/ell+obj.heightCG/ell*obj.longitudinalAccn(currentStep)/g)*(g/2+obj.heightCG/obj.widthVehicle*obj.lateralAccn(currentStep));      % Slide page45
            %Forces on front-left tire
            %Fl_fl(n)=Fz_fl(n)*mul_fl*sin(cl_fl*atan2(bl_fl*slipLongitudinal_fl(n), mul_fl));
            obj.Fl_fl(currentStep)=obj.Fz_fl(currentStep)*mul_fl*sin(obj.cl_fl*atan2(obj.bl_fl*obj.slipLongitudinal_fl(currentStep-1), mul_fl));
            obj.alpha_fl(currentStep)=obj.delta_fl(currentStep)-atan2((obj.speed(currentStep-1)*sin(obj.beta(currentStep-1))+(obj.lf/1000)*obj.dotPhi(currentStep-1)),...
                (obj.speed(currentStep-1)*cos(obj.beta(currentStep-1))-obj.widthVehicle/2000*obj.dotPhi(currentStep-1)));
            % alpha_fl(n)=delta_fl(n)-atan2((v_n*sin(beta_n)+lf*dotPhi_n),(v_n*cos(beta_n)-w/2*dotPhi_n));
            obj.slipSide_fl(currentStep)=tan(obj.alpha_fl(currentStep));
            
            obj.Fs_fl(currentStep)=obj.Fz_fl(currentStep)*mus_fl*sin(obj.cs_fl*atan2(obj.bs_fl*obj.slipSide_fl(currentStep), mus_fl));
            forceDirection_fl=atan2(obj.slipSide_fl(currentStep), obj.slipLongitudinal_fl(currentStep));
            absolutSlip_fl=sqrt(obj.slipLongitudinal_fl(currentStep)^2+obj.slipSide_fl(currentStep)^2);
            if absolutSlip_fl<1e-3
                absoluteForce_fl=0;
            else
                %absoluteForce_fl=sqrt((slipLongitudinal_fl(n)/absolutSlip_fl)^2*Fl_fl(n)^2+(slipSide_fl(n)/absolutSlip_fl)^2*Fs_fl(n)^2);
                absoluteForce_fl=sqrt((obj.slipLongitudinal_fl(currentStep)/absolutSlip_fl)^2*obj.Fl_fl(currentStep)^2+...
                    (obj.slipSide_fl(currentStep)/absolutSlip_fl)^2*obj.Fs_fl(currentStep)^2);
            end
            obj.Fl_fl(currentStep)= absoluteForce_fl*cos(forceDirection_fl);
            obj.Fs_fl(currentStep)= absoluteForce_fl*sin(forceDirection_fl);
            %Forces on front-right tire
            obj.Fl_fr(currentStep)=obj.Fz_fr(currentStep)*mul_fr*sin(obj.cl_fr*atan2(obj.bl_fr*obj.slipLongitudinal_fr(currentStep), mul_fr)); %Magic Tire Formula Slide
            obj.alpha_fr(currentStep)=obj.delta_fr(currentStep)-atan2((obj.speed(currentStep-1)*sin(obj.beta(currentStep-1))+obj.lf/1000*obj.dotPhi(currentStep-1)),...
                (obj.speed(currentStep-1)*cos(obj.beta(currentStep-1))+obj.widthVehicle/2000*obj.dotPhi(currentStep-1)));  %Slide page41
            % alpha_fl(n)=delta_fl(n)-atan2((v_n*sin(beta_n)+lf*dotPhi_n),(v_n*cos(beta_n)-w/2*dotPhi_n));
            obj.slipSide_fr(currentStep)=tan(obj.alpha_fr(currentStep)); %Sideslip = tan(steering angle)
            obj.Fs_fr(currentStep)=obj.Fz_fr(currentStep)*mus_fr*sin(obj.cs_fr*atan2(obj.bs_fr*obj.slipSide_fr(currentStep), mus_fr));  %Magic Tire Formula Slide
            forceDirection_fr=atan2(obj.slipSide_fr(currentStep), obj.slipLongitudinal_fr(currentStep));    %Slide page36
            absolutSlip_fr=sqrt(obj.slipLongitudinal_fr(currentStep)^2+obj.slipSide_fr(currentStep)^2);     %Slide page36
            if absolutSlip_fr<1e-3
                absoluteForce_fr=0;
            else
                absoluteForce_fr=sqrt((obj.slipLongitudinal_fr(currentStep)/absolutSlip_fr)^2*obj.Fl_fr(currentStep)^2+...
                    (obj.slipSide_fr(currentStep)/absolutSlip_fr)^2*obj.Fs_fr(currentStep)^2); %Slide page36
            end
            obj.Fl_fr(currentStep)=absoluteForce_fr*cos(forceDirection_fr);
            obj.Fs_fr(currentStep)=absoluteForce_fr*sin(forceDirection_fr);
            %Forces on rear-left tire
            obj.Fl_rl(currentStep)=obj.Fz_rl(currentStep)*mul_rl*sin(obj.cl_rl*atan2(obj.bl_rl*obj.slipLongitudinal_rl(currentStep), mul_rl));
            obj.alpha_rl(currentStep)=obj.delta_rl(currentStep)-atan2((obj.speed(currentStep-1)*sin(obj.beta(currentStep-1))-obj.lr/1000*obj.dotPhi(currentStep-1)),...
                (obj.speed(currentStep-1)*cos(obj.beta(currentStep-1))-obj.widthVehicle/2000*obj.dotPhi(currentStep-1)));
            obj.slipSide_rl(currentStep)=tan(obj.alpha_rl(currentStep));
            obj.Fs_rl(currentStep)=obj.Fz_rl(currentStep)*mus_rl*sin(obj.cs_rl*atan2(obj.bs_rl*obj.slipSide_rl(currentStep), mus_rl));
            forceDirection_rl=atan2(obj.slipSide_rl(currentStep), obj.slipLongitudinal_rl(currentStep));
            absolutSlip_rl=sqrt(obj.slipLongitudinal_rl(currentStep)^2+obj.slipSide_rl(currentStep)^2);
            if absolutSlip_rl<1e-3
                absoluteForce_rl=0;
            else
                absoluteForce_rl=sqrt((obj.slipLongitudinal_rl(currentStep)/absolutSlip_rl)^2*obj.Fl_rl(currentStep)^2+...
                    (obj.slipSide_rl(currentStep)/absolutSlip_rl)^2*obj.Fs_rl(currentStep)^2);
            end
            obj.Fl_rl(currentStep)=absoluteForce_rl*cos(forceDirection_rl);
            obj.Fs_rl(currentStep)=absoluteForce_rl*sin(forceDirection_rl);
            %Forces on rear-right tire
            obj.Fl_rr(currentStep)=obj.Fz_rr(currentStep)*mul_rr*sin(obj.cl_rr*atan2(obj.bl_rr*obj.slipLongitudinal_rr(currentStep), mul_rr));
            obj.alpha_rr(currentStep)=obj.delta_rr(currentStep)-atan2((obj.speed(currentStep-1)*sin(obj.beta(currentStep-1))-obj.lr/1000*obj.dotPhi(currentStep-1)),(obj.speed(currentStep-1)*cos(obj.beta(currentStep-1))+obj.widthVehicle/2000*obj.dotPhi(currentStep-1)));
            obj.slipSide_rr(currentStep)=tan(obj.alpha_rr(currentStep));
            obj.Fs_rr(currentStep)=obj.Fz_rr(currentStep)*mus_rr*sin(obj.cs_rr*atan2(obj.bs_rr*obj.slipSide_rr(currentStep), mus_rr));
            forceDirection_rr=atan2(obj.slipSide_rr(currentStep), obj.slipLongitudinal_rr(currentStep));
            absolutSlip_rr=sqrt(obj.slipLongitudinal_rr(currentStep)^2+obj.slipSide_rr(currentStep)^2);
            if absolutSlip_rr<1e-3
                absoluteForce_rr=0;
            else
                absoluteForce_rr=sqrt((obj.slipLongitudinal_rr(currentStep)/absolutSlip_rr)^2*obj.Fl_rr(currentStep)^2+...
                    (obj.slipSide_rr(currentStep)/absolutSlip_rr)^2*obj.Fs_rr(currentStep)^2);
            end
            obj.Fl_rr(currentStep)=absoluteForce_rr*cos(forceDirection_rr);
            obj.Fs_rr(currentStep)=absoluteForce_rr*sin(forceDirection_rr);
            
            % SOLVING DIFFERENTIAL EQUATION NUMERICALLY
            obj.speed(currentStep) = obj.speed(currentStep-1) + T*(1/obj.vehicleMass)*...
                (obj.Fl_fl(currentStep)*cos(obj.delta_fl(currentStep)-obj.beta(currentStep-1))+...
                obj.Fl_fr(currentStep)*cos(obj.delta_fr(currentStep)-obj.beta(currentStep-1))-...
                obj.Fs_fl(currentStep)*sin(obj.delta_fl(currentStep)-obj.beta(currentStep-1))-...
                obj.Fs_fr(currentStep)*sin(obj.delta_fr(currentStep)-obj.beta(currentStep-1))+...
                obj.Fl_rl(currentStep)*cos(obj.delta_rl(currentStep)-obj.beta(currentStep-1))+...
                obj.Fl_rr(currentStep)*cos(obj.delta_rr(currentStep)-obj.beta(currentStep-1))-...
                obj.Fs_rl(currentStep)*sin(obj.delta_rl(currentStep)-obj.beta(currentStep-1))-...
                obj.Fs_rr(currentStep)*sin(obj.delta_rr(currentStep)-obj.beta(currentStep-1)));
            if obj.speed(currentStep)>3/3.6          % ????
                
                
%                 beta_np1 = beta_n + T*(1/(m*v_n)*(Fl_fl(n)*sin(delta_fl(n)-beta_n)+Fl_fr(n)*sin(delta_fr(n)-beta_n)+Fs_fl(n)*cos(delta_fl(n)-beta_n)+...
%             Fs_fr(n)*cos(delta_fr(n)-beta_n)+Fl_rl(n)*sin(delta_rl(n)-beta_n)+Fl_rr(n)*sin(delta_rr(n)-beta_n)+...
%             Fs_rl(n)*cos(delta_rl(n)-beta_n)+Fs_rr(n)*cos(delta_rr(n)-beta_n))-dotPhi_n);
        
                obj.beta(currentStep)=obj.beta(currentStep-1) + T*((1/(obj.vehicleMass*obj.speed(currentStep-1)))*...
                   (obj.Fl_fl(currentStep)*sin(obj.delta_fl(currentStep)-obj.beta(currentStep-1))+...
                    obj.Fl_fr(currentStep)*sin(obj.delta_fr(currentStep)-obj.beta(currentStep-1))+...
                    obj.Fs_fl(currentStep)*cos(obj.delta_fl(currentStep)-obj.beta(currentStep-1))+...
                    obj.Fs_fr(currentStep)*cos(obj.delta_fr(currentStep)-obj.beta(currentStep-1))+...
                    obj.Fl_rl(currentStep)*sin(obj.delta_rl(currentStep)-obj.beta(currentStep-1))+...
                    obj.Fl_rr(currentStep)*sin(obj.delta_rr(currentStep)-obj.beta(currentStep-1))+...
                    obj.Fs_rl(currentStep)*cos(obj.delta_rl(currentStep)-obj.beta(currentStep-1))+...
                    obj.Fs_rr(currentStep)*cos(obj.delta_rr(currentStep)-obj.beta(currentStep-1)))-obj.dotPhi(currentStep-1));
               
              
                obj.dotPhi(currentStep) = obj.dotPhi(currentStep-1) + T*((1/obj.Iz)*...
                    ((obj.lf/1000)*(obj.Fl_fl(currentStep)*sin(obj.delta_fl(currentStep))+obj.Fl_fr(currentStep)*sin(obj.delta_fr(currentStep))+...
                    obj.Fs_fl(currentStep)*cos(obj.delta_fl(currentStep))+obj.Fs_fr(currentStep)*cos(obj.delta_fr(currentStep)))+...
                    (obj.widthVehicle/2000)*(obj.Fl_fr(currentStep)*cos(obj.delta_fr(currentStep))-obj.Fl_fl(currentStep)*cos(obj.delta_fl(currentStep))-...
                    obj.Fs_fr(currentStep)*sin(obj.delta_fr(currentStep))+obj.Fs_fl(currentStep)*sin(obj.delta_fl(currentStep)))-...
                    (obj.lr/1000)*(obj.Fl_rl(currentStep)*sin(obj.delta_rl(currentStep))+obj.Fl_rr(currentStep)*sin(obj.delta_rr(currentStep))+...
                    obj.Fs_rl(currentStep)*cos(obj.delta_rl(currentStep))+obj.Fs_rr(currentStep)*cos(obj.delta_rr(currentStep)))+...
                    (obj.widthVehicle/2000)*(obj.Fl_rr(currentStep)*cos(obj.delta_rr(currentStep))-obj.Fl_rl(currentStep)*cos(obj.delta_rl(currentStep))-...
                    obj.Fs_rr(currentStep)*sin(obj.delta_rr(currentStep))+obj.Fs_rl(currentStep)*sin(obj.delta_rl(currentStep)))));%slide28
            elseif obj.speed(currentStep)<0
                obj.beta(currentStep) = 0;
                obj.dotPhi(currentStep) = 0;
                obj.speed(currentStep) =0;
                obj.lateralAccn(currentStep)= 0;
                obj.alpha_fl(currentStep)=0;
                obj.alpha_fr(currentStep)=0;
                obj.alpha_rl(currentStep)=0;
                obj.alpha_rr(currentStep)=0;
                obj.Fs_fl(currentStep)=0;
                obj.Fs_fr(currentStep)=0;
                obj.Fs_rl(currentStep)=0;
                obj.Fs_rr(currentStep)=0;
            else
                obj.beta(currentStep) = 0;
                obj.dotPhi(currentStep) = 0;
                obj.lateralAccn(currentStep)= 0;
                obj.alpha_fl(currentStep)=0;
                obj.alpha_fr(currentStep)=0;
                obj.alpha_rl(currentStep)=0;
                obj.alpha_rr(currentStep)=0;
                obj.Fs_fl(currentStep)=0;
                obj.Fs_fr(currentStep)=0;
                obj.Fs_rl(currentStep)=0;
                obj.Fs_rr(currentStep)=0;
            end
            
            obj.angleDir(currentStep)=obj.angleDir(currentStep-1)+T*obj.dotPhi(currentStep-1);
%             obj.steeringAngle(currentStep) = (obj.delta_fl(currentStep) + obj.delta_fr(currentStep))/2;
%             obj.angleDir(currentStep) =obj.angleDir(currentStep-1)+obj.speed(currentStep)*tan(obj.steeringAngle(currentStep))/(obj.lengthVehicle/1000);
%              xGlob_np1 = xGlob_n + T*(v_n*cos(beta_n)*cos(Phi_n)-v_n*sin(beta_n)*sin(Phi_n));
%                yGlob_np1 = yGlob_n + T*(v_n*cos(beta_n)*sin(Phi_n)+v_n*sin(beta_n)*cos(Phi_n));
    
            obj.xCG(currentStep)=obj.xCG(currentStep-1) + T*(obj.speed(currentStep)*cos(obj.beta(currentStep))*cos(obj.angleDir(currentStep))-...
                obj.speed(currentStep)*sin(obj.beta(currentStep))*sin(obj.angleDir(currentStep)));
            obj.yCG(currentStep) = obj.yCG(currentStep-1) + T*(obj.speed(currentStep)*cos(obj.beta(currentStep))*sin(obj.angleDir(currentStep))+...
                obj.speed(currentStep)*sin(obj.beta(currentStep))*cos(obj.angleDir(currentStep)));
            
%             if strcmp(obj.type,'EGO')
%                 plot(obj.xCG((currentStep-1):currentStep),obj.yCG((currentStep-1):currentStep),'b-','Linewidth',1.25);
%             else
%                 plot(obj.xCG((currentStep-1):currentStep),obj.yCG((currentStep-1):currentStep),'r-','Linewidth',1.25);
%             end
            
            %% vehicle position update
            
            ex=cos(obj.angleDir(currentStep));%cos((str2double(get(handles.vehicleDiretion,'String')))*pi/180);
            ey=sin(obj.angleDir(currentStep));%sin((str2double(get(handles.vehicleDiretion,'String')))*pi/180);
            exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
            eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
            scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
            x_Data = zeros(4,1);
            y_Data = zeros(4,1);
            
            x_Data(3)=obj.xCG(currentStep)+ex*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)+((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(3)=obj.yCG(currentStep)+ey*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)+((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(4)=obj.xCG(currentStep)+ex*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)-((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(4)=obj.yCG(currentStep)+ey*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)-((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(2)=obj.xCG(currentStep)-ex*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)+((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(2)=obj.yCG(currentStep)-ey*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)+((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(1)=obj.xCG(currentStep)-ex*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)-((obj.widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(1)=obj.yCG(currentStep)-ey*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)-((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            
             obj.xCoordinates(currentStep,:) = [x_Data(1) x_Data(2) x_Data(3) x_Data(4) x_Data(1)];
            obj.yCoordinates(currentStep,:) = [y_Data(1) y_Data(2) y_Data(3) y_Data(4) y_Data(1)];
%             set(obj.handle,'xData',x_Data,'yData',y_Data);
            
           % toc
        end
        
        function updateVehiclePosition(obj,currentStep)
%              ex=cos(obj.angleDir(currentStep));%cos((str2double(get(handles.vehicleDiretion,'String')))*pi/180);
%             ey=sin(obj.angleDir(currentStep));%sin((str2double(get(handles.vehicleDiretion,'String')))*pi/180);
%             exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
%             eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
%             scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
%             x_Data = zeros(4,1);
%             y_Data = zeros(4,1);
%             
%             x_Data(3)=obj.xCG(currentStep)+ex*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)+((obj.widthVehicle)/2000*exOrtho/scalingUnit);
%             y_Data(3)=obj.yCG(currentStep)+ey*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)+((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
%             x_Data(4)=obj.xCG(currentStep)+ex*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)-((obj.widthVehicle)/2000*exOrtho/scalingUnit);
%             y_Data(4)=obj.yCG(currentStep)+ey*(((obj.lf)+(obj.lfToFront))/1000*scalingUnit)-((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
%             x_Data(2)=obj.xCG(currentStep)-ex*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)+((obj.widthVehicle)/2000*exOrtho/scalingUnit);
%             y_Data(2)=obj.yCG(currentStep)-ey*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)+((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
%             x_Data(1)=obj.xCG(currentStep)-ex*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)-((obj.widthVehicle)/2000*exOrtho/scalingUnit);
%             y_Data(1)=obj.yCG(currentStep)-ey*(((obj.lr)+(obj.lrToRear))/1000*scalingUnit)-((obj.widthVehicle)/2000*eyOrtho/scalingUnit);
            
%              obj.xCoordinates(currentStep,:) = [x_Data(1) x_Data(2) x_Data(3) x_Data(4) x_Data(1)];
%             obj.yCoordinates(currentStep,:) = [y_Data(1) y_Data(2) y_Data(3) y_Data(4) y_Data(1)];

            set(obj.handle,'xData',obj.xCoordinates(currentStep,:),'yData',obj.yCoordinates(currentStep,:));
            
        end
        %     figure;
        %     h=axes;
        %     %axis(h,[0,100,0,20]);
        %     axis manual;
        %     hold on;
        %     plot(currentTime,steeringAngleDriver,'b-');
        %     plot(currentTime,delta_fl,'r-');
    end
    
end





