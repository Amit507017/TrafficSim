classdef Pedestrian < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        handle;
        xPos;
        yPos;
        angleDir;
        speed;
        accn;
        maxSpeed;
        minSpeed;
        
       % travel Points
       xTravelPoints;
       yTravelPoints;
       
       %for collision detection
        xCoordinates;
        yCoordinates;
        
    end
    
    methods
        %Constructor
        function obj = Pedestrian()
        obj.angleDir = 0;
        obj.speed =10/3.6;
        obj.accn =0.3;
        obj.maxSpeed=15/3.6;
        obj.minSpeed=0/3.6;
        end
        
        function move(obj,xPoints,yPoints,T,currentStep)
%             step = size(obj.xPos);
%             currentStep = step(2)+1;
            size_xPoints = size(xPoints);
            
            for i=1:size_xPoints(2)
                temp= sqrt(((xPoints(i) - obj.xPos(currentStep-1))^2) + (yPoints(i) - obj.yPos(currentStep-1))^2);
                        if i==1
                            dist = temp;
                            if (i+2)>size_xPoints(2)
                                xGoal = xPoints(end);
                                yGoal = yPoints(end);
                            else                                
                                xGoal = xPoints(i+2);
                                yGoal = yPoints(i+2);
                            end
                        elseif temp<dist
                            dist=temp;
                            if (i+2)>size_xPoints(2)
                                xGoal = xPoints(end);
                                yGoal = yPoints(end);
                            else
                                xGoal = xPoints(i+2);
                                yGoal = yPoints(i+2);
                            end
                        end
            end
            
            obj.angleDir(currentStep) = atan2((yGoal-obj.yPos(currentStep-1)),(xGoal-obj.xPos(currentStep-1)));
            
            
           %new Speed
           obj.speed(currentStep) = obj.speed(currentStep-1)+ obj.accn*T;
           distanceCovered = obj.speed(currentStep-1)*T+0.5*obj.accn*T*T;
           
           if obj.speed(currentStep) > obj.maxSpeed 
               obj.speed(currentStep)=obj.maxSpeed;
           elseif  obj.speed(currentStep) < obj.minSpeed
               obj.speed(currentStep)=obj.minSpeed;
           end
           
           %Position update
           ex=cos(obj.angleDir(currentStep-1));
           ey=sin(obj.angleDir(currentStep-1));
           
           obj.xPos(currentStep) = obj.xPos(currentStep-1)+ex*(distanceCovered);
           obj.yPos(currentStep) = obj.yPos(currentStep-1)+ey*(distanceCovered);
           obj.xCoordinates(currentStep,:)=[obj.xPos(currentStep) obj.xPos(currentStep) obj.xPos(currentStep)+0.8 obj.xPos(currentStep)+0.8 obj.xPos(currentStep)];
           obj.yCoordinates(currentStep,:)=[obj.yPos(currentStep) obj.yPos(currentStep)+0.8 obj.yPos(currentStep)+0.8 obj.yPos(currentStep) obj.yPos(currentStep)];
           set(obj.handle,'Position',[obj.xPos(currentStep),obj.yPos(currentStep),0.8,0.8]);
        end
        
        function updatePedestrianPosition(obj,currentStep)
            set(obj.handle,'Position',[obj.xPos(currentStep),obj.yPos(currentStep),0.8,0.8]);
        end
    end       
end

