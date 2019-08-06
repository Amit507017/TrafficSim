
function [xRoadPoints,yRoadPoints] = curvefit()
% h=axes;
% axis(h,[0,50,0,50]);
% axis manual;
% hold on;
% set(gcf,'Pointer','crosshair','doublebuffer','on');
[x,y] = ginput;
sizex=size(x);
% if endpoints of two roads are not near to each other
        init_x=[x(1),x(2),x(3)];
        init_y=[y(1),y(2),y(3)];
        % plot(init_x,init_y,'ro')
        hold on
        if ((init_x(2)> init_x(1)) && (init_x(2) > init_x(3)) || ((init_x(2) < init_x(1) && init_x(2) < init_x(3)))) %% Condition for checking if y-axis parabola is to be drawn
            pz = polyfit(init_y,init_x,2);
            yfit_init = [linspace(init_y(1),init_y(2),100) linspace(init_y(2),init_y(3),100)];
            xfit_init = polyval(pz,yfit_init);
            CLF = hypot(diff(xfit_init), diff(yfit_init));    % Calculate integrand from x,y derivatives
            CL = trapz(CLF);                          % Integrate to calculate arc length
            scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
            noPoints = floor(CL/(0.2*scalingUnit));
            yRoadPoints = linspace(y(1),y(3),noPoints);
            xRoadPoints = polyval(pz,yRoadPoints);
            pz_der=polyder(pz);
            slope_endpt=polyval(pz_der,y(3));
            endPointsSlope(1) = polyval(pz_der,x(1));
            
            %roadDatabase(1,sizeRoadDatabase(2)+1).endPointsSlope(1) = polyval(pz_der,y(1));
            y_parabola=1;
            x_parabola=0;
        else
            xfit_init = [linspace(init_x(1),init_x(2),100) linspace(init_x(2),init_x(3),100)] ;
            pz = polyfit(init_x,init_y,2);
            yfit_init = polyval(pz,xfit_init);
            CLF = hypot(diff(xfit_init), diff(yfit_init));    % Calculate integrand from x,y derivatives
            CL = trapz(CLF);                          % Integrate to calculate arc length
            scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
            noPoints = floor(CL/(0.2*scalingUnit));
            xRoadPoints = linspace(x(1),x(3),noPoints);
            yRoadPoints = polyval(pz,xRoadPoints);
            pz_der=polyder(pz);
            slope_endpt=polyval(pz_der,x(3));
            endPointsSlope(1) = polyval(pz_der,x(1));
            x_parabola = 1;
            y_parabola=0;
        end
        
        %plot(xfit_init,yfit_init,'b-')
        hold on;
        
        xfit=xfit_init;
        yfit=yfit_init;
        
        % Drawing a new parabola for next point and doing this for rest of the points
        coef_polys = zeros(3,3);
        y_value = zeros(3,1);
        new_coef = zeros(3,1);
        new_poly=pz;
        
        
        for i=1:(sizex(1)-3)
            if ((x(i+2)>x(i+1) && x(i+2)>x(i+3)) || (x(i+2)<x(i+1) && x(i+2)<x(i+3)))
                %          if(((x(i+2)>x(i+1) && x(i+2)>x(i+3)) || (x(i+2)<x(i+1) && x(i+2)<x(i+3))) && (y(i+3)>y(i+2)))
                
                slope_endpt = 1/slope_endpt;
                x_parabola=0;
                y_parabola = 1;
                coef_polys(1,1)= (y(i+2)^2);
                coef_polys(1,2)= y(i+2);
                coef_polys(1,3)= 1;
                coef_polys(2,1)= (y(i+3)^2);
                coef_polys(2,2)= y(i+3);
                coef_polys(2,3)= 1;
                coef_polys(3,1)=2*y(i+2);
                coef_polys(3,2)=1;
                
                x_value=[x(i+2);x(i+3);slope_endpt];
                new_poly=coef_polys\x_value;
                
                new_para_y=linspace(y(i+2),y(i+3),100);
                new_para_x=polyval(new_poly,new_para_y);
                % plot(new_para_x,new_para_y,'r-');
                CLF = hypot(diff(new_para_x), diff(new_para_y));    % Calculate integrand from x,y derivatives
                CL = trapz(CLF);                          % Integrate to calculate arc length
                scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
                noPoints = floor(CL/(0.2*scalingUnit));
                new_yRoadPoints = linspace(y(i+2),y(i+3),noPoints);
                new_xRoadPoints = polyval(new_poly,new_yRoadPoints);
                yRoadPoints = [yRoadPoints new_yRoadPoints];
                xRoadPoints = [xRoadPoints new_xRoadPoints];
                xfit=[xfit new_para_x];
                yfit=[yfit new_para_y];
                pz_der=polyder(new_poly);
                slope_endpt=polyval(pz_der,y(i+3));
                endPointsSlope(2) =1/ polyval(pz_der,y(i+3));
                %  plot(xRoadPoints,yRoadPoints,'go');
            else
                if (y_parabola == 1)
                    slope_endpt = 1/slope_endpt;
                    
                end
                x_parabola = 1;
                y_parabola=0;
                coef_polys(1,1)= (x(i+2)^2);
                coef_polys(1,2)= x(i+2);
                coef_polys(1,3)= 1;
                coef_polys(2,1)= (x(i+3)^2);
                coef_polys(2,2)= x(i+3);
                coef_polys(2,3)= 1;
                coef_polys(3,1)=2*x(i+2);
                coef_polys(3,2)=1;
                
                y_value=[y(i+2);y(i+3);slope_endpt];
                %%%%need to check if vertical or horizontal parabola is needed
                new_poly=coef_polys\y_value;
                
                new_para_x=linspace(x(i+2),x(i+3),100);
                new_para_y=polyval(new_poly,new_para_x);
                % plot(new_para_x,new_para_y,'r-');
                
                CLF = hypot(diff(new_para_x), diff(new_para_y));    % Calculate integrand from x,y derivatives
                CL = trapz(CLF);                          % Integrate to calculate arc length
                scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
                noPoints = floor(CL/(0.2*scalingUnit));
                new_xRoadPoints = linspace(x(i+2),x(i+3),noPoints);
                new_yRoadPoints = polyval(new_poly,new_xRoadPoints);
                yRoadPoints = [yRoadPoints new_yRoadPoints];
                xRoadPoints = [xRoadPoints new_xRoadPoints];
                xfit=[xfit new_para_x];
                yfit=[yfit new_para_y];
                pz_der=polyder(new_poly);
                slope_endpt=polyval(pz_der,x(i+3));
                endPointsSlope(2) = polyval(pz_der,x(i+3));
                % plot(xRoadPoints,yRoadPoints,'r-');
            end
            
        end