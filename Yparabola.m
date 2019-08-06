

function [new_xRoadPoints new_yRoadPoints slope_endpt] = Yparabola(x1,y1,x2,y2,slope_endpt)
% function for Y-axis parabola
% Inputs:
% x1,y1: coordinates of the first point
% x2,y2: coordinates of the second point
% slope_endpt: required slope at the second point
x(1)=x1;
x(2)=x2;
y(1)=y1;
y(2)=y2;

coef_polys(1,1)= (y(1)^2);
coef_polys(1,2)= y(1);
coef_polys(1,3)= 1;
coef_polys(2,1)= (y(2)^2);
coef_polys(2,2)= y(2);
coef_polys(2,3)= 1;
coef_polys(3,1)=2*y(1);
coef_polys(3,2)=1;

y_value=[x(1);x(2);1/slope_endpt];
%%%%need to check if vertical or horizontal parabola is needed
new_poly=coef_polys\y_value;

new_para_y=linspace(y(1),y(2),100);
new_para_x=polyval(new_poly,new_para_y);
% plot(new_para_y,new_para_y,'r-');

CLF = hypot(diff(new_para_x), diff(new_para_y));    % Calculate integrand from x,y derivatives
CL = trapz(CLF);                          % Integrate to calculate arc length
scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
noPoints = floor(CL/(0.2*scalingUnit));
new_yRoadPoints = linspace(y(1),y(2),noPoints);
new_xRoadPoints = polyval(new_poly,new_yRoadPoints);
pz_der=polyder(new_poly);
slope_endpt=polyval(pz_der,y(2));