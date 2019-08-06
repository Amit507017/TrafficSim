

function [new_xRoadPoints new_yRoadPoints slope_endpt] = XparabolaSpecPoints(x1,y1,x2,y2,slope_endpt,noPoints)

x(1)=x1;
x(2)=x2;
y(1)=y1;
y(2)=y2;

coef_polys(1,1)= (x(1)^2);
coef_polys(1,2)= x(1);
coef_polys(1,3)= 1;
coef_polys(2,1)= (x(2)^2);
coef_polys(2,2)= x(2);
coef_polys(2,3)= 1;
coef_polys(3,1)=2*x(1);
coef_polys(3,2)=1;

y_value=[y(1);y(2);slope_endpt];
%%%%need to check if vertical or horizontal parabola is needed
new_poly=coef_polys\y_value;

new_para_x=linspace(x(1),x(2),100);
new_para_y=polyval(new_poly,new_para_x);
% plot(new_para_x,new_para_y,'r-');

CLF = hypot(diff(new_para_x), diff(new_para_y));    % Calculate integrand from x,y derivatives
CL = trapz(CLF);                          % Integrate to calculate arc length
scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
new_xRoadPoints = linspace(x(1),x(2),noPoints);
new_yRoadPoints = polyval(new_poly,new_xRoadPoints);
pz_der=polyder(new_poly);
slope_endpt=polyval(pz_der,x(2));