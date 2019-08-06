function simplesim()
% Clear all the windows and the variables
close all;clc;
clear -global;

% Variable Declaration
% Global variables
global vehicleDatabase;
global pedestrianDatabase;
global bicycleDatabase;
global roadDatabase;
global kreuzungDatabase;
global stationaryObjectDatabase;
global kreiselDatabase;


% Local variables
size_roadDatabase = size(roadDatabase);
size_vehicleDatabase = size(vehicleDatabase);
size_kreuzungDatabase = size(kreuzungDatabase);
size_pedestrianDatabase = size(pedestrianDatabase);
size_bicycleDatabase=size(bicycleDatabase);
size_objectDatabase=size(stationaryObjectDatabase);
m_simulationtime=3;
m_simulationstep=0.02;
global currentHandle;
colergen = @(color,text) ['<table border=0 width=400 bgcolor=',color,'><TR><TD>',text,'</TD></TR> </table>'];
n=1; % no of road tracks

%-------------------------------------------------------------------------
%---------------   GUI Creation ------------------------------------------
%-------------------------------------------------------------------------

%-----------------------------------------------------------------------
% GUI window and the main plot creation
%  Create and then hide the UI as it is being constructed.
m_MainWindow = figure('Visible','off','Position',[360,500,1600,1200]);
m_PlotWindow = axes('Visible','on','Units','pixels','Position',[50,50,1500,1050],'xlim',([0 200]),'ylim',([0 200]));


% Initialize the UI.
% Change units to normalized so components resize automatically.
m_MainWindow.Units = 'normalized';
m_PlotWindow.Units = 'normalized';


% Assign the a name to appear in the window title.
m_MainWindow.Name = 'Simple Sim';

% Move the window to the center of the screen.
movegui(m_MainWindow,'center')

% Make the window visible.
m_MainWindow.Visible = 'on';

%-----------------------------------------------------------------------
% Buttons and User interface

m_savescenario = uicontrol('Style','pushbutton',...
    'String','Save','Position',[50,930,100,40],...
    'Callback',@m_savescenarioCallback);

m_loadscenario = uicontrol('Style','pushbutton',...
    'String','Load','Position',[150,930,100,40],...
    'Callback',@m_loadscenarioCallback);

m_clearscenario = uicontrol('Style','pushbutton',...
    'String','Clear','Position',[250,930,100,40],...
    'Callback',@m_clearscenarioCallback);

m_playscenario=uicontrol('Style','pushbutton',...
    'String','Play','Position',[350,930,100,40],...
    'Callback',@m_playscenarioCallback);


m_SimTime=uicontrol('Style','text','String','Sim. Time','Position',[520,920,80,40]);
m_simulationtime=uicontrol('Style','edit','String','3','Position',[600,930,50,40],'CallBack',@m_simulationtimeCall);
m_StepsTime=uicontrol('Style','text','String','Step size','Position',[660,920,80,40]);
m_simulationstepstime=uicontrol('Style','edit','String','0.02','Position',[740,930,50,40],'CallBack',@m_simulationstepCall);
m_simulatescene=uicontrol('Style','pushbutton',...
    'String','Simulate','Position',[800,930,100,40],...
    'Callback',@m_startsimulationCallback);


m_savescenario.Units = 'normalized';
m_loadscenario.Units = 'normalized';
m_clearscenario.Units = 'normalized';
m_playscenario.Units = 'normalized';
m_simulationtime.Units ='normalized';
m_SimTime.Units ='normalized';
m_StepsTime.Units ='normalized';
m_simulationstepstime.Units ='normalized';
m_simulatescene.Units ='normalized';
%----------------------------------------------------------------------
% Pop-up buttons

% Pop-up menus

cmenu=uicontextmenu('Parent',m_MainWindow,'Position',[10 215]);
vh1=uimenu(cmenu,'Text','New vehicle','Callback',@vehicleParametersMenu_Callback);
ph1=uimenu(cmenu,'Text','New pedestrian','Callback',@pedestrianParametersMenu_Callback);
bh1=uimenu(cmenu,'Text','New bicycle','Callback',@bicycleParametersMenu_Callback);
oh1=uimenu(cmenu,'Text','New object','Callback',@objectParametersMenu_Callback);
m_nRoad = uimenu(cmenu,'Label','New Road','Callback',@m_newRoadCreate);
m_nScenario = uimenu(cmenu,'Label','Def scene','Callback',@m_newScenario);
m_PlotWindow.UIContextMenu=cmenu;
% -------------------------------------------------------------------------
% Button groups
%--------------------------------------------------------------------------
% Vehicle parameters Start-------------------------------------------------
vP = uibuttongroup('Parent',m_MainWindow,'Visible','off','Position',[0.8 0.61 1 1]);
%Title
Vt_title=uicontrol(vP,'Style','text','String','Vehicle parameters','Position',[2 280 150 100],...
    'FontWeight','bold','FontSize',12);
%Left row
Vt_speed=uicontrol(vP,'Style','text','String','Speed (m/s)','Position',[5 330 90 20]);
Vv_speed=uicontrol(vP,'Style','edit','Position',[5 300 90 20]);
Vt_long_accel=uicontrol(vP,'Style','text','String',['Long.Accn. (m/s', char(178),')'],'Position',[5 250 90 20]);
Vv_long_accel=uicontrol(vP,'Style','edit','Position',[5 220 90 20]);
Vt_lat_accel=uicontrol(vP,'Style','text','String',['Lat.Accn. (m/s', char(178),')'],'Position',[5 170 90 20]);
Vv_lat_accel=uicontrol(vP,'Style','edit','Position',[5 140 90 20]);
Vt_angle_dir=uicontrol(vP,'Style','text','String','Angle direction (°)','Position',[5 90 90 20]);
Vv_angle_dir=uicontrol(vP,'Style','edit','Position',[5 60 90 20]);
%Centre row
Vt_mass=uicontrol(vP,'Style','text','String','Vehicle mass (kg)','Position',[100 330 90 20]);
Vv_mass=uicontrol(vP,'Style','edit','Position',[100 300 90 20]);
Vt_height=uicontrol(vP,'Style','text','String','CG Height (mm)','Position',[100 250 90 20]);
Vv_height=uicontrol(vP,'Style','edit','Position',[100 220 90 20]);
Vt_length=uicontrol(vP,'Style','text','String','Length (mm)','Position',[100 170 90 20]);
Vv_length=uicontrol(vP,'Style','edit','Position',[100 140 90 20]);
Vt_width=uicontrol(vP,'Style','text','String','Width (mm)','Position',[100 90 90 20]);
Vv_width=uicontrol(vP,'Style','edit','Position',[100 60 90 20]);
%Right row
Vt_lf=uicontrol(vP,'Style','text','String','lf (mm)','Position',[195 330 90 20]);
Vv_lf=uicontrol(vP,'Style','edit','Position',[195 300 90 20]);
Vt_lr=uicontrol(vP,'Style','text','String','lr (mm)','Position',[195 250 90 20]);
Vv_lr=uicontrol(vP,'Style','edit','Position',[195 220 90 20]);
Vt_lf_to_front=uicontrol(vP,'Style','text','String','lf to front (mm)','Position',[195 170 90 20]);
Vv_lf_to_front=uicontrol(vP,'Style','edit','Position',[195 140 90 20]);
Vt_lr_to_rear=uicontrol(vP,'Style','text','String','lr to rear (mm)','Position',[195 90 90 20]);
Vv_lr_to_rear=uicontrol(vP,'Style','edit','Position',[195 60 90 20]);
%Buttons
Vv_button_new = uicontrol(vP,'Style','pushbutton','String','New','Position',[5 10 90 30],...
    'Callback',@m_newVehicleCallback);
Vv_button_save = uicontrol(vP,'Style','pushbutton','String','Save','Position',[100 10 90 30],...
    'Callback',@m_saveVehicle_Callback);
Vv_button_reset = uicontrol(vP,'Style','pushbutton','String','Default values','Position',[195 10 90 30],...
    'Callback',@m_defaultVehicle_Callback);
%Normalized
Vt_title.Units = 'normalized';Vt_speed.Units = 'normalized';Vv_speed.Units = 'normalized';
Vt_long_accel.Units = 'normalized';Vv_long_accel.Units = 'normalized';
Vt_lat_accel.Units = 'normalized';Vv_lat_accel.Units = 'normalized';
Vt_angle_dir.Units = 'normalized';Vv_angle_dir.Units = 'normalized';
Vt_mass.Units = 'normalized';Vv_mass.Units = 'normalized';
Vt_height.Units = 'normalized';Vv_height.Units = 'normalized';
Vt_length.Units = 'normalized';Vv_length.Units = 'normalized';
Vt_width.Units = 'normalized';Vv_width.Units = 'normalized';
Vt_lf.Units = 'normalized';Vv_lf.Units = 'normalized';
Vt_lr.Units = 'normalized';Vv_lr.Units = 'normalized';
Vt_lf_to_front.Units = 'normalized';Vv_lf_to_front.Units = 'normalized';
Vt_lr_to_rear.Units = 'normalized';Vv_lr_to_rear.Units = 'normalized';
Vv_button_new.Units='normalized';Vv_button_save.Units='normalized';Vv_button_reset.Units='normalized';
Vt_mass.Units = 'normalized';Vv_mass.Units = 'normalized';
% Vehicle parameters End---------------------------------------------------
% Pedestrian parameters Start----------------------------------------------
pP = uibuttongroup('Parent',m_MainWindow,'Visible','off','Position',[0.8 0.78 1 1]);
%Title
Pt_title=uicontrol(pP,'Style','text','String','Pedestrian parameters','Position',[2 110 180 100],...
    'FontWeight','bold','FontSize',12);
%Left row
Pt_speed=uicontrol(pP,'Style','text','String','Speed (m/s)','Position',[5 160 90 20]);
Pv_speed=uicontrol(pP,'Style','edit','Position',[5 130 90 20]);
Pt_max_speed=uicontrol(pP,'Style','text','String','Max. speed (m/s)','Position',[5 80 90 20]);
Pv_max_speed=uicontrol(pP,'Style','edit','Position',[5 50 90 20]);

%Centre row
Pt_accn=uicontrol(pP,'Style','text','String',['Accn. (m/s', char(178),')'],'Position',[100 160 90 20]);
Pv_accn=uicontrol(pP,'Style','edit','Position',[100 130 90 20]);
Pt_min_speed=uicontrol(pP,'Style','text','String','Min. speed (m/s)','Position',[100 80 90 20]);
Pv_min_speed=uicontrol(pP,'Style','edit','Position',[100 50 90 20]);
%Right row
Pt_direction=uicontrol(pP,'Style','text','String','Direction (°)','Position',[195 160 90 20]);
Pv_direction=uicontrol(pP,'Style','edit','Position',[195 130 90 20]);
%Buttons
Pv_button_new = uicontrol(pP,'Style','pushbutton','String','New','Position',[5 10 90 30],...
    'Callback',@m_newpedestrianCallback);
Pv_button_save = uicontrol(pP,'Style','pushbutton','String','Save','Position',[100 10 90 30],...
    'Callback',@m_savePedestrian_Callback);
Pv_button_reset = uicontrol(pP,'Style','pushbutton','String','Default values','Position',[195 10 90 30],...
    'Callback',@m_defaultPedestrian_Callback);
%Normalized
Pt_title.Units = 'normalized';Pt_speed.Units = 'normalized';Pv_speed.Units = 'normalized';
Pt_max_speed.Units = 'normalized';Pv_max_speed.Units = 'normalized';
Pt_min_speed.Units = 'normalized';Pv_min_speed.Units = 'normalized';
Pt_accn.Units = 'normalized';Pv_accn.Units = 'normalized';
Pt_direction.Units = 'normalized';Pv_direction.Units = 'normalized';
Pv_button_new.Units='normalized';Pv_button_save.Units='normalized';Pv_button_reset.Units='normalized';
% Pedestrian parameters End------------------------------------------------
% Bicycle parameters Start-------------------------------------------------
bP = uibuttongroup('Parent',m_MainWindow,'Visible','off','Position',[0.8 0.78 1 1]);
%Title
Bt_title=uicontrol(bP,'Style','text','String','Bicycle parameters','Position',[2 110 150 100],...
    'FontWeight','bold','FontSize',12);
%Left row
Bt_speed=uicontrol(bP,'Style','text','String','Speed (m/s)','Position',[5 160 90 20]);
Bv_speed=uicontrol(bP,'Style','edit','Position',[5 130 90 20]);
Bt_direction=uicontrol(bP,'Style','text','String','Direction (°)','Position',[5 80 90 20]);
Bv_direction=uicontrol(bP,'Style','edit','Position',[5 50 90 20]);
%Centre row
Bt_long_accn=uicontrol(bP,'Style','text','String',['Long.Accn. (m/s', char(178),')'],'Position',[100 160 90 20]);
Bv_long_accn=uicontrol(bP,'Style','edit','Position',[100 130 90 20]);

%Right row
Bt_lat_accn=uicontrol(bP,'Style','text','String',['Lat.Accn. (m/s', char(178),')'],'Position',[195 160 90 20]);
Bv_lat_accn=uicontrol(bP,'Style','edit','Position',[195 130 90 20]);

%Buttons
Bv_button_new = uicontrol(bP,'Style','pushbutton','String','New','Position',[5 10 90 30],...
    'Callback',@newBicycle_Callback);
Bv_button_save = uicontrol(bP,'Style','pushbutton','String','Save','Position',[100 10 90 30],...
    'Callback',@m_saveBicycle_Callback);
Bv_button_reset = uicontrol(bP,'Style','pushbutton','String','Default values','Position',[195 10 90 30],...
    'Callback',@m_defaultBicycle_Callback);
%Normalized
Bt_title.Units = 'normalized';Bt_speed.Units = 'normalized';Bv_speed.Units = 'normalized';
Bt_direction.Units = 'normalized';Bv_direction.Units = 'normalized';
Bt_long_accn.Units = 'normalized';Bv_long_accn.Units = 'normalized';
Bt_lat_accn.Units = 'normalized';Bv_lat_accn.Units = 'normalized';
Bv_button_new.Units='normalized';Bv_button_save.Units='normalized';Bv_button_reset.Units='normalized';
% Bicycle parameters End---------------------------------------------------
% Object parameters Start-------------------------------------------------
oP = uibuttongroup('Parent',m_MainWindow,'Visible','off','Position',[0.8 0.85 1 1]);
%Title
Ot_title=uicontrol(oP,'Style','text','String','Object parameters','Position',[2 40 150 100],...
    'FontWeight','bold','FontSize',12);
%Left row
Ot_length=uicontrol(oP,'Style','text','String','Length (m)','Position',[5 90 90 20]);
Ov_length=uicontrol(oP,'Style','edit','Position',[5 60 90 20]);

%Centre row
Ot_breadth=uicontrol(oP,'Style','text','String','Breadth (m)','Position',[100 90 90 20]);
Ov_breadth=uicontrol(oP,'Style','edit','Position',[100 60 90 20]);

%Right row
Ot_angle=uicontrol(oP,'Style','text','String','Angle (°)','Position',[195 90 90 20]);
Ov_angle=uicontrol(oP,'Style','edit','Position',[195 60 90 20]);

%Buttons
Ov_button_new = uicontrol(oP,'Style','pushbutton','String','New','Position',[5 10 90 30],...
    'Callback',@m_newObjectCallback);
Ov_button_save = uicontrol(oP,'Style','pushbutton','String','Save','Position',[100 10 90 30],...
    'Callback',@m_saveObject_Callback);
Ov_button_reset = uicontrol(oP,'Style','pushbutton','String','Default values','Position',[195 10 90 30],...
    'Callback',@m_defaultObject_Callback);

%Normalized
Ot_title.Units = 'normalized';Ot_length.Units = 'normalized';Ov_length.Units = 'normalized';
Ot_breadth.Units = 'normalized';Ov_breadth.Units = 'normalized';
Ot_angle.Units = 'normalized';Ov_angle.Units = 'normalized';
Ov_button_new.Units='normalized';Ov_button_save.Units='normalized';Ov_button_reset.Units='normalized';

% Scenario define
m_nscene = uibuttongroup('Parent',m_MainWindow,'Visible','off','Position',[0.8 0.61 1 1]);
%Title
m_nscene_title=uicontrol(m_nscene,'Style','text','String','Scenario Define','Position',[2 285 150 100],...
    'FontWeight','bold','FontSize',12);
m_nscenetable=uitable(m_nscene,'ColumnName',{'Object'; 'Type'},'CellSelectionCallback',{@cell_select_callback},'Position',[10 55 300 300]);
m_selecttrack = uicontrol(m_nscene,'Style','pushbutton','String','Defined','Position',[10 10 90 30],...
    'Callback',@m_selectTrackCallback);
m_manualtrack = uicontrol(m_nscene,'Style','pushbutton','String','Manual','Position',[110 10 90 30],...
    'Callback',@defineManualPath_Callback);
m_cleartrack = uicontrol(m_nscene,'Style','pushbutton','String','Clwar','Position',[210 10 90 30],...
    'Callback',@clearPath_Callback);

m_nscene.Units = 'normalized';
m_nscene_title.Units = 'normalized';
m_nscenetable.Units = 'normalized';
m_selecttrack.Units = 'normalized';
m_manualtrack.Units = 'normalized';
m_cleartrack.Units = 'normalized';
% Object parameters End---------------------------------------------------
m_MainWindow.Units = 'normalized';
m_PlotWindow.Units = 'normalized';

% ---------------------------------------------------------------------
% Panels

% ---------------------------------------------------------------------
% ------------------- Callbacks ---------------------------------------
% ---------------------------------------------------------------------
% Context menu callbacks
% Define the scenario callback
    function cell_select_callback(src, eventdata)
        size_vehicleDatabase = size(vehicleDatabase);
        size_roadDatabase = size(roadDatabase);
        size_pedestrianDatabase = size(pedestrianDatabase);
        size_bicycleDatabase=size(bicycleDatabase);
        
        
        Indices=nan(1,2);
        data='';
        if ~isempty(eventdata.Indices)
            handles.currentCell=eventdata.Indices;
            
            Indices=handles.currentCell;
            data=get(m_nscenetable,'Data');
            data=data(Indices(1),2);
        end
        if strcmp(data,'Vehile')
            for j=1:size_vehicleDatabase(2)
                if (j==Indices(1))
                    currentHandle=vehicleDatabase(1,j).handle;
                    set(vehicleDatabase(1,j).handle,'Facecolor','y');
                elseif strcmp(vehicleDatabase(1,j).type,'EGO')
                    set(vehicleDatabase(1,j).handle,'Facecolor','b');
                else
                    set(vehicleDatabase(1,j).handle,'Facecolor','r');
                    
                end
            end
        elseif strcmp(data,'Pedestrian')
            
            for j=1:size_pedestrianDatabase(2)
                if (j+size_vehicleDatabase(2)==Indices(1))
                    currentHandle=pedestrianDatabase(1,j).handle;
                    set(pedestrianDatabase(1,j).handle,'Facecolor','y');
                else
                    set(pedestrianDatabase(1,j).handle,'Facecolor','b');
                end
            end
        elseif strcmp(data,'Bicycle')
            for j=1:size_bicycleDatabase(2)
                if (j+size_vehicleDatabase(2)+size_pedestrianDatabase(2)==Indices(1))
                    currentHandle=bicycleDatabase(1,j).handle;
                    set(bicycleDatabase(1,j).handle,'Facecolor','y');
                else
                    set(bicycleDatabase(1,j).handle,'Facecolor','m');
                end
            end
        end
        
        
        
        
    end
% Clear track
    function clearPath_Callback(src, eventdata)
        size_vehicleDatabase = size(vehicleDatabase);
        size_roadDatabase = size(roadDatabase);
        size_pedestrianDatabase = size(pedestrianDatabase);
        size_bicycleDatabase=size(bicycleDatabase);
        for i=1:size_vehicleDatabase(2)
            vehicleDatabase(1,i).trackHandles = [];
            vehicleDatabase(1,i).xTravelPoints = [];
            vehicleDatabase(1,i).yTravelPoints = [];
        end
        for i=1:size_pedestrianDatabase(2)
            pedestrianDatabase(1,i).xTravelPoints = [];
            pedestrianDatabase(1,i).yTravelPoints = [];
        end
        for i=1:size_bicycleDatabase(2)
            bicycleDatabase(1,i).trackHandles = [];
            bicycleDatabase(1,i).xTravelPoints = [];
            bicycleDatabase(1,i).yTravelPoints = [];
        end
        %         m_nscenetable.BackgroundColor=[];
        m_nscenetable.BackgroundColor=[1 1 1];
    end
% Select the tracks
    function m_selectTrackCallback(src, eventdata)
        size_vehicleDatabase = size(vehicleDatabase);
        size_roadDatabase = size(roadDatabase);
        size_pedestrianDatabase = size(pedestrianDatabase);
        size_bicycleDatabase=size(bicycleDatabase);
        
        isVehicle=0;
        isBicycle=0;
        noVehicle=0;
        for i=1:size_vehicleDatabase(2)
            if vehicleDatabase(1,i).handle==currentHandle
                noVehicle = i;
                isVehicle = 1;
                break;
            end
        end
        
        for i=1:size_bicycleDatabase(2)
            if bicycleDatabase(1,i).handle==currentHandle
                noBicycle = i;
                isBicycle =1;
                break;
            end
        end
        if isVehicle
            %clear all tracks
            vehicleDatabase(1,noVehicle).trackHandles = [];
            vehicleDatabase(1,noVehicle).xTravelPoints = [];
            vehicleDatabase(1,noVehicle).yTravelPoints = [];
            k=0;
            k=waitforbuttonpress;
            
            while k==0
                
                if k==1
                    break;
                end
                
                size_trackHandles = size(vehicleDatabase(1,noVehicle).trackHandles);
                vehicleDatabase(1,noVehicle).trackHandles(size_trackHandles(2)+1) = gco;
                set(gco,'LineWidth',1.5);
                
                %finding of which road the track is part of and then assign the points
                %of this track as travel path for selected vehicle
                for i=1:size_roadDatabase(2)
                    size_innerTracks = size(roadDatabase(1,i).innerTrackHandles);
                    size_outerTracks = size(roadDatabase(1,i).outerTrackHandles);
                    for j=1:size_innerTracks(2)
                        if gco == roadDatabase(1,i).innerTrackHandles(j)
                            vehicleDatabase(1,noVehicle).xTravelPoints=[vehicleDatabase(1,noVehicle).xTravelPoints roadDatabase(1,i).xInnerCurveTrackPoints(j,:)];
                            vehicleDatabase(1,noVehicle).yTravelPoints=[vehicleDatabase(1,noVehicle).yTravelPoints roadDatabase(1,i).yInnerCurveTrackPoints(j,:)];
                        end
                    end
                    
                    for j=1:size_outerTracks(2)
                        if gco == roadDatabase(1,i).outerTrackHandles(j)
                            vehicleDatabase(1,noVehicle).xTravelPoints=[vehicleDatabase(1,noVehicle).xTravelPoints roadDatabase(1,i).xOuterCurveTrackPoints(j,:)];
                            vehicleDatabase(1,noVehicle).yTravelPoints=[vehicleDatabase(1,noVehicle).yTravelPoints roadDatabase(1,i).yOuterCurveTrackPoints(j,:)];
                        end
                    end
                end
                
                for i=1:size_kreuzungDatabase(2)
                    size_noRoads = size(kreuzungDatabase(1,i).noRoadToTracks);
                    
                    for j=1:size_noRoads(2)
                        for k=1:kreuzungDatabase(1,i).noRoadToTracks(j)
                            if gco == kreuzungDatabase(1,i).innerRoadToTracksHandles(j,k)
                                num = size(vehicleDatabase(1,noVehicle).xTrafficSignalPoints);
                                vehicleDatabase(1,noVehicle).xTrafficSignalPoints(num+1) = kreuzungDatabase(1,i).XInnerRoadToTracksPoints(j,k,end);
                                vehicleDatabase(1,noVehicle).yTrafficSignalPoints(num+1) = kreuzungDatabase(1,i).YInnerRoadToTracksPoints(j,k,end);
                                vehicleDatabase(1,noVehicle).trafficSignalHandles(num+1) = kreuzungDatabase(1,i).trafficSignal(j,k);
                                vehicleDatabase(1,noVehicle).xTravelPoints=[vehicleDatabase(1,noVehicle).xTravelPoints (reshape(kreuzungDatabase(1,i).XInnerRoadToTracksPoints(j,k,:),[],1))'];
                                vehicleDatabase(1,noVehicle).yTravelPoints=[vehicleDatabase(1,noVehicle).yTravelPoints (reshape(kreuzungDatabase(1,i).YInnerRoadToTracksPoints(j,k,:),[],1))'];
                            end
                        end
                    end
                    
                    for j=1:size_noRoads(2)
                        for k=1:kreuzungDatabase(1,i).noRoadAwayTracks(j)
                            if gco == kreuzungDatabase(1,i).innerRoadAwayTracksHandles(j,k)
                                num = size(vehicleDatabase(1,noVehicle).xTrafficSignalPoints);
                                vehicleDatabase(1,noVehicle).xTrafficSignalPoints(num+1) = kreuzungDatabase(1,i).XInnerRoadToTracksPoints(j,k,1);
                                vehicleDatabase(1,noVehicle).yTrafficSignalPoints(num+1) = kreuzungDatabase(1,i).YInnerRoadToTracksPoints(j,k,1);
                                vehicleDatabase(1,noVehicle).xTravelPoints=[vehicleDatabase(1,noVehicle).xTravelPoints (reshape(kreuzungDatabase(1,i).XInnerRoadAwayTracksPoints(j,k,:),[],1))'];
                                vehicleDatabase(1,noVehicle).yTravelPoints=[vehicleDatabase(1,noVehicle).yTravelPoints (reshape(kreuzungDatabase(1,i).YInnerRoadAwayTracksPoints(j,k,:),[],1))'];
                            end
                        end
                    end
                    
                    
                    noInternalRoads = size(kreuzungDatabase(1,i).roadJoiningHandles);
                    for j=1:noInternalRoads(2)
                        if gco == kreuzungDatabase(1,i).roadJoiningHandles(j)
                            new_poly = polyfit(kreuzungDatabase(1,i).xRoadJoiningPoints(j,:),kreuzungDatabase(1,i).yRoadJoiningPoints(j,:),2);
                            CLF = hypot(diff(kreuzungDatabase(1,i).xRoadJoiningPoints(j,:)), diff(kreuzungDatabase(1,i).yRoadJoiningPoints(j,:)));    % Calculate integrand from x,y derivatives
                            CL = trapz(CLF);                          % Integrate to calculate arc length
                            scalingUnit = 1;
                            noPoints = floor(CL/(0.2*scalingUnit));
                            new_xRoadPoints = linspace(kreuzungDatabase(1,i).xRoadJoiningPoints(j,1),kreuzungDatabase(1,i).xRoadJoiningPoints(j,end),noPoints);
                            new_yRoadPoints = polyval(new_poly,new_xRoadPoints);
                            vehicleDatabase(1,noVehicle).xTravelPoints=[vehicleDatabase(1,noVehicle).xTravelPoints new_xRoadPoints];
                            vehicleDatabase(1,noVehicle).yTravelPoints=[vehicleDatabase(1,noVehicle).yTravelPoints new_yRoadPoints];
                        end
                    end
                    
                end
                if(~isempty(vehicleDatabase(1,noVehicle).xTravelPoints))
                    %m_nscenetable.BackgroundColor=[0.85 0.85 1];
                    for m=1:size_vehicleDatabase(2)+size_pedestrianDatabase(2)+size_bicycleDatabase(2)
                        if(m==noVehicle)
                            m_nscenetable.BackgroundColor(noVehicle,:) = [.4 .4 .8];
                        end
                    end
                    %                       m_nscenetable.BackgroundColor(noVehicle,:) = [.4 .4 .8];
                    k=1;
                end
                %                   k=waitforbuttonpress;
                
            end
            
        elseif isBicycle
            
            %clear all tracks
            bicycleDatabase(1,noBicycle).trackHandles = [];
            bicycleDatabase(1,noBicycle).xTravelPoints = [];
            bicycleDatabase(1,noBicycle).yTravelPoints = [];
            k=0;
            k=waitforbuttonpress;
            
            while k==0
                
                if k==1
                    break;
                end
                
                size_trackHandles = size(bicycleDatabase(1,noBicycle).trackHandles);
                bicycleDatabase(1,noBicycle).trackHandles(size_trackHandles(2)+1) = gco;
                set(gco,'LineWidth',1.5);
                
                %finding of which road the track is part of and then assign the points
                %of this track as travel path for selected vehicle
                for i=1:size_roadDatabase(2)
                    size_innerTracks = size(roadDatabase(1,i).innerTrackHandles);
                    size_outerTracks = size(roadDatabase(1,i).outerTrackHandles);
                    for j=1:size_innerTracks(2)
                        if gco == roadDatabase(1,i).innerTrackHandles(j)
                            bicycleDatabase(1,noBicycle).xTravelPoints=[bicycleDatabase(1,noBicycle).xTravelPoints roadDatabase(1,i).xInnerCurveTrackPoints(j,:)];
                            bicycleDatabase(1,noBicycle).yTravelPoints=[bicycleDatabase(1,noBicycle).yTravelPoints roadDatabase(1,i).yInnerCurveTrackPoints(j,:)];
                        end
                    end
                    
                    for j=1:size_outerTracks(2)
                        if gco == roadDatabase(1,i).outerTrackHandles(j)
                            bicycleDatabase(1,noBicycle).xTravelPoints=[bicycleDatabase(1,noBicycle).xTravelPoints roadDatabase(1,i).xOuterCurveTrackPoints(j,:)];
                            bicycleDatabase(1,noBicycle).yTravelPoints=[bicycleDatabase(1,noBicycle).yTravelPoints roadDatabase(1,i).yOuterCurveTrackPoints(j,:)];
                        end
                    end
                end
                
                for i=1:size_kreuzungDatabase(2)
                    size_noRoads = size(kreuzungDatabase(1,i).noRoadToTracks);
                    
                    for j=1:size_noRoads(2)
                        for k=1:kreuzungDatabase(1,i).noRoadToTracks(j)
                            if gco == kreuzungDatabase(1,i).innerRoadToTracksHandles(j,k)
                                num = size(bicycleDatabase(1,noBicycle).xTrafficSignalPoints);
                                bicycleDatabase(1,noBicycle).xTrafficSignalPoints(num+1) = kreuzungDatabase(1,i).XInnerRoadToTracksPoints(j,k,end);
                                bicycleDatabase(1,noBicycle).yTrafficSignalPoints(num+1) = kreuzungDatabase(1,i).YInnerRoadToTracksPoints(j,k,end);
                                bicycleDatabase(1,noBicycle).trafficSignalHandles(num+1) = kreuzungDatabase(1,i).trafficSignal(j,k);
                                bicycleDatabase(1,noBicycle).xTravelPoints=[bicycleDatabase(1,noBicycle).xTravelPoints (reshape(kreuzungDatabase(1,i).XInnerRoadToTracksPoints(j,k,:),[],1))'];
                                bicycleDatabase(1,noBicycle).yTravelPoints=[bicycleDatabase(1,noBicycle).yTravelPoints (reshape(kreuzungDatabase(1,i).YInnerRoadToTracksPoints(j,k,:),[],1))'];
                            end
                        end
                    end
                    
                    for j=1:size_noRoads(2)
                        for k=1:kreuzungDatabase(1,i).noRoadAwayTracks(j)
                            if gco == kreuzungDatabase(1,i).innerRoadAwayTracksHandles(j,k)
                                num = size(bicycleDatabase(1,noBicycle).xTrafficSignalPoints);
                                bicycleDatabase(1,noBicycle).xTrafficSignalPoints(num+1) = kreuzungDatabase(1,i).XInnerRoadToTracksPoints(j,k,1);
                                bicycleDatabase(1,noBicycle).yTrafficSignalPoints(num+1) = kreuzungDatabase(1,i).YInnerRoadToTracksPoints(j,k,1);
                                bicycleDatabase(1,noBicycle).xTravelPoints=[bicycleDatabase(1,noBicycle).xTravelPoints (reshape(kreuzungDatabase(1,i).XInnerRoadAwayTracksPoints(j,k,:),[],1))'];
                                bicycleDatabase(1,noBicycle).yTravelPoints=[bicycleDatabase(1,noBicycle).yTravelPoints (reshape(kreuzungDatabase(1,i).YInnerRoadAwayTracksPoints(j,k,:),[],1))'];
                            end
                        end
                    end
                    
                    noInternalRoads = size(kreuzungDatabase(1,i).roadJoiningHandles);
                    for j=1:noInternalRoads(2)
                        if gco == kreuzungDatabase(1,i).roadJoiningHandles(j)
                            new_poly = polyfit(kreuzungDatabase(1,i).xRoadJoiningPoints(j,:),kreuzungDatabase(1,i).yRoadJoiningPoints(j,:),2);
                            CLF = hypot(diff(kreuzungDatabase(1,i).xRoadJoiningPoints(j,:)), diff(kreuzungDatabase(1,i).yRoadJoiningPoints(j,:)));    % Calculate integrand from x,y derivatives
                            CL = trapz(CLF);                          % Integrate to calculate arc length
                            scalingUnit = 1;
                            noPoints = floor(CL/(0.2*scalingUnit));
                            new_xRoadPoints = linspace(kreuzungDatabase(1,i).xRoadJoiningPoints(j,1),kreuzungDatabase(1,i).xRoadJoiningPoints(j,end),noPoints);
                            new_yRoadPoints = polyval(new_poly,new_xRoadPoints);
                            bicycleDatabase(1,noBicycle).xTravelPoints=[bicycleDatabase(1,noBicycle).xTravelPoints new_xRoadPoints];
                            bicycleDatabase(1,noBicycle).yTravelPoints=[bicycleDatabase(1,noBicycle).yTravelPoints new_yRoadPoints];
                        end
                    end
                    
                end
                if(~isempty(bicycleDatabase(1,noBicycle).xTravelPoints))
                    %m_nscenetable.BackgroundColor=[0.85 0.85 1];
                    for q=1:size_bicycleDatabase(2)
                        if(q==noBicycle)
                            m_nscenetable.BackgroundColor(noBicycle+ size_vehicleDatabase(2)+size_pedestrianDatabase(2),:) = [.4 .4 .8];
                            
                        end
                    end
                    m_nscenetable.BackgroundColor(noBicycle+ size_vehicleDatabase(2)+size_pedestrianDatabase(2),:) = [.4 .4 .8];
                    k=1;
                end
            end
        end
    end
% Define the manual path used for pedestrians
    function defineManualPath_Callback(src, eventdata)
        
        
        
        size_pedestrianDatabase= size(pedestrianDatabase);
        size_vehicleDatabase = size(vehicleDatabase);
        size_roadDatabase = size(roadDatabase);
        size_kreuzungDatabase = size(kreuzungDatabase);
        size_bicycleDatabase = size(bicycleDatabase);
        
        
        
        isVehicle=0;
        isBicycle=0;
        isPedestrian=0;
        %find the  vehicle or bicycle or pedestrian number in database
        for i=1:size_vehicleDatabase(2)
            if vehicleDatabase(1,i).handle==currentHandle
                noVehicle = i;
                isVehicle = 1;
                break;
            end
        end
        
        for i=1:size_bicycleDatabase(2)
            if bicycleDatabase(1,i).handle==currentHandle
                noBicycle = i;
                isBicycle =1;
                break;
            end
        end
        
        for i=1:size_pedestrianDatabase(2)
            if pedestrianDatabase(1,i).handle==currentHandle
                noPedestrian = i;
                isPedestrian =1;
                break;
            end
        end
        
        if isVehicle
            %clear all tracks
            vehicleDatabase(1,noVehicle).trackHandles = [];
            vehicleDatabase(1,noVehicle).xTravelPoints = [];
            vehicleDatabase(1,noVehicle).yTravelPoints = [];
            
            hold on;
            [new_xRoadPoints new_yRoadPoints] = curvefit();
            vehicleDatabase(1,noVehicle).xTravelPoints = new_xRoadPoints;
            vehicleDatabase(1,noVehicle).yTravelPoints = new_yRoadPoints;
            plot (new_xRoadPoints,new_yRoadPoints,'--b');
            
        elseif isBicycle
            %clear all tracks
            bicycleDatabase(1,noBicycle).trackHandles = [];
            bicycleDatabase(1,noBicycle).xTravelPoints = [];
            bicycleDatabase(1,noBicycle).yTravelPoints = [];
            hold on;
            [new_xRoadPoints new_yRoadPoints] = curvefit();
            bicycleDatabase(1,noBicycle).xTravelPoints = new_xRoadPoints;
            bicycleDatabase(1,noBicycle).yTravelPoints = new_yRoadPoints;
            plot (new_xRoadPoints,new_yRoadPoints,'--b');
            
        elseif isPedestrian
            pedestrianDatabase(1,noPedestrian).xTravelPoints = [];
            pedestrianDatabase(1,noPedestrian).yTravelPoints = [];
            hold on;
            [new_xRoadPoints new_yRoadPoints] = curvefit();
            pedestrianDatabase(1,noPedestrian).xTravelPoints = new_xRoadPoints;
            pedestrianDatabase(1,noPedestrian).yTravelPoints = new_yRoadPoints;
            plot (new_xRoadPoints,new_yRoadPoints,'--b');
            for q=1:size_pedestrianDatabase(2)
                if(q==noPedestrian)
                    m_nscenetable.BackgroundColor(noPedestrian+ size_vehicleDatabase(2),:) = [.4 .4 .8];
                    
                end
            end
            
        end
    end
    function m_deleteVehicle(~,~)
        %% Contextmenu item for deleting the vehicle
        size_veh_data = size(vehicleDatabase);
        for i=1:size_veh_data(2)
            if vehicleDatabase(1,i).handle == gco
                vehicleDatabase(:,i) = [];
                break;
            end
        end
        delete (gco);
    end
    function m_deleteObject(~,~)
        %% Contextmenu item for deleting the object
        size_objectDatabase = size(objectDatabase);
        for i=1:size_objectDatabase(2)
            if objectDatabase(1,i).handle == gco
                objectDatabase(:,i) = [];
                break;
            end
        end
        delete (gco);
    end
    function m_changetoEGO(~,~)
        %% Contextmenu item for setting the blue color for the vehicle
        size_veh_data = size(vehicleDatabase);
        for i=1:size_veh_data(2)
            if vehicleDatabase(1,i).handle == gco
                vehicleDatabase(1,i).type = 'EGO';
                set(gco,'Facecolor','b');
                break;
            end
        end
    end
    function m_changetoCO(~,~)
        %% Contextmenu item for setting the red color for the vehicle
        size_veh_data = size(vehicleDatabase);
        for i=1:size_veh_data(2)
            if vehicleDatabase(1,i).handle == gco
                vehicleDatabase(1,i).type = 'Other';
                set(gco,'Facecolor','r');
                break;
            end
        end
    end
    function m_deleteRoad(~,~)
        %% Contextmenu item for deleting the road
        
        size_roadDatabase = size(roadDatabase);
        for i=1:size_roadDatabase(2)
            if roadDatabase(1,i).mainHandle ==gco
                delete gco;
                delete(roadDatabase(1,i).mainHandle);
                delete(roadDatabase(1,i).innerRoadHandles);
                delete(roadDatabase(1,i).outerRoadHandles);
                delete(roadDatabase(1,i).innerTrackHandles);
                delete(roadDatabase(1,i).outerTrackHandles);
            end
        end
    end
    function m_newRoadCreate(~,~)
        x=[];
        y=[];
        
        width=3.25;
        if(n==1)
            m_noTracks=1;
        else
            m_noTracks=2;
        end
        m_newroadCallback(x,y,width,m_noTracks);
    end
    function vehicleParametersMenu_Callback(source,eventdata)
        % Display surf plot of the currently selected data.
        vP.Visible = 'on';
        pP.Visible = 'off';
        bP.Visible = 'off';
        oP.Visible = 'off';
        m_nscene.Visible='off';
    end
    function pedestrianParametersMenu_Callback(source,eventdata)
        % Display surf plot of the currently selected data.
        vP.Visible = 'off';
        pP.Visible = 'on';
        bP.Visible = 'off';
        oP.Visible = 'off';
        m_nscene.Visible='off';
    end
    function bicycleParametersMenu_Callback(source,eventdata)
        % Display surf plot of the currently selected data.
        vP.Visible = 'off';
        pP.Visible = 'off';
        bP.Visible = 'on';
        oP.Visible = 'off';
        m_nscene.Visible='off';
    end
    function objectParametersMenu_Callback(source,eventdata)
        % Display surf plot of the currently selected data.
        vP.Visible = 'off';
        pP.Visible = 'off';
        bP.Visible = 'off';
        oP.Visible = 'on';
        m_nscene.Visible='off';
    end
    function m_newScenario(source,eventdata)
        size_vehicleDatabase = size(vehicleDatabase);
        size_pedestrianDatabase = size(pedestrianDatabase);
        size_bicycleDatabase=size(bicycleDatabase);
        % Display surf plot of the currently selected data.
        vP.Visible = 'off';
        pP.Visible = 'off';
        bP.Visible = 'off';
        oP.Visible = 'off';
        m_nscene.Visible='on';
        m_totalsizee=size_vehicleDatabase(2)+size_pedestrianDatabase(2)+size_bicycleDatabase(2);
        m_vehData={};
        for i=1:size_vehicleDatabase(2)
            m_veh=strcat('Vehicle ',num2str(i));
            m_vehData(i,:)={m_veh 'Vehile'};
        end
        for i=1:size_pedestrianDatabase(2)
            m_veh=strcat('Pedestrian ',num2str(i));
            m_vehData(end+1,:)={m_veh 'Pedestrian'};
        end
        
        for i=1:size_bicycleDatabase(2)
            m_veh=strcat('Cycle ',num2str(i));
            m_vehData(end+1,:)={m_veh 'Bicycle'};
        end
        for l=1:size_vehicleDatabase(2)+size_pedestrianDatabase(2)+size_bicycleDatabase(2)
            m_nscenetable.BackgroundColor(l,:) = [.9 .9 .9];
        end
        m_nscenetable.Data=m_vehData;
    end
    function m_edit(source,eventdata)
    size_bicycleDatabase = size(bicycleDatabase);
    size_pedestrianDatabase = size(pedestrianDatabase);
    
        handle = gco;
        currentHandle=gco;
        type=get(handle,'Type');
    
    if strcmp(type,'patch') %%Check if the object is vehicle
        vP.Visible='on';
        pP.Visible='off';
        bP.Visible='off';
        oP.Visible='off';
        m_nscene.Visible='off';
        sizeVehDatabase = size(vehicleDatabase);
        for i=1:sizeVehDatabase(2)
            if handle == vehicleDatabase(1,i).handle
                set(Vv_speed,'String',vehicleDatabase(1,i).speed);
                set(Vv_lat_accel,'String',vehicleDatabase(1,i).lateralAccn);
                set(Vv_long_accel,'String',vehicleDatabase(1,i).longitudinalAccn);
                set(Vv_mass,'String',vehicleDatabase(1,i).vehicleMass);
                set(Vv_height,'String',vehicleDatabase(1,i).heightCG);
                set(Vv_length,'String',vehicleDatabase(1,i).lengthVehicle);
                set(Vv_width,'String',vehicleDatabase(1,i).widthVehicle);
                set(Vv_lf,'String',vehicleDatabase(1,i).lf);
                set(Vv_lr,'String',vehicleDatabase(1,i).lr);
                set(Vv_lf_to_front,'String',vehicleDatabase(1,i).lfToFront);
                set(Vv_lr_to_rear,'String',vehicleDatabase(1,i).lrToRear);
                set(Vv_angle_dir,'String',vehicleDatabase(1,i).angleDir*180/pi);               
            end
        end
        
        for i=1:size_bicycleDatabase(2)
        vP.Visible='off';
        pP.Visible='off';
        bP.Visible='on';
        oP.Visible='off';
        m_nscene.Visible='off';
           if handle == bicycleDatabase(1,i).handle
                set(Bv_speed,'String',bicycleDatabase(1,i).speed);
                set(Bv_lat_accn,'String',bicycleDatabase(1,i).lateralAccn);
                set(Bv_long_accn,'String',bicycleDatabase(1,i).longitudinalAccn);
                set(Bv_direction,'String',bicycleDatabase(1,i).angleDir*180/pi);
            end
        end
    elseif strcmp(type,'rectangle') % check if the object is pedestrian
        vP.Visible='off';
        pP.Visible='on';
        bP.Visible='off';
        oP.Visible='off';
        m_nscene.Visible='off';
        size_pedestrianDatabase = size(pedestrianDatabase);
        for i=1:size_pedestrianDatabase(2)
            if handle == pedestrianDatabase(1,i).handle          
                set(Pv_speed,'String',pedestrianDatabase(1,i).speed);
                set(Pv_accn,'String',pedestrianDatabase(1,i).accn);
                set(Pv_direction,'String',pedestrianDatabase(1,i).angleDir*180/pi);
                set(Pv_max_speed,'String',pedestrianDatabase(1,i).maxSpeed);
                set(Pv_min_speed,'String',pedestrianDatabase(1,i).minSpeed);
            end
        end
        
    end
 end
    function m_saveVehicle_Callback(hObject, eventdata, handles)
    size_veh_data=size(vehicleDatabase);
    %% saving all changed parameter values in vehicle Database
    for i=1:size_veh_data(2)
        if (vehicleDatabase(1,i).handle==currentHandle)
             
        vehicleDatabase(1,i).speed=str2double(get(Vv_speed,'string'));
        vehicleDatabase(1,i).longitudinalAccn=str2double(get(Vv_long_accel,'string'));
        vehicleDatabase(1,i).lateralAccn=str2double(get(Vv_lat_accel,'string'));
        vehicleDatabase(1,i).angleDir=str2double(get(Vv_angle_dir,'string'))*pi/180;
        vehicleDatabase(1,i).vehicleMass=str2double(get(Vv_mass,'string'));
        vehicleDatabase(1,i).heightCG=str2double(get(Vv_height,'string'));
        vehicleDatabase(1,i).lengthVehicle=str2double(get(Vv_length,'string'));
        vehicleDatabase(1,i).widthVehicle=str2double(get(Vv_width,'string'));
        vehicleDatabase(1,i).lf=str2double(get(Vv_lf,'string'));
        vehicleDatabase(1,i).lr=str2double(get(Vv_lr,'string'));
        vehicleDatabase(1,i).lfToFront=str2double(get(Vv_lf_to_front,'string'));
        vehicleDatabase(1,i).lrToRear=str2double(get(Vv_lr_to_rear,'string'));
      
            %% Rotate the vehicle with calculating the unit vector
            ex=cos(str2double(get(Vv_angle_dir,'string'))*pi/180);
            ey=sin(str2double(get(Vv_angle_dir,'string'))*pi/180);     
            exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
            eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
            scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
            x_Data = zeros(4,1);
            y_Data = zeros(4,1);

            x_Data(3)=vehicleDatabase(1,i).xCG+ex*(((vehicleDatabase(1,i).lf)+(vehicleDatabase(1,i).lfToFront))/1000*scalingUnit)+((vehicleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(3)=vehicleDatabase(1,i).yCG+ey*(((vehicleDatabase(1,i).lf)+(vehicleDatabase(1,i).lfToFront))/1000*scalingUnit)+((vehicleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(4)=vehicleDatabase(1,i).xCG+ex*(((vehicleDatabase(1,i).lf)+(vehicleDatabase(1,i).lfToFront))/1000*scalingUnit)-((vehicleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(4)=vehicleDatabase(1,i).yCG+ey*(((vehicleDatabase(1,i).lf)+(vehicleDatabase(1,i).lfToFront))/1000*scalingUnit)-((vehicleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(2)=vehicleDatabase(1,i).xCG-ex*(((vehicleDatabase(1,i).lr)+(vehicleDatabase(1,i).lrToRear))/1000*scalingUnit)+((vehicleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(2)=vehicleDatabase(1,i).yCG-ey*(((vehicleDatabase(1,i).lr)+(vehicleDatabase(1,i).lrToRear))/1000*scalingUnit)+((vehicleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(1)=vehicleDatabase(1,i).xCG-ex*(((vehicleDatabase(1,i).lr)+(vehicleDatabase(1,i).lrToRear))/1000*scalingUnit)-((vehicleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(1)=vehicleDatabase(1,i).yCG-ey*(((vehicleDatabase(1,i).lr)+(vehicleDatabase(1,i).lrToRear))/1000*scalingUnit)-((vehicleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);

            vehicleDatabase(1,i).xCoordinates(1,:) = [x_Data(1) x_Data(2) x_Data(3) x_Data(4) x_Data(1)];
            vehicleDatabase(1,i).yCoordinates(1,:) = [y_Data(1) y_Data(2) y_Data(3) y_Data(4) y_Data(1)];

            set(vehicleDatabase(1,i).handle,'xData',x_Data,'yData',y_Data);
        end
      end
    end
    function m_defaultVehicle_Callback(source,eventdata)
        size_veh_data=size(vehicleDatabase);
        for i=1:size_veh_data(2)
        if (vehicleDatabase(1,i).handle==currentHandle)
        vehicleDatabase(1,(size_veh_data(2)+1)) = Vehicle;
        set(Vv_speed,'String',vehicleDatabase(1,(size_veh_data(2)+1)).speed);
        set(Vv_lat_accel,'String',vehicleDatabase(1,(size_veh_data(2)+1)).lateralAccn);
        set(Vv_long_accel,'String',vehicleDatabase(1,(size_veh_data(2)+1)).longitudinalAccn);
        set(Vv_mass,'String',vehicleDatabase(1,(size_veh_data(2)+1)).vehicleMass);
        set(Vv_height,'String',vehicleDatabase(1,(size_veh_data(2)+1)).heightCG);
        set(Vv_length,'String',vehicleDatabase(1,(size_veh_data(2)+1)).lengthVehicle);
        set(Vv_width,'String',vehicleDatabase(1,(size_veh_data(2)+1)).widthVehicle);
        set(Vv_lf,'String',vehicleDatabase(1,(size_veh_data(2)+1)).lf);
        set(Vv_lr,'String',vehicleDatabase(1,(size_veh_data(2)+1)).lr);
        set(Vv_lf_to_front,'String',vehicleDatabase(1,(size_veh_data(2)+1)).lfToFront);
        set(Vv_lr_to_rear,'String',vehicleDatabase(1,(size_veh_data(2)+1)).lrToRear);
        set(Vv_angle_dir,'String',vehicleDatabase(1,(size_veh_data(2)+1)).angleDir*180/pi); 
        vehicleDatabase(:,size_veh_data(2)+1) = [];
        m_saveVehicle_Callback;
        end
        end
    end
    function m_savePedestrian_Callback(hObject, eventdata, handles)
    size_pedestrianDatabase = size(pedestrianDatabase);
    %% saving all changed parameter values in vehicle Database
    for i=1:size_pedestrianDatabase(2)
        if (pedestrianDatabase(1,i).handle==currentHandle)
             
        pedestrianDatabase(1,i).speed=str2double(get(Pv_speed,'string'));
        pedestrianDatabase(1,i).accn=str2double(get(Pv_accn,'string'));
        pedestrianDatabase(1,i).angleDir=str2double(get(Pv_direction,'string'))*pi/180;
        pedestrianDatabase(1,i).maxSpeed=str2double(get(Pv_max_speed,'string'));
        pedestrianDatabase(1,i).minSpeed=str2double(get(Pv_min_speed,'string'));
        end
      end
    end
    function m_defaultPedestrian_Callback(source,eventdata)
        size_pedestrianDatabase = size(pedestrianDatabase);
        for i=1:size_pedestrianDatabase(2)
        if (pedestrianDatabase(1,i).handle==currentHandle)
        pedestrianDatabase(1,(size_pedestrianDatabase(2)+1)) = Pedestrian;
        set(Pv_speed,'String',pedestrianDatabase(1,(size_pedestrianDatabase(2)+1)).speed);
        set(Pv_accn,'String',pedestrianDatabase(1,(size_pedestrianDatabase(2)+1)).accn);
        set(Pv_direction,'String',pedestrianDatabase(1,(size_pedestrianDatabase(2)+1)).angleDir);
        set(Pv_max_speed,'String',pedestrianDatabase(1,(size_pedestrianDatabase(2)+1)).maxSpeed);
        set(Pv_min_speed,'String',pedestrianDatabase(1,(size_pedestrianDatabase(2)+1)).minSpeed);
        pedestrianDatabase(:,size_pedestrianDatabase(2)+1) = [];
        m_savePedestrian_Callback;
        end
        end
    end
    function m_saveBicycle_Callback(source,eventdata)
        size_bicycleDatabase = size(bicycleDatabase);
        %% saving all changed parameter values in vehicle Database
        for i=1:size_bicycleDatabase(2)
            if (bicycleDatabase(1,i).handle==currentHandle)
                
                bicycleDatabase(1,i).speed=str2double(get(Bv_speed,'string'));
                bicycleDatabase(1,i).lateralAccn=str2double(get(Bv_lat_accn,'string'));
                bicycleDatabase(1,i).longitudinalAccn=str2double(get(Bv_long_accn,'string'));
                bicycleDatabase(1,i).angleDir=str2double(get(Bv_direction,'string'))*pi/180;
                %% Rotate the vehicle with calculating the unit vector
                ex=cos(str2double(get(Bv_direction,'string'))*pi/180);
                ey=sin(str2double(get(Bv_direction,'string'))*pi/180);
                exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
                eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
                scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
                x_Data = zeros(4,1);
                y_Data = zeros(4,1);
                
                x_Data(3)=bicycleDatabase(1,i).xCG+ex*(((bicycleDatabase(1,i).lengthVehicle))/1000*scalingUnit)+((bicycleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
                y_Data(3)=bicycleDatabase(1,i).yCG+ey*(((bicycleDatabase(1,i).lengthVehicle))/1000*scalingUnit)+((bicycleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
                x_Data(4)=bicycleDatabase(1,i).xCG+ex*(((bicycleDatabase(1,i).lengthVehicle))/1000*scalingUnit)-((bicycleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
                y_Data(4)=bicycleDatabase(1,i).yCG+ey*(((bicycleDatabase(1,i).lengthVehicle))/1000*scalingUnit)-((bicycleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
                x_Data(2)=bicycleDatabase(1,i).xCG-ex*(((bicycleDatabase(1,i).lengthVehicle))/1000*scalingUnit)+((bicycleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
                y_Data(2)=bicycleDatabase(1,i).yCG-ey*(((bicycleDatabase(1,i).lengthVehicle))/1000*scalingUnit)+((bicycleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
                x_Data(1)=bicycleDatabase(1,i).xCG-ex*(((bicycleDatabase(1,i).lengthVehicle))/1000*scalingUnit)-((bicycleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
                y_Data(1)=bicycleDatabase(1,i).yCG-ey*(((bicycleDatabase(1,i).lengthVehicle))/1000*scalingUnit)-((bicycleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
                
                bicycleDatabase(1,i).xCoordinates(1,:) = [x_Data(1) x_Data(2) x_Data(3) x_Data(4) x_Data(1)];
                bicycleDatabase(1,i).yCoordinates(1,:) = [y_Data(1) y_Data(2) y_Data(3) y_Data(4) y_Data(1)];
                
                set(bicycleDatabase(1,i).handle,'xData',x_Data,'yData',y_Data);
            end
        end
    end
    function m_defaultBicycle_Callback(source,eventdata)
        size_bicycleDatabase = size(bicycleDatabase);
        for i=1:size_bicycleDatabase(2)
        if (bicycleDatabase(1,i).handle==currentHandle)
        bicycleDatabase(1,(size_bicycleDatabase(2)+1)) = Bicycle;
        set(Bv_speed,'String',bicycleDatabase(1,(size_bicycleDatabase(2)+1)).speed);
        set(Bv_lat_accn,'String',bicycleDatabase(1,(size_bicycleDatabase(2)+1)).lateralAccn);
        set(Bv_long_accn,'String',bicycleDatabase(1,(size_bicycleDatabase(2)+1)).longitudinalAccn);
        set(Bv_direction,'String',bicycleDatabase(1,(size_bicycleDatabase(2)+1)).angleDir);
        bicycleDatabase(:,size_bicycleDatabase(2)+1) = [];
        m_saveBicycle_Callback;
        end
        end
    end
    function m_saveObject_Callback(hObject, eventdata, handles)
      end
% --------------------------------------------------------------------
% Text boxes
% Simulation time callback Start
    function m_simulationtimeCall(H,~)
        m_simulationtime = get(H,'string');
    end
% Simulation time callback End
% Simulation steps callback Start
    function m_simulationstepCall(H,~)
        m_simulationstep = get(H,'string');
    end
% Simulation steps callback end
%---------------------------------------------------------------------
% Main windiow
% Buttons
%  Load Scenario Callback  Start -> m_loadscenarioCallback()
    function m_loadscenarioCallback(source,eventdata)
        % Get the file name of the *.mat to be loaded
        [fileName,pathName,~] = uigetfile('.mat','Select the Scenario','F:\THI\Thesis\Fahrdynamik Modell code\Scenario');
        wholeName = strcat(pathName,fileName);
        load(wholeName);
        size_roadDatabase = size(roadDatabase);
        size_vehicleDatabase = size(vehicleDatabase);
        size_kreuzungDatabase = size(kreuzungDatabase);
        size_pedestrianDatabase = size(pedestrianDatabase);
        size_bicycleDatabase=size(bicycleDatabase);
        size_objectDatabase=size(objectDatabase);
        % Road Database load
        % Roads
        for i=1:size_roadDatabase(2)
            hold on;
            size_outerTrackHandles = size(roadDatabase(1,i).outerTrackHandles);
            size_innerTrackHandles = size(roadDatabase(1,i).innerTrackHandles);
            roadDatabase(1,i).mainHandle = plot(roadDatabase(1,i).mainXRoadPoints,roadDatabase(1,i).mainYRoadPoints,'b-','LineWidth',1.5);
            if(size_outerTrackHandles(2)>=2)
                size_outerCurves = size(roadDatabase(1,i).xOuterCurvePoints);
                size_innerCurves = size(roadDatabase(1,i).xInnerCurvePoints);
                for j=1:size_outerCurves(1)
                    roadDatabase(1,i).outerRoadHandles(j)=plot(roadDatabase(1,i).xOuterCurvePoints(j,:),roadDatabase(1,i).yOuterCurvePoints(j,:),'g-');
                end
                roadDatabase(1,i).outerRoadHandles(size_outerCurves(1))= plot(roadDatabase(1,i).xOuterEndPoints,roadDatabase(1,i).yOuterEndPoints,'g-','LineWidth',1.5);
                for j=1:size_innerCurves(1)
                    roadDatabase(1,i).innerRoadHandles(j)=plot(roadDatabase(1,i).xInnerCurvePoints(j,:),roadDatabase(1,i).yInnerCurvePoints(j,:),'r-');
                    roadDatabase(1,i).innerRoadHandles(j+1)=plot(roadDatabase(1,i).xInnerEndPoints,roadDatabase(1,i).yInnerEndPoints,'r-','LineWidth',1.5);
                end
            end
            
            
            for j=1:size_outerTrackHandles(2)
                roadDatabase(1,i).outerTrackHandles(j) = plot(roadDatabase(1,i).xOuterCurveTrackPoints(j,:),roadDatabase(1,i).yOuterCurveTrackPoints(j,:),'--g');
            end
            
            for j=1:size_innerTrackHandles(2)
                roadDatabase(1,i).innerTrackHandles(j) = plot(roadDatabase(1,i).xInnerCurveTrackPoints(j,:),roadDatabase(1,i).yInnerCurveTrackPoints(j,:),'--r');
            end
            rcmenu = uicontextmenu;
            r1=uimenu(rcmenu,'label','Delete','callback',@m_deleteRoad);
            set(roadDatabase(1,i).mainHandle,'uicontextmenu',rcmenu);
            
        end
        
        % Crossings
        for i=1:size_kreuzungDatabase(2)
            hold on;
            
            size_noRoadToTracks = size(kreuzungDatabase(1,i).noRoadToTracks);
            
            for j=1:size_noRoadToTracks(2)
                kreuzungDatabase(1,i).mainRoadHandles=plot(kreuzungDatabase(1,i).XRoadPoints(j,:),kreuzungDatabase(1,i).YRoadPoints(j,:),'b-','LineWidth',1.5);
            end
            size_noRoadToTracks = size(kreuzungDatabase(1,i).noRoadToTracks);
            
            for j=1:size_noRoadToTracks(2)
                size_toTracksRoads = (kreuzungDatabase(1,i).noRoadToTracks(j));
                for k=1:size_toTracksRoads
                    if k==size_toTracksRoads
                        kreuzungDatabase(1,i).toTracksHandles(j,k) = plot(reshape(kreuzungDatabase(1,i).XRoadToTracksPoints(j,k,:),[],1),...
                            reshape(kreuzungDatabase(1,i).YRoadToTracksPoints(j,k,:),[],1),'r-','Linewidth',1.5);
                    else
                        kreuzungDatabase(1,i).toTracksHandles(j,k) = plot(reshape(kreuzungDatabase(1,i).XRoadToTracksPoints(j,k,:),[],1),...
                            reshape(kreuzungDatabase(1,i).YRoadToTracksPoints(j,k,:),[],1),'r-');
                    end
                end
            end
            
            size_noRoadAwayTracks = size(kreuzungDatabase(1,i).noRoadAwayTracks);
            
            for j=1:size_noRoadAwayTracks(2)
                size_awayTracksRoads = (kreuzungDatabase(1,i).noRoadAwayTracks(j));
                for k=1:size_awayTracksRoads
                    if k==size_awayTracksRoads
                        kreuzungDatabase(1,i).awayTrackHandles(j,k) = plot(reshape(kreuzungDatabase(1,i).XRoadAwayTracksPoints(j,k,:),[],1),...
                            reshape(kreuzungDatabase(1,i).YRoadAwayTracksPoints(j,k,:),[],1),'g-','Linewidth',1.5);
                    else
                        kreuzungDatabase(1,i).awayTrackHandles(j,k) = plot(reshape(kreuzungDatabase(1,i).XRoadAwayTracksPoints(j,k,:),[],1),...
                            reshape(kreuzungDatabase(1,i).YRoadAwayTracksPoints(j,k,:),[],1),'g-');
                    end
                end
            end
            
            size_noToTracks = size(kreuzungDatabase(1,i).noRoadToTracks);
            
            for j=1:size_noToTracks(2)
                size_toTracks = (kreuzungDatabase(1,i).noRoadToTracks(j));
                for k=1:size_toTracks
                    kreuzungDatabase(1,i).innerRoadToTracksHandles(j,k) = plot(reshape(kreuzungDatabase(1,i).XInnerRoadToTracksPoints(j,k,:),[],1),...
                        reshape(kreuzungDatabase(1,i).YInnerRoadToTracksPoints(j,k,:),[],1),'--r');
                end
            end
            
            size_noAwayTracks = size(kreuzungDatabase(1,i).noRoadAwayTracks);
            
            for j=1:size_noAwayTracks(2)
                size_awayTracks = (kreuzungDatabase(1,i).noRoadAwayTracks(j));
                for k=1:size_awayTracks
                    kreuzungDatabase(1,i).innerRoadAwayTracksHandles(j,k) = plot((reshape(kreuzungDatabase(1,i).XInnerRoadAwayTracksPoints(j,k,:),[],1)),...
                        (reshape(kreuzungDatabase(1,i).YInnerRoadAwayTracksPoints(j,k,:),[],1)),'--g');
                end
            end
            
            size_noRoadJoining = size(kreuzungDatabase(1,i).xRoadJoiningPoints);
            
            for j=1:size_noRoadJoining(1)
                kreuzungDatabase(1,i).roadJoiningHandles = plot(kreuzungDatabase(1,i).xRoadJoiningPoints(j,:),kreuzungDatabase(1,i).yRoadJoiningPoints(j,:),'--b');
            end
            
            
            for j=1:size_noRoadToTracks(2)
                plot(kreuzungDatabase(1,i).xCornerPoints(j,:),kreuzungDatabase(1,i).yCornerPoints(j,:),'b-','Linewidth',1.5);
            end
        end
        
        % Vehicle
        for i=1:size_vehicleDatabase(2)
            ex=cos(vehicleDatabase(1,i).angleDir(1));
            ey=sin(vehicleDatabase(1,i).angleDir(1));
            exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
            eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
            scalingUnit =1;
            x_Data = zeros(4,1);
            y_Data = zeros(4,1);
            
            x_Data(3)=vehicleDatabase(1,i).xCG(1)+ex*(((vehicleDatabase(1,i).lf)+(vehicleDatabase(1,i).lfToFront))/1000*scalingUnit)+((vehicleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(3)=vehicleDatabase(1,i).yCG(1)+ey*(((vehicleDatabase(1,i).lf)+(vehicleDatabase(1,i).lfToFront))/1000*scalingUnit)+((vehicleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(4)=vehicleDatabase(1,i).xCG(1)+ex*(((vehicleDatabase(1,i).lf)+(vehicleDatabase(1,i).lfToFront))/1000*scalingUnit)-((vehicleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(4)=vehicleDatabase(1,i).yCG(1)+ey*(((vehicleDatabase(1,i).lf)+(vehicleDatabase(1,i).lfToFront))/1000*scalingUnit)-((vehicleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(2)=vehicleDatabase(1,i).xCG(1)-ex*(((vehicleDatabase(1,i).lr)+(vehicleDatabase(1,i).lrToRear))/1000*scalingUnit)+((vehicleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(2)=vehicleDatabase(1,i).yCG(1)-ey*(((vehicleDatabase(1,i).lr)+(vehicleDatabase(1,i).lrToRear))/1000*scalingUnit)+((vehicleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
            x_Data(1)=vehicleDatabase(1,i).xCG(1)-ex*(((vehicleDatabase(1,i).lr)+(vehicleDatabase(1,i).lrToRear))/1000*scalingUnit)-((vehicleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
            y_Data(1)=vehicleDatabase(1,i).yCG(1)-ey*(((vehicleDatabase(1,i).lr)+(vehicleDatabase(1,i).lrToRear))/1000*scalingUnit)-((vehicleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
            vehicleObj = patch(x_Data,y_Data,'r');
            vehicleDatabase(1,i).handle=vehicleObj;
            
            %contextmenu
            %Contextmenu for vehicle
            hcmenu=uicontextmenu;
            h2=uimenu(hcmenu,'label','Vehicle Type');
            h1=uimenu(hcmenu,'label','Color');
            
            
            h3=uimenu(hcmenu,'label','Delete','Callback',@m_deleteVehicle);
            
            hcb1=['set(gco,''FaceColor'',[1 0 0])'];
            hcb2=['set(gco,''FaceColor'',[0 0 1])'];
            colorBlue = uimenu(h1,'label','Blue','callback',hcb2);
            colorRed = uimenu(h1,'label','Red','callback',hcb1);
            EGOVehicle=uimenu(h2,'label','EGO','callback',@m_changetoEGO);
            otherVehicle=uimenu(h2,'label','Other','callback',@m_changetoCO);
            
            set(vehicleDatabase(1).handle,'uicontextmenu',hcmenu);
            if strcmp(vehicleDatabase(1,i).type,'EGO');
                set(vehicleDatabase(1).handle,'Facecolor','b');
            elseif(~isempty(vehicleDatabase(1,i).m_polyVisionX))
                m_polyobj=fill(vehicleDatabase(1,i).m_polyVisionX(1,:),vehicleDatabase(1,i).m_polyVisionY(1,:),'r','FaceAlpha',0.05,'Visible', 'off');
                vehicleDatabase(1,i).m_polyVision=m_polyobj;
            end
            
            
        end
        
        % Pedestrians
        for k=1:size_pedestrianDatabase(2)
            xs=pedestrianDatabase(1,k).xPos(1) ;
            ys=pedestrianDatabase(1,k).yPos(1) ;
            rectangleObj = rectangle('Position',[xs,ys,0.8,0.8],'FaceColor',[0 0 1],'Curvature',[1,1]); %line(xs,ys,'tag','tmpregsel',varargin{:});
            pedestrianDatabase(1,k).handle =rectangleObj;
        end
    end
%  Load Scenario Callback End

% ***********************************************************************************
% Save Scenatio Callback Start -> m_savescenarioCallback()
    function m_savescenarioCallback(source,eventdata)
        uisave({'roadDatabase','vehicleDatabase','kreuzungDatabase', 'kreiselDatabase','pedestrianDatabase', 'bicycleDatabase','objectDatabase'});
        %         varData = whos;
        %         saveIndex = cellfun(@isempty, regexp({varData.class}, 'matlab.(graphics|ui)'));
        %         saveVars = {varData(saveIndex).name};
        %         save('no_handles.mat', saveVars{:});
    end
% Save Scenatio Callback End
% ***********************************************************************************
% Clear Scenario Start ->m_clearscenarioCallback()
    function m_clearscenarioCallback(source,eventdata)
        
        
        % Vehicles
        for i=1:size_vehicleDatabase(2)
            gvh=vehicleDatabase(1,i).handle ;
            % Laks specifc
            %             kvh=vehicleDatabase(1,i).m_polyVision;
            delete (gvh);
            % Laks specifc
            %             delete (kvh);
        end
        vehicleDatabase = [];
        
        % Bicycle
        for i=1:size_bicycleDatabase(2)
            gvh=bicycleDatabase(1,i).handle ;
            delete (gvh);
        end
        bicycleDatabase = [];
        
        % Pedestrians
        for i=1:size_pedestrianDatabase(2)
            gvh=pedestrianDatabase(1,i).handle ;
            delete (gvh);
        end
        pedestrianDatabase = [];
        
        % Objects
        for i=1:size_objectDatabase(2)
            gvh=objectDatabase(1,i).handle ;
            delete (gvh);
        end
        objectDatabase = [];
        
        
        % Roadbase
        for i=1:size_roadDatabase(2)
            delete(roadDatabase(1,i).mainHandle);
            delete(roadDatabase(1,i).innerRoadHandles);
            delete(roadDatabase(1,i).outerRoadHandles);
            delete(roadDatabase(1,i).innerTrackHandles);
            delete(roadDatabase(1,i).outerTrackHandles);
        end
        cla(m_PlotWindow,'reset')
        m_PlotWindow.XLim=[0 200];m_PlotWindow.YLim=[0 200];m_PlotWindow.Units='pixels';
        
    end
% Clear Scenario End
%*************************************************************************
% Play Scenario call back Start -> m_playscenarioCallback()
    function m_playscenarioCallback(source,eventdata)
        
        % local variables for all the processing
        m_sizevehicles = size(vehicleDatabase,2);
        m_simulationsteps = size(vehicleDatabase(1, 1).xCoordinates,1);
        m_sizepedestrian = size(pedestrianDatabase,2);
        
        for i=1:m_simulationsteps-1
            for j=1:m_sizevehicles
                % Laks Specific
                %                 if(strcmp(vehicleDatabase(1,j).type,'EGO') && (vehicleDatabase(1,j).m_trigger(i)|| vehicleDatabase(1,j).m_AreaRatio(i,:)>0.50))
                %                     k=0;
                %                 end
                vehicleDatabase(1,j).updateVehiclePosition(i);
                
                pause(0.01);
            end
            
            for k=1:m_sizepedestrian
                pedestrianDatabase(1,k).updatePedestrianPosition(i);
            end
        end
    end
% Play Scenario call back End
%---------------------------------------------------------------------
% Popup window

%---------------------------------------------------------------------
% Within Popup window
% New Vehicles Start-> m_newVehicleCallback()
    function m_newVehicleCallback(source,eventdata)
        %Contextmenu for vehicle
        hcmenu=uicontextmenu;
        h2=uimenu(hcmenu,'label','Vehicle Type');
        h1=uimenu(hcmenu,'label','Color');
           h4=uimenu(hcmenu,'label','Edit','Callback',@m_edit);
        
        h3=uimenu(hcmenu,'label','Delete','Callback',@m_deleteVehicle);
        
        hcb1=['set(gco,''FaceColor'',[1 0 0])'];
        hcb2=['set(gco,''FaceColor'',[0 0 1])'];
        colorBlue = uimenu(h1,'label','Blue','callback',hcb2);
        colorRed = uimenu(h1,'label','Red','callback',hcb1);
        EGOVehicle=uimenu(h2,'label','EGO','callback',@m_changetoEGO);
        otherVehicle=uimenu(h2,'label','Other','callback',@m_changetoCO);
        set(gcf,'Pointer','crosshair','doublebuffer','on');
        
        w=0;
        while w==0
            %Get the initial point
            [xs,ys,zs] = ginput(1);
            hold on;
            % if enter is pressed xs is empty and it is an indication to stop
            % creating new vehicle
            if (isempty(xs))
                w=1;
                set(gcf,'Pointer','arrow');
                break;
            end
            
            if (w==1)
                break;
            end
            scalingUnit =1;
            size_vehicleDatabase = size(vehicleDatabase);
            
            %% assigning vehicle radiobutton5 and calculating the dimention of vehicle and plotting into graph
            if isempty(vehicleDatabase)
                vehicleDatabase=Vehicle;
                xCoordinate(1)= xs-(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lr + vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lrToRear)/1000)/scalingUnit);
                xCoordinate(2)= xs-(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lr + vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lrToRear)/1000)/scalingUnit);
                xCoordinate(3)= xs+(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lf + vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lfToFront)/1000)/scalingUnit);
                xCoordinate(4)= xs+(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lf + vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lfToFront)/1000)/scalingUnit);
                yCoordinate(1)= ys-(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(2)= ys+(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(3)=ys+(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(4)=ys-(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                
                vehicleObj = patch(xCoordinate,yCoordinate,'r');
                vehicleDatabase(1).handle=vehicleObj;
                vehicleDatabase(1).xCG = xs;
                vehicleDatabase(1).yCG=ys;
                vehicleDatabase(1).xCoordinates(1,:) = [xCoordinate(1) xCoordinate(2) xCoordinate(3) xCoordinate(4) xCoordinate(1)];
                vehicleDatabase(1).yCoordinates(1,:) = [yCoordinate(1) yCoordinate(2) yCoordinate(3) yCoordinate(4) yCoordinate(1)];
                
                xDiagonal1 = [xCoordinate(1) xCoordinate(3)];
                xDiagonal2 = [xCoordinate(2) xCoordinate(4)];
                yDiagonal1 = [yCoordinate(1) yCoordinate(3)];
                yDiagonal2 = [yCoordinate(2) yCoordinate(4)];
                % Laks Specific
                % Save the vehicle Database for generating hypothesis
                %                 for k=1:3
                %                     m_LocalCopy(k)=copy(vehicleDatabase(1));
                %                 end
                %                 for k=1:3
                %                     vehicleDatabase(1).m_VehicleDatabaseHypo(1,k).veh=m_LocalCopy(k);
                %                 end
                set(vehicleDatabase(1).handle,'uicontextmenu',hcmenu);
            else
                vehicleDatabase(1,(size_vehicleDatabase(2)+1)) = Vehicle;
                xCoordinate(1)= xs-(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lr + vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lrToRear)/1000)/scalingUnit);
                xCoordinate(2)= xs-(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lr + vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lrToRear)/1000)/scalingUnit);
                xCoordinate(3)= xs+(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lf + vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lfToFront)/1000)/scalingUnit);
                xCoordinate(4)= xs+(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lf + vehicleDatabase(1,(size_vehicleDatabase(2)+1)).lfToFront)/1000)/scalingUnit);
                yCoordinate(1)= ys-(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(2)= ys+(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(3)=ys+(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(4)=ys-(((vehicleDatabase(1,(size_vehicleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                vehicleObj = patch(xCoordinate,yCoordinate,'r');
                
                
                vehicleDatabase(1,(size_vehicleDatabase(2)+1)).handle=vehicleObj;
                vehicleDatabase(1,(size_vehicleDatabase(2)+1)).xCG=xs;
                vehicleDatabase(1,(size_vehicleDatabase(2)+1)).yCG=ys;
                vehicleDatabase(1,(size_vehicleDatabase(2)+1)).xCoordinates(1,:)= [xCoordinate(1) xCoordinate(2) xCoordinate(3) xCoordinate(4) xCoordinate(1)];
                vehicleDatabase(1,(size_vehicleDatabase(2)+1)).yCoordinates(1,:) = [yCoordinate(1) yCoordinate(2) yCoordinate(3) yCoordinate(4) yCoordinate(1)];
                
                %----------------------------------------------------
                % Laks Specific
                %                 % Save the vehicle Database for generating hypothesis
                %                 for k=1:3
                %                     m_LocalCopy(k)=copy(vehicleDatabase(1,(size_vehicleDatabase(2)+1)));
                %                 end
                %                 for k=1:3
                %                     vehicleDatabase(1,(size_vehicleDatabase(2)+1)).m_VehicleDatabaseHypo(1,k).veh= m_LocalCopy(k);
                %                 end
                
                xDiagonal1 = [xCoordinate(1) xCoordinate(3)];
                xDiagonal2 = [xCoordinate(2) xCoordinate(4)];
                yDiagonal1 = [yCoordinate(1) yCoordinate(3)];
                yDiagonal2 = [yCoordinate(2) yCoordinate(4)];
                set(vehicleDatabase(1,(size_vehicleDatabase(2)+1)).handle,'uicontextmenu',hcmenu);
            end
        end
    end
% New Vehicles End
% ***********************************************************************************
% New Roads call back Start-> m_newroadCallback()
    function m_newroadCallback(x,y,m_width,m_tracksnumbers)
        
        
        if(isempty(x) || isempty(y))
            x=[-30;40;60;150];
            y=[34;34;34;34];
            m_width=3.25;
            m_tracksnumbers=2;
        end
        
        sizeRoadDatabase = size(roadDatabase);
        sizeKreuzungDatabase = size(kreuzungDatabase);
        sizeKreiselDatabase = size(kreiselDatabase);
        
        %plot(x,y,'ro');
        hold on;
        sizex=size(x);
        xfit=[]; yfit=[];
        x_parabola = 0;
        y_parabola = 0;
        if (sizex(1)==2)
            xfit=x;
            yfit=y;
            line(xfit,yfit);
        elseif (sizex(1)==3)
            %plot(x,y,'ro')
            hold on
            if ((x(2)>x(1))&&(x(2)>x(3)) || ((x(2)<x(1) && x(2)<x(3))))
                pz = polyfit(y,x,2);
                yfit = linspace(y(1),y(3),100);
                xfit=polyval(pz,yfit);
                CLF = hypot(diff(xfit), diff(yfit));        % Calculate integrand from x,y derivatives c = sqrt(abs(a).^2 + abs(b).^2)
                CL = trapz(CLF);                          % Integrate to calculate arc length
                scalingUnit =1;
                noPoints = floor(CL/(0.2*scalingUnit));
                yRoadPoints = linspace(y(1),y(3),noPoints);
                xRoadPoints = polyval(pz,yRoadPoints);
                %plot(xRoadPoints,yRoadPoints,'go')
            else
                pz = polyfit(x,y,2);
                xfit = linspace(x(1),x(3),100);
                yfit = polyval(pz,xfit);
                CLF = hypot(diff(xfit), diff(yfit));    % Calculate integrand from x,y derivatives
                CL = trapz(CLF);                          % Integrate to calculate arc length
                scalingUnit =1;
                noPoints =floor( CL/(0.2*scalingUnit));
                xRoadPoints = linspace(x(1),x(3),noPoints);
                yRoadPoints = polyval(pz,xRoadPoints);
            end
            % plot(xfit,yfit,'b-')
            hold on;
        else
            connectPoint = 0;
            connectPointEnd=0;
            %% Checking the endpoints of current road with endpoints of other roads
            for j=1:sizeRoadDatabase(2)
                dist1 = sqrt((x(1)-roadDatabase(1,j).xEndPoints(1)).^2 + (y(1)- roadDatabase(1,j).yEndPoints(1)).^2);
                if dist1 < 2
                    x(1) = roadDatabase(1,j).xEndPoints(1);
                    y(1) = roadDatabase(1,j).yEndPoints(1);
                    slope_endpt = roadDatabase(1,j).endPointsSlope(1);
                    connectPoint = 1;
                end
                dist2 = sqrt((x(end)-roadDatabase(1,j).xEndPoints(1)).^2 + (y(end)- roadDatabase(1,j).yEndPoints(1)).^2);
                if dist2 < 2
                    x(end) = roadDatabase(1,j).xEndPoints(1);
                    y(end) = roadDatabase(1,j).yEndPoints(1);
                    slope_endpt2 = roadDatabase(1,j).endPointsSlope(1);
                    connectPointEnd = 1;
                end
                dist3 = sqrt((x(1)-roadDatabase(1,j).xEndPoints(2)).^2 + (y(1)- roadDatabase(1,j).yEndPoints(2)).^2);
                if dist3 < 2
                    x(1) = roadDatabase(1,j).xEndPoints(2);
                    y(1) = roadDatabase(1,j).yEndPoints(2);
                    slope_endpt = roadDatabase(1,j).endPointsSlope(2);
                    connectPoint = 1;
                end
                dist4 = sqrt((x(end)-roadDatabase(1,j).xEndPoints(2)).^2 + (y(end)- roadDatabase(1,j).yEndPoints(2)).^2);
                if dist4 < 2
                    x(end) = roadDatabase(1,j).xEndPoints(2);
                    y(end) = roadDatabase(1,j).yEndPoints(2);
                    slope_endpt2 = roadDatabase(1,j).endPointsSlope(2);
                    connectPointEnd = 1;
                end
            end
            %With Kreuzung Endpoints
            for k=1:sizeKreuzungDatabase(2)
                noEndPoints  = size(kreuzungDatabase(1,k).xEndPoints);
                for l=1:noEndPoints(2)
                    dist1 = sqrt((x(1)-kreuzungDatabase(1,k).xEndPoints(l)).^2 + (y(1)- kreuzungDatabase(1,k).yEndPoints(l)).^2);
                    if dist1 < 2
                        x(1) = kreuzungDatabase(1,k).xEndPoints(l);
                        y(1) = kreuzungDatabase(1,k).yEndPoints(l);
                        connectPoint = 1;
                        slope_endpt = kreuzungDatabase(1,k).endPointsSlope(l);
                        
                    end
                    dist2 = sqrt((x(end)-kreuzungDatabase(1,k).xEndPoints(l)).^2 + (y(end)- kreuzungDatabase(1,k).yEndPoints(l)).^2);
                    if dist2 < 2
                        x(end) = kreuzungDatabase(1,k).xEndPoints(l);
                        y(end) = kreuzungDatabase(1,k).yEndPoints(l);
                        connectPointEnd =1;
                        slope_endpt2 = kreuzungDatabase(1,k).endPointsSlope(l);
                    end
                end
            end
            
            for k=1:sizeKreiselDatabase(2)
                noEndPoints  = size(kreiselDatabase(1,k).xEndPoints);
                for l=1:noEndPoints(2)
                    dist1 = sqrt((x(1)-kreiselDatabase(1,k).xEndPoints(l)).^2 + (y(1)- kreiselDatabase(1,k).yEndPoints(l)).^2);
                    if dist1 < 2
                        x(1) = kreiselDatabase(1,k).xEndPoints(l);
                        y(1) = kreiselDatabase(1,k).yEndPoints(l);
                        connectPoint = 1;
                        slope_endpt = kreiselDatabase(1,k).endPointsSlope(l);
                        
                    end
                    dist2 = sqrt((x(end)-kreiselDatabase(1,k).xEndPoints(l)).^2 + (y(end)- kreiselDatabase(1,k).yEndPoints(l)).^2);
                    if dist2 < 2
                        x(end) = kreiselDatabase(1,k).xEndPoints(l);
                        y(end) = kreiselDatabase(1,k).yEndPoints(l);
                        connectPointEnd =1;
                        slope_endpt2 = kreiselDatabase(1,k).endPointsSlope(l);
                    end
                end
            end
            
            yRoadPoints = [];
            xRoadPoints =[];
            %% if start point of road is end point of another road or kreuzung
            if connectPoint == 1
                % first two points parabola
                coef_polys(1,1)= (x(1)^2);
                coef_polys(1,2)= x(1);
                coef_polys(1,3)= 1;
                coef_polys(2,1)= (x(2)^2);
                coef_polys(2,2)= x(2);
                coef_polys(2,3)= 1;
                coef_polys(3,1)=2*x(1);
                coef_polys(3,2)=1;
                
                y_value=[y(1);y(2);slope_endpt];
                
                pz=coef_polys\y_value;
                
                new_para_x=linspace(x(1),x(2),100);
                new_para_y=polyval(pz,new_para_x);
                % plot(new_para_x,new_para_y,'r-');
                
                CLF = hypot(diff(new_para_x), diff(new_para_y));    % Calculate integrand from x,y derivatives
                CL = trapz(CLF);                          % Integrate to calculate arc length
                scalingUnit = 1;
                noPoints = floor(CL/(0.2*scalingUnit));
                new_xRoadPoints = linspace(x(1),x(2),noPoints);
                new_yRoadPoints = polyval(pz,new_xRoadPoints);
                yRoadPoints = [yRoadPoints new_yRoadPoints];
                xRoadPoints = [xRoadPoints new_xRoadPoints];
                xfit=[xfit new_para_x];
                yfit=[yfit new_para_y];
                pz_der=polyder(pz);
                slope_endpt=polyval(pz_der,x(2));
                endPointsSlope(1) = polyval(pz_der,x(1));
                
                coef_polys = zeros(3,3);
                y_value = zeros(3,1);
                new_coef = zeros(3,1);
                new_poly=pz;
                
                
                for i=0:(sizex(1)-3)
                    % Rest of the points with parabola
                    if (i==(sizex(1)-3) && connectPointEnd ==1)
                        coef_polys = zeros(4,4);
                        new_coef = zeros(3,1);
                        
                        
                        coef_polys(1,1)= (x(end-1)^3);
                        coef_polys(1,2)= x(end-1)^2;
                        coef_polys(1,3)= x(end-1);
                        coef_polys(1,4)=1;
                        coef_polys(2,1)= (x(end)^3);
                        coef_polys(2,2)= x(end)^2;
                        coef_polys(2,3)= x(end);
                        coef_polys(2,4)=1;
                        
                        coef_polys(3,1)= 3*(x(end-1)^2);
                        coef_polys(3,2)= 2*x(end-1);
                        coef_polys(3,3)= 1;
                        coef_polys(4,1)= 3*(x(end)^2);
                        coef_polys(4,2)= 2*x(end);
                        coef_polys(4,3)= 1;
                        
                        y_value=[y(end-1);y(end);slope_endpt;slope_endpt2];
                        new_poly=coef_polys\y_value;
                        
                        new_para_x=linspace(x(end-1),x(end),100);
                        new_para_y=polyval(new_poly,new_para_x);
                        % plot(new_para_x,new_para_y,'r-');
                        
                        CLF = hypot(diff(new_para_x), diff(new_para_y));    % Calculate integrand from x,y derivatives
                        CL = trapz(CLF);                          % Integrate to calculate arc length
                        scalingUnit = 1;
                        noPoints = floor(CL/(0.2*scalingUnit));
                        new_xRoadPoints = linspace(x(end-1),x(end),noPoints);
                        new_yRoadPoints = polyval(new_poly,new_xRoadPoints);
                        yRoadPoints = [yRoadPoints new_yRoadPoints];
                        xRoadPoints = [xRoadPoints new_xRoadPoints];
                        pz_der=polyder(new_poly);
                        % slope_endpt=polyval(pz_der,x(i));
                        
                    elseif ((x(i+2)>x(i+1) && x(i+2)>x(i+3)) || (x(i+2)<x(i+1) && x(i+2)<x(i+3)))
                        
                        if (x_parabola == 1 && i~=1)
                            
                            
                        end
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
                        scalingUnit = 1;
                        noPoints = floor(CL/(0.2*scalingUnit));
                        new_yRoadPoints = linspace(y(i+2),y(i+3),noPoints);
                        new_xRoadPoints = polyval(new_poly,new_yRoadPoints);
                        yRoadPoints = [yRoadPoints new_yRoadPoints];
                        xRoadPoints = [xRoadPoints new_xRoadPoints];
                        xfit=[xfit new_para_x];
                        yfit=[yfit new_para_y];
                        pz_der=polyder(new_poly);
                        slope_endpt=polyval(pz_der,y(i+3));
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
                        scalingUnit = 1;
                        noPoints = floor(CL/(0.2*scalingUnit));
                        new_xRoadPoints = linspace(x(i+2),x(i+3),noPoints);
                        new_yRoadPoints = polyval(new_poly,new_xRoadPoints);
                        yRoadPoints = [yRoadPoints new_yRoadPoints];
                        xRoadPoints = [xRoadPoints new_xRoadPoints];
                        xfit=[xfit new_para_x];
                        yfit=[yfit new_para_y];
                        pz_der=polyder(new_poly);
                        slope_endpt=polyval(pz_der,x(i+3));
                        % plot(xRoadPoints,yRoadPoints,'r-');
                        
                        
                    end
                    
                end
                
            else
                
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
                    scalingUnit =1;
                    noPoints = floor(CL/(0.2*scalingUnit));
                    yRoadPoints = linspace(y(1),y(3),noPoints);
                    xRoadPoints = polyval(pz,yRoadPoints);
                    pz_der=polyder(pz);
                    slope_endpt=polyval(pz_der,y(3));
                    endPointsSlope(1) = polyval(pz_der,x(1));
                    
                    %roadDatabase(1,sizeRoadDatabase(2)+1).endPointsSlope(1) = polyval(pz_der,y(1));
                    y_parabola=1;
                else
                    xfit_init = [linspace(init_x(1),init_x(2),100) linspace(init_x(2),init_x(3),100)] ;
                    pz = polyfit(init_x,init_y,2);
                    yfit_init = polyval(pz,xfit_init);
                    CLF = hypot(diff(xfit_init), diff(yfit_init));    % Calculate integrand from x,y derivatives
                    CL = trapz(CLF);                          % Integrate to calculate arc length
                    scalingUnit =1;
                    noPoints = floor(CL/(0.2*scalingUnit));
                    xRoadPoints = linspace(x(1),x(3),noPoints);
                    yRoadPoints = polyval(pz,xRoadPoints);
                    pz_der=polyder(pz);
                    slope_endpt=polyval(pz_der,x(3));
                    endPointsSlope(1) = polyval(pz_der,x(1));
                    
                    x_parabola = 1;
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
                    if (i==(sizex(1)-3) && connectPointEnd ==1)
                        coef_polys = zeros(4,4);
                        new_coef = zeros(3,1);
                        
                        
                        coef_polys(1,1)= (x(end-1)^3);
                        coef_polys(1,2)= x(end-1)^2;
                        coef_polys(1,3)= x(end-1);
                        coef_polys(1,4)=1;
                        coef_polys(2,1)= (x(end)^3);
                        coef_polys(2,2)= x(end)^2;
                        coef_polys(2,3)= x(end);
                        coef_polys(2,4)=1;
                        
                        coef_polys(3,1)= 3*(x(end-1)^2);
                        coef_polys(3,2)= 2*x(end-1);
                        coef_polys(3,3)= 1;
                        coef_polys(4,1)= 3*(x(end)^2);
                        coef_polys(4,2)= 2*x(end);
                        coef_polys(4,3)= 1;
                        
                        y_value=[y(end-1);y(end);slope_endpt;slope_endpt2];
                        new_poly=coef_polys\y_value;
                        
                        new_para_x=linspace(x(end-1),x(end),100);
                        new_para_y=polyval(new_poly,new_para_x);
                        % plot(new_para_x,new_para_y,'r-');
                        
                        CLF = hypot(diff(new_para_x), diff(new_para_y));    % Calculate integrand from x,y derivatives
                        CL = trapz(CLF);                          % Integrate to calculate arc length
                        scalingUnit = 1;
                        noPoints = floor(CL/(0.2*scalingUnit));
                        new_xRoadPoints = linspace(x(end-1),x(end),noPoints);
                        new_yRoadPoints = polyval(new_poly,new_xRoadPoints);
                        yRoadPoints = [yRoadPoints new_yRoadPoints];
                        xRoadPoints = [xRoadPoints new_xRoadPoints];
                        pz_der=polyder(new_poly);
                        % slope_endpt=polyval(pz_der,x(i));
                    elseif ((x(i+2)>x(i+1) && x(i+2)>x(i+3)) || (x(i+2)<x(i+1) && x(i+2)<x(i+3)))
                        %          if(((x(i+2)>x(i+1) && x(i+2)>x(i+3)) || (x(i+2)<x(i+1) && x(i+2)<x(i+3))) && (y(i+3)>y(i+2)))
                        if (x_parabola == 1 && i~=1)
                            
                            
                        end
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
                        scalingUnit =1;
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
                        scalingUnit =1;
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
            end
            
            
        end
        
        
        %% offset for parallel curve
        % To be replaced
        d=3.25;
        %         d=str2double(get(handles.widthTrack,'String'));
        
        % % Make sure that x and y are column vectors.
        xRoadPoints=xRoadPoints(:);
        yRoadPoints=yRoadPoints(:);
        
        %%% Contextmenu for Road
        %direction of road
        if (xRoadPoints(1)>xRoadPoints(2))
            
            xRoadPoints=flipud(xRoadPoints);
            yRoadPoints=flipud(yRoadPoints);
        end
        
        % % Calculate the unit gradient in the x-direction.
        dx=gradient(xRoadPoints);
        
        % % Calculate the unit gradient in the y-direction.
        dy=gradient(yRoadPoints);
        
        % % Calculate the unit second gradient in the x-direction.
        dx2=gradient(dx);
        
        % % Calculate the unit second gradient in the y-direction.
        dy2=gradient(dy);
        
        % % Calculate the normal vector
        nv=[dy, -dx];
        
        % % normalize the normal vector
        unv=zeros(size(nv));
        norm_nv=sqrt(dy.^2+dx.^2);
        unv(:, 1)=nv(:, 1)./norm_nv;
        unv(:, 2)=nv(:, 2)./norm_nv;
        
        %     % % Determine concavity
        %     concavity=2*(dy2 > 0)-1;
        
        
        noInnerCurve=m_tracksnumbers;
        noOuterCurve=m_tracksnumbers;
        
        %         noInnerCurve = str2double(get(handles.noInnerTracks,'String'));
        %         noOuterCurve = str2double(get(handles.noOuterTracks,'String'));
        
        % % determine radius of curvature
        R=(dx.^2+dy.^2).^(3/2)./abs(dx.*dy2-dy.*dx2);
        
        % % Determine overlap points for inner normal curve
        overlap= (R < noInnerCurve * d) ;
        underlap = (R < noOuterCurve * d);
        
        %Contextmenu
        
        rcmenu= uicontextmenu;
        r1=uimenu(rcmenu,'label','Delete','callback',@m_deleteRoad);
        
        if (any(overlap) || any(underlap))
            errordlg('The track width is too large for given road profile.')
        else
            hold on;
            roadDatabase(1,sizeRoadDatabase(2)+1).endPointsSlope = endPointsSlope;
            roadDatabase(1,sizeRoadDatabase(2)+1).xEndPoints = [x(1) x(end)];
            roadDatabase(1,sizeRoadDatabase(2)+1).yEndPoints = [y(1) y(end)];
            roadDatabase(1,sizeRoadDatabase(2)+1).mainHandle = plot(xRoadPoints,yRoadPoints,'b','Linewidth',2);
            % TBR
            roadDatabase(1,sizeRoadDatabase(2)+1).widthTrack = m_width;
            %             roadDatabase(1,sizeRoadDatabase(2)+1).widthTrack = str2double(get(handles.widthTrack,'String'));
            roadDatabase(1,sizeRoadDatabase(2)+1).mainXRoadPoints = xRoadPoints;
            roadDatabase(1,sizeRoadDatabase(2)+1).mainYRoadPoints = yRoadPoints;
            roadDatabase(1,sizeRoadDatabase(2)+1).endPointsSlope(2)=slope_endpt;
            set(roadDatabase(1,sizeRoadDatabase(2)+1).mainHandle,'uicontextmenu',rcmenu);
            % TBR
            %             roadDatabase(1,sizeRoadDatabase(2)+1).speedlimit = str2double(get(handles.speedlimit,'String'));
            roadDatabase(1,sizeRoadDatabase(2)+1).speedlimit = 10;
            
            for i=1:noInnerCurve
                % % For inner normal curve
                x_inner=xRoadPoints-unv(:, 1).*d*(i-0.5);
                y_inner=yRoadPoints-unv(:, 2).*d*(i-0.5);
                roadDatabase(1,sizeRoadDatabase(2)+1).xInnerCurveTrackPoints(i,:)=flipud(x_inner);
                roadDatabase(1,sizeRoadDatabase(2)+1).yInnerCurveTrackPoints(i,:)=flipud(y_inner);
                roadDatabase(1,sizeRoadDatabase(2)+1).innerTrackHandles(i) = plot(x_inner, y_inner, '--r');
            end
            
            for i=1:noInnerCurve
                % % For inner normal curve
                x_inner=xRoadPoints-unv(:, 1).*d*i;
                y_inner=yRoadPoints-unv(:, 2).*d*i;
                if (i==noInnerCurve)
                    roadDatabase(1,sizeRoadDatabase(2)+1).innerRoadHandles(i) = plot(x_inner, y_inner, 'r','Linewidth', 2);
                    roadDatabase(1,sizeRoadDatabase(2)+1).xInnerEndPoints=x_inner;
                    roadDatabase(1,sizeRoadDatabase(2)+1).yInnerEndPoints=y_inner;
                else
                    roadDatabase(1,sizeRoadDatabase(2)+1).xInnerCurvePoints(i,:)=x_inner;
                    roadDatabase(1,sizeRoadDatabase(2)+1).yInnerCurvePoints(i,:)=y_inner;
                    roadDatabase(1,sizeRoadDatabase(2)+1).innerRoadHandles(i) = plot(x_inner, y_inner, 'r');
                end
                
                
            end
            
            
            
            for i=1:noOuterCurve
                % % For outer normal curve
                x_outer=xRoadPoints+unv(:, 1).*d*i;
                y_outer=yRoadPoints+unv(:, 2).*d*i;
                if (i==noOuterCurve)
                    roadDatabase(1,sizeRoadDatabase(2)+1).outerRoadHandles(i) = plot(x_outer, y_outer, 'g','Linewidth',2);
                    roadDatabase(1,sizeRoadDatabase(2)+1).xOuterEndPoints=x_outer;
                    roadDatabase(1,sizeRoadDatabase(2)+1).yOuterEndPoints=y_outer;
                else
                    roadDatabase(1,sizeRoadDatabase(2)+1).xOuterCurvePoints(i,:)=x_outer;
                    roadDatabase(1,sizeRoadDatabase(2)+1).yOuterCurvePoints(i,:)=y_outer;
                    roadDatabase(1,sizeRoadDatabase(2)+1).outerRoadHandles(i) = plot(x_outer, y_outer, 'g');
                end
            end
            
            for i=1:noOuterCurve
                % % For outer normal curve
                x_outer=xRoadPoints+unv(:, 1).*d*(i-0.5);
                y_outer=yRoadPoints+unv(:, 2).*d*(i-0.5);
                roadDatabase(1,sizeRoadDatabase(2)+1).xOuterCurveTrackPoints(i,:)=x_outer;
                roadDatabase(1,sizeRoadDatabase(2)+1).yOuterCurveTrackPoints(i,:)=y_outer;
                roadDatabase(1,sizeRoadDatabase(2)+1).outerTrackHandles(i) = plot(x_outer, y_outer, '--g');
            end
            
        end
    end
% New Roads call back End
% ***********************************************************************************
% New Pedestrian call back start -> m_newpedestrianCallback()
    function m_newpedestrianCallback(source,eventdata)
        % hObject    handle to newPedestrian (see GCBO)
        % eventdata  reserved - to be defined in a future version of MATLAB
        % handles    structure with handles and user data (see GUIDATA)
        set(gcf,'Pointer','crosshair','doublebuffer','on');
        w=0;
        while w==0
            %Get the initial point
            [xs,ys,zs] = ginput(1);
            hold on;
            % if enter is pressed xs is empty and it is an indication to stop
            % creating new vehicle
            if (isempty(xs))
                w=1;
                set(gcf,'Pointer','arrow');
                break;
            end
            
            if (w==1)
                break;
            end
            
            size_pedestrianDatabase = size(pedestrianDatabase);
            
            if isempty(pedestrianDatabase)
                pedestrianDatabase=Pedestrian;
                pedestrianDatabase(1,1).xPos = xs;
                pedestrianDatabase(1,1).yPos = ys;
                pedestrianDatabase(1,1).speed = 5/3.6;
                pedestrianDatabase(1,1).accn=0;
                pedestrianDatabase(1,1).angleDir=0;
                pedestrianDatabase(1,1).maxSpeed=7/3.6;
                pedestrianDatabase(1,1).minSpeed=0;
            else
                pedestrianDatabase(1,size_pedestrianDatabase(2)+1)=Pedestrian;
                pedestrianDatabase(1,size_pedestrianDatabase(2)+1).xPos = xs;
                pedestrianDatabase(1,size_pedestrianDatabase(2)+1).yPos = ys;
                pedestrianDatabase(1,size_pedestrianDatabase(2)+1).speed = 5/3.6;
                pedestrianDatabase(1,size_pedestrianDatabase(2)+1).accn=0;
                pedestrianDatabase(1,size_pedestrianDatabase(2)+1).angleDir=0;
                pedestrianDatabase(1,size_pedestrianDatabase(2)+1).maxSpeed=7/3.6;
                pedestrianDatabase(1,size_pedestrianDatabase(2)+1).minSpeed=0;
            end
            %Create and store line radiobutton5
            rectangleObj = rectangle('Position',[xs,ys,0.8,0.8],'FaceColor',[0 0 1],'Curvature',[1,1]); %line(xs,ys,'tag','tmpregsel',varargin{:});
            
            pedestrianDatabase(1,size_pedestrianDatabase(2)+1).handle =rectangleObj;
            
            
            %Contextmenu for pedestrin
            hcmenu=uicontextmenu;
			  h1=uimenu(hcmenu,'label','Color');
            hcb1=['set(gco,''FaceColor'',[1 0 0])'];
            hcb2=['set(gco,''FaceColor'',[0 0 1])'];
            item1=uimenu(h1,'Label','Red','Callback',hcb1);
            item2=uimenu(h1,'Label','Blue','Callback',hcb2);
            item3=uimenu(hcmenu,'Label','Edit','Callback',@m_edit);
            
            hRectangles = findall(gca,'Type','Rectangle');
            
            for Rectangle=1:length(hRectangles)
                set(hRectangles(Rectangle),'uicontextmenu',hcmenu);
            end
            
        end
    end
% New Pedestrian call back start
% *****************************************************************************
% New object call back start -> m_newobjectCallback()
    function m_newObjectCallback(source,eventdata)
        
        %Contextmenu for vehicle
        hcmenu=uicontextmenu;
        h3=uimenu(hcmenu,'label','Delete','Callback',@m_deleteObject);
        set(gcf,'Pointer','crosshair','doublebuffer','on');
        w=0;
        while(w==0)
            %Get the initial point
            [xs,ys,zs] = ginput(1);
            hold on
            if (isempty(xs))
                w=1;
                set(gcf,'Pointer','arrow');
                break;
            end
            
            if (w==1)
                break;
            end
        %Get the initial point
        
        size_objectDatabase = size(stationaryObjectDatabase);
        if isempty(stationaryObjectDatabase)
            objectDatabase=StationaryObject;
        else
            objectDatabase(1,size_objectDatabase(2)+1)=StationaryObject;
        end
        
        %TBR
        % objectDatabase(1,size_objectDatabase(2)+1).length = str2double(get(handles.lengthObject,'String'));
        % objectDatabase(1,size_objectDatabase(2)+1).breadth = str2double(get(handles.breadthObject,'String'));
        % objectDatabase(1,size_objectDatabase(2)+1).angle = str2double(get(handles.angleObject,'String'));
        objectDatabase(1,size_objectDatabase(2)+1).xCenter = xs;
        objectDatabase(1,size_objectDatabase(2)+1).yCenter = ys;
        %TBR
        ex=0;
        ey=0;
        % ex=cos((str2double(get(handles.angleObject,'String')))*pi/180);
        % ey=sin((str2double(get(handles.angleObject,'String')))*pi/180);
        exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
        eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
        scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
        x_Data = zeros(4,1);
        y_Data = zeros(4,1);
        
        x_Data(3)=objectDatabase(1,size_objectDatabase(2)+1).xCenter+ex*((objectDatabase(1,size_objectDatabase(2)+1).length)/2)+((objectDatabase(1,size_objectDatabase(2)+1).breadth)/2*exOrtho/scalingUnit);
        y_Data(3)=objectDatabase(1,size_objectDatabase(2)+1).yCenter+ey*((objectDatabase(1,size_objectDatabase(2)+1).length)/2)+((objectDatabase(1,size_objectDatabase(2)+1).breadth)/2*eyOrtho/scalingUnit);
        x_Data(4)=objectDatabase(1,size_objectDatabase(2)+1).xCenter+ex*((objectDatabase(1,size_objectDatabase(2)+1).length)/2)-((objectDatabase(1,size_objectDatabase(2)+1).breadth)/2*exOrtho/scalingUnit);
        y_Data(4)=objectDatabase(1,size_objectDatabase(2)+1).yCenter+ey*((objectDatabase(1,size_objectDatabase(2)+1).length)/2)-((objectDatabase(1,size_objectDatabase(2)+1).breadth)/2*eyOrtho/scalingUnit);
        x_Data(2)=objectDatabase(1,size_objectDatabase(2)+1).xCenter-ex*((objectDatabase(1,size_objectDatabase(2)+1).length)/2)+((objectDatabase(1,size_objectDatabase(2)+1).breadth)/2*exOrtho/scalingUnit);
        y_Data(2)=objectDatabase(1,size_objectDatabase(2)+1).yCenter-ey*((objectDatabase(1,size_objectDatabase(2)+1).length)/2)+((objectDatabase(1,size_objectDatabase(2)+1).breadth)/2*eyOrtho/scalingUnit);
        x_Data(1)=objectDatabase(1,size_objectDatabase(2)+1).xCenter-ex*((objectDatabase(1,size_objectDatabase(2)+1).length)/2)-((objectDatabase(1,size_objectDatabase(2)+1).breadth)/2*exOrtho/scalingUnit);
        y_Data(1)=objectDatabase(1,size_objectDatabase(2)+1).yCenter-ey*((objectDatabase(1,size_objectDatabase(2)+1).length)/2)-((objectDatabase(1,size_objectDatabase(2)+1).breadth)/2*eyOrtho/scalingUnit);
        
        objectDatabase(1,size_objectDatabase(2)+1).xCoordinates(1,:) = [x_Data(1) x_Data(2) x_Data(3) x_Data(4) x_Data(1)];
        objectDatabase(1,size_objectDatabase(2)+1).yCoordinates(1,:) = [y_Data(1) y_Data(2) y_Data(3) y_Data(4) y_Data(1)];
        
        objectDatabase(1,size_objectDatabase(2)+1).handle = plot(objectDatabase(1,size_objectDatabase(2)+1).xCoordinates,objectDatabase(1,size_objectDatabase(2)+1).yCoordinates,'k-','Linewidth',1.5);
        set(objectDatabase(1,size_objectDatabase(2)+1).handle,'uicontextmenu',hcmenu);
        end
    end
% New object call back End
%*******************************************************************************
% New bicycle call back Start -> m_newbicycleCallback()
    function newBicycle_Callback(source,eventdata)
        % hObject    handle to newBicycle (see GCBO)
        % eventdata  reserved - to be defined in a future version of MATLAB
        % handles    structure with handles and user data (see GUIDATA)
        %Contextmenu for vehicle
        hcmenu=uicontextmenu;
        h1=uimenu(hcmenu,'label','Edit','Callback',@m_edit);
        set(gcf,'Pointer','crosshair','doublebuffer','on');
        w=0;
        while(w==0)
            %Get the initial point
            [xs,ys,zs] = ginput(1);
            hold on
            if (isempty(xs))
                w=1;
                set(gcf,'Pointer','arrow');
                break;
            end
            
            if (w==1)
                break;
            end
            
            size_bicycleDatabase = size(bicycleDatabase);
            if isempty (bicycleDatabase)
                bicycleDatabase = Bicycle;
                scalingUnit=1;
                xCoordinate(1)= xs-(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).lengthVehicle)/2000)/scalingUnit);
                xCoordinate(2)= xs-(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).lengthVehicle)/2000 )/scalingUnit);
                xCoordinate(3)= xs+(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).lengthVehicle)/2000)/scalingUnit);
                xCoordinate(4)= xs+(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).lengthVehicle)/2000)/scalingUnit);
                yCoordinate(1)= ys-(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(2)= ys+(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(3)=ys+(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(4)=ys-(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                bicycleObj = patch(xCoordinate,yCoordinate,'m');
                bicycleDatabase(1,1).handle = bicycleObj;
                bicycleDatabase(1,1).xCG = xs;
                bicycleDatabase(1,1).yCG = ys;
                bicycleDatabase(1).xCoordinates(1,:) = [xCoordinate(1) xCoordinate(2) xCoordinate(3) xCoordinate(4) xCoordinate(1)];
                bicycleDatabase(1).yCoordinates(1,:) = [yCoordinate(1) yCoordinate(2) yCoordinate(3) yCoordinate(4) yCoordinate(1)];
                set(bicycleDatabase(1,(size_bicycleDatabase(2)+1)).handle,'uicontextmenu',hcmenu);
            else
                bicycleDatabase(1,size_bicycleDatabase(2)+1) = Bicycle;
                scalingUnit=1;
                xCoordinate(1)= xs-(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).lengthVehicle)/2000)/scalingUnit);
                xCoordinate(2)= xs-(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).lengthVehicle)/2000 )/scalingUnit);
                xCoordinate(3)= xs+(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).lengthVehicle)/2000)/scalingUnit);
                xCoordinate(4)= xs+(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).lengthVehicle)/2000)/scalingUnit);
                yCoordinate(1)= ys-(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(2)= ys+(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(3)=ys+(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                yCoordinate(4)=ys-(((bicycleDatabase(1,(size_bicycleDatabase(2)+1)).widthVehicle)/2000)/scalingUnit);
                bicycleObj = patch(xCoordinate,yCoordinate,'m');
                bicycleDatabase(1,(size_bicycleDatabase(2)+1)).handle = bicycleObj;
                bicycleDatabase(1,size_bicycleDatabase(2)+1).xCG = xs;
                bicycleDatabase(1,size_bicycleDatabase(2)+1).yCG = ys;
                bicycleDatabase(1,(size_bicycleDatabase(2)+1)).xCoordinates(1,:) = [xCoordinate(1) xCoordinate(2) xCoordinate(3) xCoordinate(4) xCoordinate(1)];
                bicycleDatabase(1,(size_bicycleDatabase(2)+1)).yCoordinates(1,:)= [yCoordinate(1) yCoordinate(2) yCoordinate(3) yCoordinate(4) yCoordinate(1)];
                set(bicycleDatabase(1,(size_bicycleDatabase(2)+1)).handle,'uicontextmenu',hcmenu);
            end
        end
    end
% New bicycle call back end
%********************************************************************************
% Start Simulation call back Start -> m_startsimulationCallback()
    function m_startsimulationCallback(source,eventdata)
        
        axes  = findall(gcf,'Type','axes');  %%%finding all the axes in gui\
        size_vehicleDatabase = size(vehicleDatabase);
        size_pedestrianDatabase = size(pedestrianDatabase);
        size_bicycleDatabase = size(bicycleDatabase);
        size_roadDatabase = size(roadDatabase);
        size_kreuzungDatabase = size(kreuzungDatabase);
        size_objectDatabase=size(stationaryObjectDatabase);
        currentStep=1;
        [~,idx]=min(sqrt((vehicleDatabase(1,1).xTravelPoints-vehicleDatabase(1,1).xCG(1)).^2+(vehicleDatabase(1,1).yTravelPoints-vehicleDatabase(1,1).yCG(1)).^2));
        originalxCG=vehicleDatabase(1,1).xCG;
        originalyCG=vehicleDatabase(1,1).yCG;
        originalSpeed=vehicleDatabase(1,1).speed(1);
        originalAccn=vehicleDatabase(1,1).longitudinalAccn(1);
        count=1;
        vP.Visible = 'off';
        pP.Visible = 'off';
        bP.Visible = 'off';
        oP.Visible = 'off';
        m_nscene.Visible='off';
        %Data Generation
        %         for i=1:size_vehicleDatabase(2)
        %             [~,idx]=min(sqrt((vehicleDatabase(1,1).xTravelPoints-vehicleDatabase(1,1).xCG(1)).^2+(vehicleDatabase(1,1).yTravelPoints-vehicleDatabase(1,1).yCG(1)).^2));
        %             vehicleDatabase(1,1).xCG=vehicleDatabase(1,1).xTravelPoints(idx+5);
        %             vehicleDatabase(1,1).yCG=vehicleDatabase(1,1).yTravelPoints(idx+5);
        %
        %             vehicleDatabase(1,1).angleDir(1)=atan2(vehicleDatabase(1,1).yTravelPoints(idx+7)-vehicleDatabase(1,1).yTravelPoints(idx+5),...
        %                 (vehicleDatabase(1,1).xTravelPoints(idx+7)-vehicleDatabase(1,1).xTravelPoints(idx+5)));
        %             vehicleDatabase(1,1).updateVehiclePosition(1);
        %             for j=1:3
        %                 vehicleDatabase(1,1).speed(1)=vehicleDatabase(1,1).speed(1)-1;
        %                 for k=1:1
        %                     ax=-4+k*2;
        %                     vehicleDatabase(1,1).longitudinalAccn=ax;
        %                     % tempVehicleDatabase(:,1) = [];
        %                     tempVehicleDatabase=vehicleDatabase;
        %                     %
        %                     tempPedestrianDatabase=pedestrianDatabase;
        %                     [count]=vehicle_para_update(tempVehicleDatabase,tempPedestrianDatabase,count);
        %                     %% simulate
        %                     flag=inters2(vehicleDatabase,pedestrianDatabase);
        %                     if count>0
        %                         simulateScenario(count,m_simulationtime,m_simulationstep);
        %                     end
        %                 end
        %             end
        %             vehicleDatabase(1,1).speed(1)=originalSpeed;
        %         end
        for i=1:size_vehicleDatabase(2)
            [~,idx]=min(sqrt((vehicleDatabase(1,1).xTravelPoints-vehicleDatabase(1,1).xCG(1)).^2+(vehicleDatabase(1,1).yTravelPoints-vehicleDatabase(1,1).yCG(1)).^2));
            % vehicleDatabase(1,1).xCG=originalxCG;
            % vehicleDatabase(1,1).yCG=originalyCG;
            %
            % vehicleDatabase(1,1).longitudinalAccn(1)=originalAccn;
            vehicleDatabase(1,i).angleDir(1)=atan2((vehicleDatabase(1,i).yTravelPoints(idx+7)-vehicleDatabase(1,i).yTravelPoints(idx+5)),((vehicleDatabase(1,i).xTravelPoints(idx+7)-vehicleDatabase(1,i).xTravelPoints(idx+5))));
            vehicleDatabase(1,i).updateVehiclePosition(1);
        end
        %% Just to simulate the scenario
        count=1;
        % Laks Specific
        %         simulateScenario(count,m_simulationtime,m_simulationstep);
        simulateScenario(count);
    end
% Start Simulation call back End

%--------------------------------------------------------------------------
end