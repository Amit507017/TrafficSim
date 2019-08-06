function varargout = gui(varargin)
% GUI MATLAB code for gui.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs ar e passed to gui_OpeningFcn via varargin.
%

%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui

% Last Modified by GUIDE v2.5 23-Oct-2017 17:02:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @gui_OpeningFcn, ...
    'gui_OutputFcn',  @gui_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before gui is made visible.
function gui_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui (see VARARGIN)

axes1  = findobj(gcf,'Tag','axes2')        %%%finding all the axes in gui\
%% Setting the default dimention of axis
axis(axes1,[0,100,0,100]);
axis manual;

%handles
axes(handles.axesTHI)

thiImage=imread('C:\Users\chaulwar\C\Amit\Projects\HySLEUS\Publish_Tool\Code\thi.png');
image(thiImage);
axis off;
axis image;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
%varargout{1} = handles.output;

%% clear all global variables while closing the GUI
clear -global;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1

function hcb3(~,~)
%% Contextmenu item for deleting the vehicle
global vehicleDatabase;
size_veh_data = size(vehicleDatabase);
for i=1:size_veh_data(2)
    if vehicleDatabase(1,i).handle == gco
        vehicleDatabase(:,i) = [];
        break;
    end ;
end
delete (gco);

function ocb3(~,~)
%% Contextmenu item for deleting the object
global objectDatabase;
size_objectDatabase = size(objectDatabase);
for i=1:size_objectDatabase(2)
    if objectDatabase(1,i).handle == gco
        objectDatabase(:,i) = [];
        break;
    end
end
delete (gco);

function hcb4(~,~)
%% Contextmenu item for setting the blue color for the vehicle
global vehicleDatabase;
size_veh_data = size(vehicleDatabase);
for i=1:size_veh_data(2)
    if vehicleDatabase(1,i).handle == gco
        vehicleDatabase(1,i).type = 'EGO';
        set(gco,'Facecolor','b');
        break;
    end
end

function hcb5(~,~)
%% Contextmenu item for setting the red color for the vehicle
global vehicleDatabase;
size_veh_data = size(vehicleDatabase);
for i=1:size_veh_data(2)
    if vehicleDatabase(1,i).handle == gco
        vehicleDatabase(1,i).type = 'Other';
        set(gco,'Facecolor','r');
        break;
    end
end


% --- Executes when selected radiobutton5 is changed in uipanel13.
function uipanel13_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected radiobutton5 in uipanel13
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected radiobutton5 or empty if none was selected
%	NewValue: handle of the currently selected radiobutton5
% handles    structure with handles and user data (see GUIDATA)

global vehicleDatabase;
axes  = findall(gcf,'Type','axes');  %%%finding all the axes in gui\
handles.output = hObject;
oldhold = ishold(gca);
hold on;
%% Displaying panels based on the selection of the radiobutton
%% Vehicle
if get(handles.radiobutton1,'Value')==1.0
    set(handles.pedestrianParameters,'Visible','Off');
    set(handles.bicycleParameters,'Visible','off');
    set(handles.roadParameters,'Visible','off');
    set(handles.roadInformation,'Visible','off');
    set(handles.squareInformation,'Visible','Off');
    set(handles.roundaboutInformation,'Visible','Off');
    set(handles.scenarioPanel,'Visible','off');
    set(handles.objectPanal,'Visible','off');
    set(handles.vehicleParameters,'Visible','on');
    
    
    
    %% Pedestrian
elseif get(handles.radiobutton2,'Value')==1.0
    set(handles.vehicleParameters,'Visible','off');
    set(handles.roadParameters,'Visible','off');
    set(handles.roadInformation,'Visible','off');
    set(handles.squareInformation,'Visible','Off');
    set(handles.roundaboutInformation,'Visible','Off');
    set(handles.scenarioPanel,'Visible','off')
    set(handles.bicycleParameters,'Visible','off');
    set(handles.objectPanal,'Visible','off');
    set(handles.pedestrianParameters,'Visible','On');
    
    
    %%Road
elseif get(handles.radiobutton3,'Value')==1.0
    set(handles.pedestrianParameters,'Visible','off');
    set(handles.vehicleParameters,'Visible','off');
    set(handles.squareInformation,'Visible','Off');
    set(handles.roundaboutInformation,'Visible','Off');
    set(handles.scenarioPanel,'Visible','off');
    set(handles.objectPanal,'Visible','off');
    set(handles.roadParameters,'Visible','on');
    set(handles.roadInformation,'Visible','on');
    set(handles.bicycleParameters,'Visible','off');
    %Bicycle
elseif get(handles.radiobutton4,'Value')==1.0
    set(handles.pedestrianParameters,'Visible','off');
    set(handles.vehicleParameters,'Visible','off');
    set(handles.roadParameters,'Visible','off');
    set(handles.roadInformation,'Visible','off');
    set(handles.squareInformation,'Visible','Off');
    set(handles.roundaboutInformation,'Visible','Off');
    set(handles.scenarioPanel,'Visible','off');
    set(handles.objectPanal,'Visible','off');
    set(handles.bicycleParameters,'Visible','on');
    
    
elseif get(handles.radiobutton5,'Value')==1.0
    set(handles.pedestrianParameters,'Visible','off');
    set(handles.vehicleParameters,'Visible','off');
    set(handles.roadParameters,'Visible','off');
    set(handles.roadInformation,'Visible','off');
    set(handles.squareInformation,'Visible','Off');
    set(handles.roundaboutInformation,'Visible','Off');
    set(handles.scenarioPanel,'Visible','off');
    set(handles.bicycleParameters,'Visible','Off');
    set(handles.objectPanal,'Visible','On');
end



% --- Executes on button press in Edit_Graph.
function Edit_Graph_Callback(hObject, eventdata, handles)
% hObject    handle to Edit_Graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Callback for Edit button
global vehicleDatabase;
global bicycleDatabase;
global pedestrianDatabase;
size_bicycleDatabase = size(bicycleDatabase);
size_pedestrianDatabase = size(pedestrianDatabase);
k=waitforbuttonpress;

while k==0
    handle = gco;
    global currentHandle;
    currentHandle=gco;
    type=get(handle,'Type');
    
    if strcmp(type,'patch'); %%Check if the object is vehicle
        set(handles.pedestrianParameters,'Visible','off');
        set(handles.objectPanal,'Visible','off');
        set(handles.bicycleParameters,'Visible','off');
        set(handles.roadParameters,'Visible','off');
        set(handles.roadInformation,'Visible','off');
        set(handles.squareInformation,'Visible','Off');
        set(handles.roundaboutInformation,'Visible','Off');
        set(handles.scenarioPanel,'Visible','off')
        sizeVehDatabase = size(vehicleDatabase);
        for i=1:sizeVehDatabase(2)
            if handle == vehicleDatabase(1,i).handle;
                
                set(handles.roadParameters,'Visible','off');
                set(handles.roadInformation,'Visible','off');
                set(handles.squareInformation,'Visible','Off');
                set(handles.roundaboutInformation,'Visible','Off');
                set(handles.scenarioPanel,'Visible','off');
                set(handles.objectPanal,'Visible','off');
                set(handles.bicycleParameters,'Visible','off');
                set(handles.vehicleParameters,'Visible','on');
                set(handles.vehicleSpeed,'String',vehicleDatabase(1,i).speed);
                set(handles.lateralAccn,'String',vehicleDatabase(1,i).lateralAccn);
                set(handles.longitudinalAccn,'String',vehicleDatabase(1,i).longitudinalAccn);
                set(handles.vehicleMass,'String',vehicleDatabase(1,i).vehicleMass);
                set(handles.heightCG,'String',vehicleDatabase(1,i).heightCG);
                set(handles.lengthVehicle,'String',vehicleDatabase(1,i).lengthVehicle);
                set(handles.widthVehicle,'String',vehicleDatabase(1,i).widthVehicle);
                set(handles.lf,'String',vehicleDatabase(1,i).lf);
                set(handles.lr,'String',vehicleDatabase(1,i).lr);
                set(handles.lfFront,'String',vehicleDatabase(1,i).lfToFront);
                set(handles.lrRear,'String',vehicleDatabase(1,i).lrToRear);
                set(handles.steeringRatio,'String',vehicleDatabase(1,i).steeringRatio);
                angle=vehicleDatabase(1,i).angleDir*180/pi;
                set(handles.vehicleDiretion,'String',angle);
                % set(handles.vehicleHandel,'String',vehicleDatabase(1,i).handle);
                set(handles.xLocationVehicle,'String',vehicleDatabase(1,i).xCG);
                set(handles.yLocationVehicle,'String',vehicleDatabase(1,i).yCG);
                set(handles.cl_fl,'String',vehicleDatabase(1,i).cl_fl);
                set(handles.cl_fr,'String',vehicleDatabase(1,i).cl_fr);
                set(handles.cl_rl,'String',vehicleDatabase(1,i).cl_rl);
                set(handles.cl_rr,'String',vehicleDatabase(1,i).cl_rr);
                set(handles.bl_fl,'String',vehicleDatabase(1,i).bl_fl);
                set(handles.bl_fr,'String',vehicleDatabase(1,i).bl_fr);
                set(handles.bl_rl,'String',vehicleDatabase(1,i).bl_rl);
                set(handles.bl_rr,'String',vehicleDatabase(1,i).bl_rr);
                set(handles.cs_fl,'String',vehicleDatabase(1,i).cs_fl);
                set(handles.cs_fr,'String',vehicleDatabase(1,i).cs_fr);
                set(handles.cs_rl,'String',vehicleDatabase(1,i).cs_rl);
                set(handles.cs_rr,'String',vehicleDatabase(1,i).cs_rr);
                set(handles.bs_fl,'String',vehicleDatabase(1,i).bs_fl);
                set(handles.bs_fr,'String',vehicleDatabase(1,i).bs_fr);
                set(handles.bs_rl,'String',vehicleDatabase(1,i).bs_rl);
                set(handles.bs_rr,'String',vehicleDatabase(1,i).bs_rr);
            end
        end
        
        for i=1:size_bicycleDatabase(2)
            if handle == bicycleDatabase(1,i).handle
                set(handles.roadParameters,'Visible','off');
                set(handles.roadInformation,'Visible','off');
                set(handles.squareInformation,'Visible','Off');
                set(handles.roundaboutInformation,'Visible','Off');
                set(handles.scenarioPanel,'Visible','off');
                set(handles.objectPanal,'Visible','off');
                set(handles.vehicleParameters,'Visible','off');
                set(handles.bicycleParameters,'Visible','on');
                set(handles.speedBicycle,'String',bicycleDatabase(1,i).speed);
                set(handles.lateralAccnBicycle,'String',bicycleDatabase(1,i).lateralAccn);
                set(handles.longAccnBicycle,'String',bicycleDatabase(1,i).longitudinalAccn);
                set(handles.xLocBicycle,'String',bicycleDatabase(1,i).xCG);
                set(handles.YLocBicycle,'String',bicycleDatabase(1,i).yCG);
                set(handles.massBicycle,'String',bicycleDatabase(1,i).vehicleMass);
                set(handles.heightCGBicycle,'String',bicycleDatabase(1,i).heightCG);
                set(handles.lengthBicycle,'String',bicycleDatabase(1,i).lengthVehicle);
                set(handles.widthBicycle,'String',bicycleDatabase(1,i).widthVehicle);
                set(handles.steeringRatioBicycle,'String',bicycleDatabase(1,i).steeringRatio);
                set(handles.lateralAccnBicycle,'String',bicycleDatabase(1,i).lateralAccn);
                set(handles.angleDirBicycle,'String',bicycleDatabase(1,i).angleDir);
                set(handles.handleBicycle,'String',bicycleDatabase(1,i).handle);
            end
        end
    elseif strcmp(type,'rectangle') % check if the object is pedestrian
        set(handles.roadParameters,'Visible','off');
        set(handles.roadInformation,'Visible','off');
        set(handles.squareInformation,'Visible','Off');
        set(handles.roundaboutInformation,'Visible','Off');
        set(handles.scenarioPanel,'Visible','off');
        set(handles.vehicleParameters,'Visible','off');
        set(handles.objectPanal,'Visible','off');
        set(handles.bicycleParameters,'Visible','off');
        set(handles.pedestrianParameters,'Visible','on');
        for i=1;size_pedestrianDatabase(2)
            if handle == pedestrianDatabase(1,i).handle;
                set(handles.speedPedestrian,'String',pedestrianDatabase(1,i).speed);
                set(handles.accnPedestrian,'String',pedestrianDatabase(1,i).accn);
                set(handles.xLocationPedestrian,'String',pedestrianDatabase(1,i).xPos);
                set(handles.yLocationPedestrian,'String',pedestrianDatabase(1,i).yPos);
                set(handles.dirPedestrian,'String',pedestrianDatabase(1,i).angleDir);
                set(handles.handlePedestrian,'String',pedestrianDatabase(1,i).handle);
                set(handles.maxSpeedPedestrian,'String',pedestrianDatabase(1,i).maxSpeed);
                set(handles.minSpeedPedestrian,'String',pedestrianDatabase(1,i).minSpeed);
            end
        end
        
    end
    k=1;
end


% --- Executes during radiobutton5 creation, after setting all properties.
function scenarioName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scenarioName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during radiobutton5 creation, after setting all properties.
function uipanel2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during radiobutton5 creation, after setting all properties.
function vehicleParameters_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vehicleParameters (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during radiobutton5 creation, after setting all properties.
function uipanel13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during radiobutton5 creation, after setting all properties.
function vehicleSpeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vehicleSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during radiobutton5 creation, after setting all properties.
function lateralAccn_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lateralAccn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during radiobutton5 creation, after setting all properties.
function lf_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vehicleSpeed_Callback(hObject, eventdata, handles)
% hObject    handle to vehicleSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vehicleSpeed as text
%        str2double(get(hObject,'String')) returns contents of vehicleSpeed as a double



function lf_Callback(hObject, eventdata, handles)
% hObject    handle to lf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lf as text
%        str2double(get(hObject,'String')) returns contents of lf as a double



function lateralAccn_Callback(hObject, eventdata, handles)
% hObject    handle to lateralAccn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lateralAccn as text
%        str2double(get(hObject,'String')) returns contents of lateralAccn as a double



function longitudinalAccn_Callback(hObject, eventdata, handles)
% hObject    handle to longitudinalAccn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of longitudinalAccn as text
%        str2double(get(hObject,'String')) returns contents of longitudinalAccn as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function longitudinalAccn_CreateFcn(hObject, eventdata, handles)
% hObject    handle to longitudinalAccn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vehicleMass_Callback(hObject, eventdata, handles)
% hObject    handle to vehicleMass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vehicleMass as text
%        str2double(get(hObject,'String')) returns contents of vehicleMass as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function vehicleMass_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vehicleMass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function heightCG_Callback(hObject, eventdata, handles)
% hObject    handle to heightCG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of heightCG as text
%        str2double(get(hObject,'String')) returns contents of heightCG as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function heightCG_CreateFcn(hObject, eventdata, handles)
% hObject    handle to heightCG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lengthVehicle_Callback(hObject, eventdata, handles)
% hObject    handle to lengthVehicle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lengthVehicle as text
%        str2double(get(hObject,'String')) returns contents of lengthVehicle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lengthVehicle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lengthVehicle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function widthVehicle_Callback(hObject, eventdata, handles)
% hObject    handle to widthVehicle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of widthVehicle as text
%        str2double(get(hObject,'String')) returns contents of widthVehicle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function widthVehicle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to widthVehicle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lr_Callback(hObject, eventdata, handles)
% hObject    handle to lr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lr as text
%        str2double(get(hObject,'String')) returns contents of lr as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lfFront_Callback(hObject, eventdata, handles)
% hObject    handle to lfFront (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lfFront as text
%        str2double(get(hObject,'String')) returns contents of lfFront as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lfFront_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lfFront (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lrRear_Callback(hObject, eventdata, handles)
% hObject    handle to lrRear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lrRear as text
%        str2double(get(hObject,'String')) returns contents of lrRear as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lrRear_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lrRear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function steeringRatio_Callback(hObject, eventdata, handles)
% hObject    handle to steeringRatio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of steeringRatio as text
%        str2double(get(hObject,'String')) returns contents of steeringRatio as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function steeringRatio_CreateFcn(hObject, eventdata, handles)
% hObject    handle to steeringRatio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vehicleDiretion_Callback(hObject, eventdata, handles)
% hObject    handle to vehicleDiretion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vehicleDiretion as text
%        str2double(get(hObject,'String')) returns contents of vehicleDiretion as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function vehicleDiretion_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vehicleDiretion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in saveVehicleState.
function saveVehicleState_Callback(hObject, eventdata, handles)
% hObject    handle to saveVehicleState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vehicleDatabase;
global currentHandle;

size_veh_data=size(vehicleDatabase);
%% saving all changed parameter values in vehicle Database
for i=1:size_veh_data(2)
    if (vehicleDatabase(1,i).handle==currentHandle)
        vehicleDatabase(1,i).speed = str2double(get(handles.vehicleSpeed,'String'));
        vehicleDatabase(1,i).lateralAccn =str2double( get(handles.lateralAccn,'String'));
        vehicleDatabase(1,i).longitudinalAccn = str2double(get(handles.longitudinalAccn,'String'));
        vehicleDatabase(1,i).vehicleMass = str2double( get(handles.vehicleMass,'String'));
        vehicleDatabase(1,i).heightCG = str2double(get(handles.heightCG,'String'));
        vehicleDatabase(1,i).lengthVehicle = str2double(get(handles.lengthVehicle,'String'));
        vehicleDatabase(1,i).widthVehicle = str2double(get(handles.widthVehicle,'String'));
        vehicleDatabase(1,i).lf = str2double(get(handles.lf,'String'));
        vehicleDatabase(1,i).lr = str2double(get(handles.lr,'String'));
        vehicleDatabase(1,i).lfToFront = str2double(get(handles.lfFront,'String'));
        vehicleDatabase(1,i).lrToRear = str2double(get(handles.lrRear,'String'));
        vehicleDatabase(1,i).steeringRatio = str2double(get(handles.steeringRatio,'String'));
        vehicleDatabase(1,i).xCG = str2double(get(handles.xLocationVehicle,'String'));
        vehicleDatabase(1,i).yCG = str2double(get(handles.yLocationVehicle,'String'));
        vehicleDatabase(1,i).cl_fl = str2double(get(handles.cl_fl,'String'));
        vehicleDatabase(1,i).cl_fr = str2double(get(handles.cl_fr,'String'));
        vehicleDatabase(1,i).cl_rl = str2double(get(handles.cl_rl,'String'));
        vehicleDatabase(1,i).cl_rr = str2double(get(handles.cl_rr,'String'));
        vehicleDatabase(1,i).bl_fl = str2double(get(handles.bl_fl,'String'));
        vehicleDatabase(1,i).bl_fr = str2double(get(handles.bl_fr,'String'));
        vehicleDatabase(1,i).bl_rl = str2double(get(handles.bl_rl,'String'));
        vehicleDatabase(1,i).bl_rr = str2double(get(handles.bl_rr,'String'));
        vehicleDatabase(1,i).cs_fl = str2double(get(handles.cs_fl,'String'));
        vehicleDatabase(1,i).cs_fr = str2double(get(handles.cs_fr,'String'));
        vehicleDatabase(1,i).cs_rl = str2double(get(handles.cs_rl,'String'));
        vehicleDatabase(1,i).cs_rr = str2double(get(handles.cs_rr,'String'));
        vehicleDatabase(1,i).bs_fl = str2double(get(handles.bs_fl,'String'));
        vehicleDatabase(1,i).bs_fr = str2double(get(handles.bs_fr,'String'));
        vehicleDatabase(1,i).bs_rl = str2double(get(handles.bs_rl,'String'));
        vehicleDatabase(1,i).bs_rr = str2double(get(handles.bs_rr,'String'));
        vehicleDatabase(1,i).angleDir=(str2double(get(handles.vehicleDiretion,'String'))*pi/180);
        
        vehicleDatabase(1,i).signalLongiController=get(handles.signalController,'Value');
        vehicleDatabase(1,i).otherObjLongiController=get(handles.longitudinalController,'Value');
        %% Rotate the vehicle with calculating the unit vector
        
        ex=cos((str2double(get(handles.vehicleDiretion,'String')))*pi/180);
        ey=sin((str2double(get(handles.vehicleDiretion,'String')))*pi/180);
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



function vehicleHandel_Callback(hObject, eventdata, handles)
% hObject    handle to vehicleHandel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vehicleHandel as text
%        str2double(get(hObject,'String')) returns contents of vehicleHandel as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function vehicleHandel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vehicleHandel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cl_fl_Callback(hObject, eventdata, handles)
% hObject    handle to cl_fl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cl_fl as text
%        str2double(get(hObject,'String')) returns contents of cl_fl as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function cl_fl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cl_fl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cs_fl_Callback(hObject, eventdata, handles)
% hObject    handle to cs_fl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cs_fl as text
%        str2double(get(hObject,'String')) returns contents of cs_fl as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function cs_fl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cs_fl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bl_fl_Callback(hObject, eventdata, handles)
% hObject    handle to bl_fl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bl_fl as text
%        str2double(get(hObject,'String')) returns contents of bl_fl as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function bl_fl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bl_fl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bs_fl_Callback(hObject, eventdata, handles)
% hObject    handle to bs_fl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bs_fl as text
%        str2double(get(hObject,'String')) returns contents of bs_fl as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function bs_fl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bs_fl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cl_fr_Callback(hObject, eventdata, handles)
% hObject    handle to cl_fr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cl_fr as text
%        str2double(get(hObject,'String')) returns contents of cl_fr as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function cl_fr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cl_fr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cs_fr_Callback(hObject, eventdata, handles)
% hObject    handle to cs_fr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cs_fr as text
%        str2double(get(hObject,'String')) returns contents of cs_fr as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function cs_fr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cs_fr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bl_fr_Callback(hObject, eventdata, handles)
% hObject    handle to bl_fr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bl_fr as text
%        str2double(get(hObject,'String')) returns contents of bl_fr as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function bl_fr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bl_fr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bs_fr_Callback(hObject, eventdata, handles)
% hObject    handle to bs_fr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bs_fr as text
%        str2double(get(hObject,'String')) returns contents of bs_fr as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function bs_fr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bs_fr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cl_rl_Callback(hObject, eventdata, handles)
% hObject    handle to cl_rl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cl_rl as text
%        str2double(get(hObject,'String')) returns contents of cl_rl as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function cl_rl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cl_rl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cs_rl_Callback(hObject, eventdata, handles)
% hObject    handle to cs_rl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cs_rl as text
%        str2double(get(hObject,'String')) returns contents of cs_rl as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function cs_rl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cs_rl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bl_rl_Callback(hObject, eventdata, handles)
% hObject    handle to bl_rl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bl_rl as text
%        str2double(get(hObject,'String')) returns contents of bl_rl as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function bl_rl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bl_rl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bs_rl_Callback(hObject, eventdata, handles)
% hObject    handle to bs_rl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bs_rl as text
%        str2double(get(hObject,'String')) returns contents of bs_rl as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function bs_rl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bs_rl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cl_rr_Callback(hObject, eventdata, handles)
% hObject    handle to cl_rr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cl_rr as text
%        str2double(get(hObject,'String')) returns contents of cl_rr as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function cl_rr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cl_rr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cs_rr_Callback(hObject, eventdata, handles)
% hObject    handle to cs_rr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cs_rr as text
%        str2double(get(hObject,'String')) returns contents of cs_rr as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function cs_rr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cs_rr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bl_rr_Callback(hObject, eventdata, handles)
% hObject    handle to bl_rr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bl_rr as text
%        str2double(get(hObject,'String')) returns contents of bl_rr as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function bl_rr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bl_rr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bs_rr_Callback(hObject, eventdata, handles)
% hObject    handle to bs_rr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bs_rr as text
%        str2double(get(hObject,'String')) returns contents of bs_rr as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function bs_rr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bs_rr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton1



function scalingUnit_Callback(hObject, eventdata, handles)
% hObject    handle to scalingUnit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of scalingUnit as text
%        str2double(get(hObject,'String')) returns contents of scalingUnit as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function scalingUnit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scalingUnit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in changeScaling.
function changeScaling_Callback(hObject, eventdata, handles)
% hObject    handle to changeScaling (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

existVehicles = findall(gcf,'Type','patch');
noVehicles = size(existVehicles);

global vehicleDatabase;
for i=1:noVehicles(1)
    ex=cos((vehicleDatabase(1,i).angleDir)*pi/180);
    ey=sin((vehicleDatabase(1,i).angleDir)*pi/180);
    exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
    eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
    scalingUnit = str2double(get(handles.scalingUnit,'String'));
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
    
    set(vehicleDatabase(1,i).handle,'xData',x_Data,'yData',y_Data);
end






function xDimensionMax_Callback(hObject, eventdata, handles)
% hObject    handle to xDimensionMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xDimensionMax as text
%        str2double(get(hObject,'String')) returns contents of xDimensionMax as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function xDimensionMax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xDimensionMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xDimensionMin_Callback(hObject, eventdata, handles)
% hObject    handle to xDimensionMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xDimensionMin as text
%        str2double(get(hObject,'String')) returns contents of xDimensionMin as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function xDimensionMin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xDimensionMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in changeAxis.
function changeAxis_Callback(hObject, eventdata, handles)
% hObject    handle to changeAxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Change the size of the axes
axes= findall(gcf,'Type','axes');
xDimensionMin=str2double(get(handles.xDimensionMin,'String'));
xDimensionMax=str2double(get(handles.xDimensionMax,'String'));
yDimensionMin=str2double(get(handles.yDimensionMin,'String'));
yDimensionMax=str2double(get(handles.yDimensionMax,'String'));
axis(axes,[xDimensionMin,xDimensionMax,yDimensionMin,yDimensionMax]);



function widthTrack_Callback(hObject, eventdata, handles)
% hObject    handle to widthTrack (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of widthTrack as text
%        str2double(get(hObject,'String')) returns contents of widthTrack as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function widthTrack_CreateFcn(hObject, eventdata, handles)
% hObject    handle to widthTrack (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function noOuterTracks_Callback(hObject, eventdata, handles)
% hObject    handle to noOuterTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of noOuterTracks as text
%        str2double(get(hObject,'String')) returns contents of noOuterTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function noOuterTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noOuterTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function noInnerTracks_Callback(hObject, eventdata, handles)
% hObject    handle to noInnerTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of noInnerTracks as text
%        str2double(get(hObject,'String')) returns contents of noInnerTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function noInnerTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noInnerTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function frictionCoefficient_Callback(hObject, eventdata, handles)
% hObject    handle to frictionCoefficient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of frictionCoefficient as text
%        str2double(get(hObject,'String')) returns contents of frictionCoefficient as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function frictionCoefficient_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frictionCoefficient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function rcb1(~,~)
%% Contextmenu item for deleting the road
global roadDatabase;
size_roadDatabase = size(roadDatabase);
for i=1:size_roadDatabase(2)
    if roadDatabase(1,i).mainHandle ==gco;
        delete gco;
        delete(roadDatabase(1,i).mainHandle);
        delete(roadDatabase(1,i).innerRoadHandles);
        delete(roadDatabase(1,i).outerRoadHandles);
        delete(roadDatabase(1,i).innerTrackHandles);
        delete(roadDatabase(1,i).outerTrackHandles);
    end
end


% --- Executes on button press in newRoad.
function newRoad_Callback(hObject, eventdata, handles)
% hObject    handle to newRoad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

axes  = findall(gcf,'Type','axes');  %%%finding all the axes in gui\
% Choose default command line output for gui
handles.output = hObject;
oldhold = ishold(gca);
hold on;
k=0;
x=[];
y=[];
while k==0
    
    set(gcf,'Pointer','crosshair','doublebuffer','on');
    [xs,ys] = ginput(1);
    plot(xs,ys,'ro');
    hold on;
    % if enter is pressed xs is empty and it is an indication to stop
    % taking points
    if (isempty(xs))
        k=1;
        set(gcf,'Pointer','arrow');
        break;
    end
    x(end+1)=xs;
    y(end+1)=ys;
end
x=x';
y=y';
global roadDatabase;
global kreuzungDatabase;
global kreiselDatabase;
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
        CLF = hypot(diff(xfit), diff(yfit));    % Calculate integrand from x,y derivatives c = sqrt(abs(a).^2 + abs(b).^2)
        CL = trapz(CLF);                          % Integrate to calculate arc length
        scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
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
        scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
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
        scalingUnit = 1;%str2double(get(handles.scalingUnit,'String'));
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
                scalingUnit = 1;%str2double(get(handles.scalingUnit,'String'));
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
                scalingUnit = str2double(get(handles.scalingUnit,'String'));
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
                scalingUnit = 1;%str2double(get(handles.scalingUnit,'String'));
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
            scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
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
            scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
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
                scalingUnit = 1;%str2double(get(handles.scalingUnit,'String'));
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
    end
    
    
end


%%offset for parallel curves
d=str2double(get(handles.widthTrack,'String'));

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

noInnerCurve = str2double(get(handles.noInnerTracks,'String'));
noOuterCurve = str2double(get(handles.noOuterTracks,'String'));

% % determine radius of curvature
R=(dx.^2+dy.^2).^(3/2)./abs(dx.*dy2-dy.*dx2);

% % Determine overlap points for inner normal curve
overlap= (R < noInnerCurve * d) ;
underlap = (R < noOuterCurve * d);

%Contextmenu

rcmenu= uicontextmenu;
r1=uimenu(rcmenu,'label','Delete','callback',@rcb1);

if (any(overlap) || any(underlap))
    errordlg('The track width is too large for given road profile.')
else
    hold on;
    roadDatabase(1,sizeRoadDatabase(2)+1).endPointsSlope = endPointsSlope;
    roadDatabase(1,sizeRoadDatabase(2)+1).xEndPoints = [x(1) x(end)];
    roadDatabase(1,sizeRoadDatabase(2)+1).yEndPoints = [y(1) y(end)];
    roadDatabase(1,sizeRoadDatabase(2)+1).mainHandle = plot(xRoadPoints,yRoadPoints,'b','Linewidth',2);
    roadDatabase(1,sizeRoadDatabase(2)+1).widthTrack = str2double(get(handles.widthTrack,'String'));
    roadDatabase(1,sizeRoadDatabase(2)+1).mainXRoadPoints = xRoadPoints;
    roadDatabase(1,sizeRoadDatabase(2)+1).mainYRoadPoints = yRoadPoints;
    roadDatabase(1,sizeRoadDatabase(2)+1).endPointsSlope(2)=slope_endpt;
    set(roadDatabase(1,sizeRoadDatabase(2)+1).mainHandle,'uicontextmenu',rcmenu);
    
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






% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vehicleDatabase;
global roadDatabase;

size_vehicleDatabase = size(vehicleDatabase);
size_roadDatabase = size(roadDatabase);

for i=1:size_roadDatabase(2)
    size_mainXRoadPoints = size(roadDatabase(1,i).mainXRoadPoints);
    
    for k=1:size_mainXRoadPoints
        %% Checking points for collision detection within the range of vehicles for end tracks
        for j=1:size_vehicleDatabase(2)
            safeDistance_mainRoad= sqrt((abs(roadDatabase(1,i).mainXRoadPoints(k) - vehicleDatabase(1,j).xCenter)).^2 + ...
                (abs(roadDatabase(1,i).mainYRoadPoints(k) - vehicleDatabase(1,j).yCenter)).^2);
            if (safeDistance_mainRoad < vehicleDatabase(1,j).radiusCollisionControl)
                plot (roadDatabase(1,i).mainXRoadPoints(k),roadDatabase(1,i).mainYRoadPoints(k),'yo');
            end
            safeDistance_innerEnd = sqrt((abs(roadDatabase(1,i).xInnerEndPoints(k) - vehicleDatabase(1,j).xCenter)).^2 + ...
                (abs(roadDatabase(1,i).yInnerCurvePoints(k) - vehicleDatabase(1,j).yCenter)).^2);
            if (safeDistance_innerEnd < vehicleDatabase(1,j).radiusCollisionControl)
                plot (roadDatabase(1,i).xInnerEndPoints(k),roadDatabase(1,i).yInnerEndPoints(k),'yo');
            end
            safeDistance_outerEnd = sqrt((abs(roadDatabase(1,i).xOuterEndPoints(k) - vehicleDatabase(1,j).xCenter)).^2 + ...
                (abs(roadDatabase(1,i).yOuterEndPoints(k) - vehicleDatabase(1,j).yCenter)).^2);
            if (safeDistance_outerEnd < vehicleDatabase(1,j).radiusCollisionControl)
                plot (roadDatabase(1,i).xOuterEndPoints(k),roadDatabase(1,i).yOuterEndPoints(k),'yo');
            end
            
            
            
            size_xInnerCurvePoints = size(roadDatabase(1,i).xInnerCurvePoints);
            size_xOuterCurvePoints = size(roadDatabase(1,i).xOuterCurvePoints);
            %% Checking points for collision detection within the range of vehicles for inner tracks
            for l=1:size_xInnerCurvePoints(1)
                safeDistance_innerCurve = sqrt((abs(roadDatabase(1,i).xInnerCurvePoints(l,k) - vehicleDatabase(1,j).xCenter)).^2 + ...
                    (abs(roadDatabase(1,i).yInnerCurvePoints(l,k) - vehicleDatabase(1,j).yCenter)).^2);
                if (safeDistance_innerCurve < vehicleDatabase(1,j).radiusCollisionControl)
                    plot (roadDatabase(1,i).xInnerCurvePoints(l,k),roadDatabase(1,i).yInnerCurvePoints(l,k),'bo');
                end
            end
            
            for l=1:size_xOuterCurvePoints(1)
                safeDistance_outerCurve = sqrt((abs(roadDatabase(1,i).xOuterCurvePoints(l,k) - vehicleDatabase(1,j).xCenter)).^2 + ...
                    (abs(roadDatabase(1,i).yOuterCurvePoints(l,k) - vehicleDatabase(1,j).yCenter)).^2);
                if (safeDistance_outerCurve < vehicleDatabase(1,j).radiusCollisionControl)
                    plot (roadDatabase(1,i).xOuterCurvePoints(l,k),roadDatabase(1,i).yOuterCurvePoints(l,k),'bo');
                end
            end
        end
    end
end







% --- Executes on button press in saveScenario.
function saveScenario_Callback(hObject, eventdata, handles)
% hObject    handle to saveScenario (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global roadDatabase;
global vehicleDatabase;
global kreuzungDatabase;
global kreiselDatabase;
global pedestrianDatabase;
global bicycleDatabase;
global collisionTime;
global isCollisionDetected;
global objectDatabase;

uisave({'roadDatabase','vehicleDatabase','kreuzungDatabase', 'kreiselDatabase','pedestrianDatabase', 'bicycleDatabase','collisionTime','isCollisionDetected','objectDatabase'});



function squareAngle_Callback(hObject, eventdata, handles)
% hObject    handle to squareAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of squareAngle as text
%        str2double(get(hObject,'String')) returns contents of squareAngle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function squareAngle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to squareAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function squareDimention_Callback(hObject, eventdata, handles)
% hObject    handle to squareDimention (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of squareDimention as text
%        str2double(get(hObject,'String')) returns contents of squareDimention as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function squareDimention_CreateFcn(hObject, eventdata, handles)
% hObject    handle to squareDimention (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function noToTracks_Callback(hObject, eventdata, handles)
% hObject    handle to noToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of noToTracks as text
%        str2double(get(hObject,'String')) returns contents of noToTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function noToTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function upperRoadToTracks_Callback(hObject, eventdata, handles)
% hObject    handle to upperRoadToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of upperRoadToTracks as text
%        str2double(get(hObject,'String')) returns contents of upperRoadToTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function upperRoadToTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to upperRoadToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function leftRoadToTracks_Callback(hObject, eventdata, handles)
% hObject    handle to leftRoadToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of leftRoadToTracks as text
%        str2double(get(hObject,'String')) returns contents of leftRoadToTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function leftRoadToTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to leftRoadToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lowerRoadToTracks_Callback(hObject, eventdata, handles)
% hObject    handle to lowerRoadToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lowerRoadToTracks as text
%        str2double(get(hObject,'String')) returns contents of lowerRoadToTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lowerRoadToTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lowerRoadToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function noAwayTracks_Callback(hObject, eventdata, handles)
% hObject    handle to noAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of noAwayTracks as text
%        str2double(get(hObject,'String')) returns contents of noAwayTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function noAwayTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function upperRoadAwayTracks_Callback(hObject, eventdata, handles)
% hObject    handle to upperRoadAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of upperRoadAwayTracks as text
%        str2double(get(hObject,'String')) returns contents of upperRoadAwayTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function upperRoadAwayTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to upperRoadAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function leftRoadAwayTracks_Callback(hObject, eventdata, handles)
% hObject    handle to leftRoadAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of leftRoadAwayTracks as text
%        str2double(get(hObject,'String')) returns contents of leftRoadAwayTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function leftRoadAwayTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to leftRoadAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lowerRoadAwayTracks_Callback(hObject, eventdata, handles)
% hObject    handle to lowerRoadAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lowerRoadAwayTracks as text
%        str2double(get(hObject,'String')) returns contents of lowerRoadAwayTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lowerRoadAwayTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lowerRoadAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function trackWidth_Callback(hObject, eventdata, handles)
% hObject    handle to trackWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of trackWidth as text
%        str2double(get(hObject,'String')) returns contents of trackWidth as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function trackWidth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trackWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function upperTrackWidth_Callback(hObject, eventdata, handles)
% hObject    handle to upperTrackWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of upperTrackWidth as text
%        str2double(get(hObject,'String')) returns contents of upperTrackWidth as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function upperTrackWidth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to upperTrackWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function leftTrackWidth_Callback(hObject, eventdata, handles)
% hObject    handle to leftTrackWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of leftTrackWidth as text
%        str2double(get(hObject,'String')) returns contents of leftTrackWidth as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function leftTrackWidth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to leftTrackWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lowerTrackWidth_Callback(hObject, eventdata, handles)
% hObject    handle to lowerTrackWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lowerTrackWidth as text
%        str2double(get(hObject,'String')) returns contents of lowerTrackWidth as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lowerTrackWidth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lowerTrackWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in newSquare.
function newSquare_Callback(hObject, eventdata, handles)
% hObject    handle to newSquare (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes  = findall(gcf,'Type','axes');  %%%finding all the axes in gui\

hold on;
set(gcf,'Pointer','crosshair','doublebuffer','on');

[x,y] = ginput(1); % taking input reference point
global kreuzungDatabase;
size_kreuzungDatabase = size(kreuzungDatabase);

% Adding to the kreuzungDatabase
if isempty(kreuzungDatabase)
    kreuzungDatabase = Kreuzung;
else
    kreuzungDatabase(1,size_kreuzungDatabase(2)+1) = Kreuzung;
end

%% input data for junction
kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle = str2double(regexp(get(handles.squareAngle,'String'),' ','split'))*pi/180;
kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trackWidth = str2double(regexp(get(handles.trackWidth,'String'),' ','split'));
kreuzungDatabase(1,size_kreuzungDatabase(2)+1).noRoadToTracks =str2double(regexp(get(handles.noToTracks,'String'),' ','split'));
kreuzungDatabase(1,size_kreuzungDatabase(2)+1).noRoadAwayTracks =str2double(regexp(get(handles.noAwayTracks,'String'),' ','split'));

noSquareRoads = size(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle);

kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareDim =str2double(get(handles.squareDimention,'String'));
kreuzungDatabase(1,size_kreuzungDatabase(2)+1).signalTime =str2double(get(handles.signalTime,'String'));

for j=1:noSquareRoads(2)
    % slope of end points of Kreuzung
    kreuzungDatabase(1,size_kreuzungDatabase(2)+1).endPointsSlope(j) = tan(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(j));
    
    % Finding unit vectors in direction of angle
    ex=cos(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(j));
    ey=sin(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(j));
    exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
    eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
    
    
    XrightRoad = [x(1)+ex*kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareDim x(1)+ex*(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareDim+5)];
    YrightRoad = [y(1)+ey*kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareDim y(1)+ey*(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareDim+5)];
    kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadPoints(j,:) = linspace(XrightRoad(1),XrightRoad(2),25);
    kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadPoints(j,:)= linspace(YrightRoad(1),YrightRoad(2),25);
    kreuzungDatabase(1,size_kreuzungDatabase(2)+1).xEndPoints(j) = XrightRoad(end);
    kreuzungDatabase(1,size_kreuzungDatabase(2)+1).yEndPoints(j) = YrightRoad(end);
    kreuzungDatabase(1,size_kreuzungDatabase(2)+1).mainRoadHandles(j) = line(XrightRoad,YrightRoad);
    %% Drawing the other roads
    for i=1:kreuzungDatabase(1,size_kreuzungDatabase(2)+1).noRoadToTracks(j)
        XRoadToTracks=XrightRoad - exOrtho*(i*kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trackWidth(j));
        YRoadToTracks=YrightRoad - eyOrtho*(i*kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trackWidth(j));
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadToTracksPoints(j,i,:) = linspace(XRoadToTracks(1),XRoadToTracks(2),25);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadToTracksPoints(j,i,:) = linspace(YRoadToTracks(1),YRoadToTracks(2),25);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).toTracksHandles(j,i)=line(XRoadToTracks,YRoadToTracks,'color','r');
        
        %% Tracks
        XInnerRoadToTracksPoints= XRoadToTracks +exOrtho*(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trackWidth(j)/2);
        YInnerRoadToTracksPoints =  YRoadToTracks +eyOrtho*(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trackWidth(j)/2);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XInnerRoadToTracksPoints(j,i,:) = linspace(XInnerRoadToTracksPoints(2),XInnerRoadToTracksPoints(1),25);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YInnerRoadToTracksPoints(j,i,:) = linspace(YInnerRoadToTracksPoints(2),YInnerRoadToTracksPoints(1),25);
        
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).innerRoadToTracksHandles(j,i) = plot (XInnerRoadToTracksPoints,YInnerRoadToTracksPoints,'-y');
        
        
        %% TrafficSignal
        if j==1
            if (i==1)
                kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trafficSignal(j,i) = line([kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadToTracksPoints(j,i,1) kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadPoints(j,1) ],...
                    [kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadToTracksPoints(j,i,1) kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadPoints(j,1)],'color','g','Linewidth',3);
            else
                kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trafficSignal(j,i) = line([kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadToTracksPoints(j,i,1) kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadToTracksPoints(j,i-1,1) ],...
                    [kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadToTracksPoints(j,i,1) kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadToTracksPoints(j,i-1,1)],'color','g','Linewidth',3);
            end
        else
            if (i==1)
                kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trafficSignal(j,i) = line([kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadToTracksPoints(j,i,1) kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadPoints(j,1) ],...
                    [kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadToTracksPoints(j,i,1) kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadPoints(j,1)],'color','r','Linewidth',3);
            else
                kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trafficSignal(j,i) = line([kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadToTracksPoints(j,i,1) kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadToTracksPoints(j,i-1,1) ],...
                    [kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadToTracksPoints(j,i,1) kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadToTracksPoints(j,i-1,1)],'color','r','Linewidth',3);
            end
        end
        if i==kreuzungDatabase(1,size_kreuzungDatabase(2)+1).noRoadToTracks(j)
            XlastRoadToTracks(j) = XRoadToTracks(1,1);
            YlastRoadToTracks (j)= YRoadToTracks(1,1);
        end
    end
    
    % Roads away from kreuzung
    for i=1:kreuzungDatabase(1,size_kreuzungDatabase(2)+1).noRoadAwayTracks(j);
        XRoadAwayTracks=XrightRoad + exOrtho*(i*kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trackWidth(j));
        YRoadAwayTracks=YrightRoad +eyOrtho*(i*kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trackWidth(j));
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XRoadAwayTracksPoints(j,i,:) = linspace(XRoadAwayTracks(1),XRoadAwayTracks(2),25);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YRoadAwayTracksPoints(j,i,:) = linspace(YRoadAwayTracks(1),YRoadAwayTracks(2),25);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).awayTrackHandles(j,i)= line(XRoadAwayTracks,YRoadAwayTracks,'color','g');
        
        %% Tracks
        XInnerRoadAwayTracksPoints= XRoadAwayTracks -exOrtho*(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trackWidth(j)/2);
        YInnerRoadAwayTracksPoints =  YRoadAwayTracks -eyOrtho*(kreuzungDatabase(1,size_kreuzungDatabase(2)+1).trackWidth(j)/2);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).XInnerRoadAwayTracksPoints(j,i,:) = linspace(XInnerRoadAwayTracksPoints(1),XInnerRoadAwayTracksPoints(2),25);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).YInnerRoadAwayTracksPoints(j,i,:) = linspace(YInnerRoadAwayTracksPoints(1),YInnerRoadAwayTracksPoints(2),25);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).innerRoadAwayTracksHandles(j,i) = plot (XInnerRoadAwayTracksPoints,YInnerRoadAwayTracksPoints,'-y');
        
        if i==kreuzungDatabase(1,size_kreuzungDatabase(2)+1).noRoadAwayTracks(j)
            XlastRoadAwayTracks (j)= XRoadAwayTracks(1,1);
            YlastRoadAwayTracks (j)= YRoadAwayTracks(1,1);
        end
    end
    
end

%% Joining the roads of junction
for i=1:noSquareRoads(2)
    
    if i==noSquareRoads(2)
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).xCornerPoints(i,:) = linspace(XlastRoadToTracks(i), XlastRoadAwayTracks(1),100);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).yCornerPoints(i,:) = linspace(YlastRoadToTracks(i), YlastRoadAwayTracks(1),100);
        %if angle of road is between 60-120 or 240 - 300 -> Y parabola
        %else X parabola
        if ((kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(i)>1.047 && kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(i)<2.094) ||...
                (kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(i)>4.188 && kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(i)<5.235))
            [new_xRoadPoints new_yRoadPoints slope_endpt]=YparabolaSpecPoints(XlastRoadToTracks(i),YlastRoadToTracks(i), XlastRoadAwayTracks(1), YlastRoadAwayTracks(1),kreuzungDatabase(1,size_kreuzungDatabase(2)+1).endPointsSlope(i),100);
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).xCornerPoints(i,:) = new_xRoadPoints;
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).yCornerPoints(i,:) = new_yRoadPoints;
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).kreuzungCornerHandles(i) = plot(new_xRoadPoints, new_yRoadPoints,'r-');
        else
            
            [new_xRoadPoints new_yRoadPoints slope_endpt]=XparabolaSpecPoints(XlastRoadToTracks(i),YlastRoadToTracks(i), XlastRoadAwayTracks(1), YlastRoadAwayTracks(1),kreuzungDatabase(1,size_kreuzungDatabase(2)+1).endPointsSlope(i),100);
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).xCornerPoints(i,:) = new_xRoadPoints;
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).yCornerPoints(i,:) = new_yRoadPoints;
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).kreuzungCornerHandles(i) = plot(new_xRoadPoints, new_yRoadPoints,'b-');
        end
        % line([XlastRoadToTracks(i) XlastRoadAwayTracks(1)], [YlastRoadToTracks(i) YlastRoadAwayTracks(1)]);
    else
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).xCornerPoints(i,:) = linspace(XlastRoadToTracks(i), XlastRoadAwayTracks(i+1),100);
        kreuzungDatabase(1,size_kreuzungDatabase(2)+1).yCornerPoints(i,:) = linspace(YlastRoadToTracks(i), YlastRoadAwayTracks(i+1),100);
        %if angle of road is between 60-120 or 240 - 300 -> Y parabola
        %else X parabola
        if ((kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(i)>1.047 && kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(i)<2.094) ||...
                (kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(i)>4.188 && kreuzungDatabase(1,size_kreuzungDatabase(2)+1).squareAngle(i)<5.235))
            [new_xRoadPoints new_yRoadPoints slope_endpt]=YparabolaSpecPoints(XlastRoadToTracks(i),YlastRoadToTracks(i), XlastRoadAwayTracks(i+1), YlastRoadAwayTracks(i+1),kreuzungDatabase(1,size_kreuzungDatabase(2)+1).endPointsSlope(i),100);
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).xCornerPoints(i,:) = new_xRoadPoints;
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).yCornerPoints(i,:) = new_yRoadPoints;
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).kreuzungCornerHandles(i)= plot(new_xRoadPoints, new_yRoadPoints,'r-');
        else
            
            [new_xRoadPoints new_yRoadPoints slope_endpt]=XparabolaSpecPoints(XlastRoadToTracks(i),YlastRoadToTracks(i), XlastRoadAwayTracks(i+1), YlastRoadAwayTracks(i+1),kreuzungDatabase(1,size_kreuzungDatabase(2)+1).endPointsSlope(i),100);
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).xCornerPoints(i,:) = new_xRoadPoints;
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).yCornerPoints(i,:) = new_yRoadPoints;
            kreuzungDatabase(1,size_kreuzungDatabase(2)+1).kreuzungCornerHandles(i)=plot(new_xRoadPoints, new_yRoadPoints,'b-');
        end
        % line([XlastRoadToTracks(i) XlastRoadAwayTracks(i+1)], [YlastRoadToTracks(i) YlastRoadAwayTracks(i+1)]);
    end
    
    
    
end







% --- Executes on button press in newKreisel.
function newKreisel_Callback(hObject, eventdata, handles)
% hObject    handle to newKreisel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes  = findall(gcf,'Type','axes');  %%%finding all the axes in gui\

hold on;
set(gcf,'Pointer','crosshair','doublebuffer','on');

global kreiselDatabase;
%Assigning Kreisel radiobutton5
size_kreiselDatabase = size(kreiselDatabase);
if isempty(kreiselDatabase)
    kreiselDatabase = Kreisel;
else
    kreiselDatabase(1,size_kreiselDatabase(2)+1) = Kreisel;
end

[x,y] = ginput(1);
kreiselDatabase(1,size_kreiselDatabase(2)+1).innerRadius = str2double(get(handles.innerRadiusKreisel,'String'));
innerCircleCurvature = 2*pi*kreiselDatabase(1,size_kreiselDatabase(2)+1).innerRadius;
noPoints = innerCircleCurvature/0.2; %points at 20 cm distance
kreiselDatabase(1,size_kreiselDatabase(2)+1).widthTrackKreisel = str2double(get(handles.widthKreisel,'String'));
kreiselDatabase(1,size_kreiselDatabase(2)+1).outerRadius = str2double(get(handles.outerRadiusKreisel,'String'));
inputAngles=(get(handles.roadAngles,'String'));
anglesArray = str2double(regexp((inputAngles),' ','split'));%convert the input to double
Angles = anglesArray;
size_anglesArray=size(anglesArray);
for i=1:size_anglesArray(2)
    kreiselDatabase(1,size_kreiselDatabase(2)+1).roadAngles(i)=anglesArray(i);
end

ang=linspace(0,2*pi,noPoints);
%inner circle points
xp = kreiselDatabase(1,size_kreiselDatabase(2)+1).innerRadius*cos(ang);
yp = kreiselDatabase(1,size_kreiselDatabase(2)+1).innerRadius*sin(ang);
%kreisel track points
xt = (kreiselDatabase(1,size_kreiselDatabase(2)+1).innerRadius + kreiselDatabase(1,size_kreiselDatabase(2)+1).widthTrackKreisel)*cos(ang);
yt = (kreiselDatabase(1,size_kreiselDatabase(2)+1).innerRadius + kreiselDatabase(1,size_kreiselDatabase(2)+1).widthTrackKreisel)*sin(ang);
kreiselDatabase(1,size_kreiselDatabase(2)+1).XpointsCircles = x+xp;
kreiselDatabase(1,size_kreiselDatabase(2)+1).YpointsCircles = y+yp;
kreiselDatabase(1,size_kreiselDatabase(2)+1).XpointsOuterCircle = x+xt;
kreiselDatabase(1,size_kreiselDatabase(2)+1).YpointsOuterCircle = y+yt;
plot(x+xt,y+yt,'-b');
plot(kreiselDatabase(1,size_kreiselDatabase(2)+1).XpointsCircles,kreiselDatabase(1,size_kreiselDatabase(2)+1).YpointsCircles,'Linewidth',2);
hold on;
% xo = outerRadius *cos(ang);
% yo = outerRadius*sin(ang);
% plot(x+xo,y+yo);
sizekreiselAngle = size(Angles);
%inputs for the roads of kreisel
kreiselDatabase(1,size_kreiselDatabase(2)+1).trackWidth = str2double(regexp(get(handles.trackWidthKreisel,'String'),' ','split'));
kreiselDatabase(1,size_kreiselDatabase(2)+1).toTracks = str2double(regexp(get(handles.kreiselToTracks,'String'),' ','split'));
kreiselDatabase(1,size_kreiselDatabase(2)+1).awayTracks = str2double(regexp(get(handles.kreiselAwayTracks,'String'),' ','split'));



for j=1:sizekreiselAngle(2)
    kreiselAngle=Angles(j)*pi/180;
    kreiselDatabase(1,size_kreiselDatabase(2)+1).endPointsSlope(j)=tan(kreiselAngle);
    
    %unit vectors in road direction
    ex=cos(kreiselAngle);
    ey=sin(kreiselAngle);
    exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
    eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
    
    XmainRoad(j,:) = [x(1)+ex*kreiselDatabase(1,size_kreiselDatabase(2)+1).outerRadius x(1)+ex*(kreiselDatabase(1,size_kreiselDatabase(2)+1).outerRadius+5)];
    YmainRoad(j,:)= [y(1)+ey*kreiselDatabase(1,size_kreiselDatabase(2)+1).outerRadius y(1)+ey*(kreiselDatabase(1,size_kreiselDatabase(2)+1).outerRadius+5)];
    % Points at 20 cm
    kreiselDatabase(1,size_kreiselDatabase(2)+1).XRoadPoints(j,:) = linspace(XmainRoad(j,1),XmainRoad(j,2),25);
    kreiselDatabase(1,size_kreiselDatabase(2)+1).YRoadPoints(j,:) = linspace(YmainRoad(j,1),YmainRoad(j,2),25);
    %End points of kreisel to connect to other roads
    kreiselDatabase(1,size_kreiselDatabase(2)+1).xEndPoints(j) =  XmainRoad(j,2);
    kreiselDatabase(1,size_kreiselDatabase(2)+1).yEndPoints(j) =  YmainRoad(j,2);
    line(XmainRoad(j,:),YmainRoad(j,:));
    
    %Roads and tracks to Kreisel
    for i=1:kreiselDatabase(1,size_kreiselDatabase(2)+1).toTracks(j)
        XRoadToPoints(j,:)=XmainRoad(j,:) - exOrtho*(i*kreiselDatabase(1,size_kreiselDatabase(2)+1).trackWidth(j));
        YRoadToPoints(j,:)=YmainRoad(j,:) - eyOrtho*(i*kreiselDatabase(1,size_kreiselDatabase(2)+1).trackWidth(j));
        XToTracksPoints = XRoadToPoints(j,:) +exOrtho*(i*kreiselDatabase(1,size_kreiselDatabase(2)+1).trackWidth(j)/2);
        YToTracksPoints = YRoadToPoints(j,:) +eyOrtho*(i*kreiselDatabase(1,size_kreiselDatabase(2)+1).trackWidth(j)/2);
        kreiselDatabase(1,size_kreiselDatabase(2)+1).XRoadToPoints(j,i,:) = linspace(XRoadToPoints(j,1),XRoadToPoints(j,end),25);
        kreiselDatabase(1,size_kreiselDatabase(2)+1).YRoadToPoints(j,i,:) = linspace(YRoadToPoints(j,1),YRoadToPoints(j,end),25);
        kreiselDatabase(1,size_kreiselDatabase(2)+1).XToTracksPoints(j,i,:) = linspace(XToTracksPoints(1),XToTracksPoints(2),25);
        kreiselDatabase(1,size_kreiselDatabase(2)+1).YToTracksPoints(j,i,:) = linspace(YToTracksPoints(1),YToTracksPoints(2),25);
        plot(XToTracksPoints,YToTracksPoints,'-y');
        line(XRoadToPoints(j,:),YRoadToPoints(j,:),'color','r');
        if i==kreiselDatabase(1,size_kreiselDatabase(2)+1).toTracks(j)
            XlastmainRoadToTracks(j) = XRoadToPoints(j,1);
            YlastmainRoadToTracks(j)= YRoadToPoints(j,1);
        end
    end
    
    %Roads and tracks away from Kreisel
    for i=1:kreiselDatabase(1,size_kreiselDatabase(2)+1).awayTracks(j)
        XRoadAwayPoints(j,:)=XmainRoad(j,:) + exOrtho*(i*kreiselDatabase(1,size_kreiselDatabase(2)+1).trackWidth(j));
        YRoadAwayPoints(j,:)=YmainRoad(j,:) + eyOrtho*(i*kreiselDatabase(1,size_kreiselDatabase(2)+1).trackWidth(j));
        XAwayTracksPoints = XRoadAwayPoints(j,:) -exOrtho*(i*kreiselDatabase(1,size_kreiselDatabase(2)+1).trackWidth(j)/2);
        YAwayTracksPoints = YRoadAwayPoints(j,:) -eyOrtho*(i*kreiselDatabase(1,size_kreiselDatabase(2)+1).trackWidth(j)/2);
        kreiselDatabase(1,size_kreiselDatabase(2)+1).XRoadAwayPoints(j,i,:) = linspace(XRoadAwayPoints(j,1),XRoadAwayPoints(j,end),25);
        kreiselDatabase(1,size_kreiselDatabase(2)+1).YRoadAwayPoints(j,i,:) = linspace(YRoadAwayPoints(j,1),YRoadAwayPoints(j,end),25);
        kreiselDatabase(1,size_kreiselDatabase(2)+1).XAwayTracksPoints(j,i,:) = linspace(XAwayTracksPoints(1),XAwayTracksPoints(2),25);
        kreiselDatabase(1,size_kreiselDatabase(2)+1).YAwayTracksPoints(j,i,:) = linspace(YAwayTracksPoints(1),YAwayTracksPoints(2),25);
        plot(XAwayTracksPoints,YAwayTracksPoints,'-y');
        line(XRoadAwayPoints(j,:),YRoadAwayPoints(j,:),'color','r');
        if i==kreiselDatabase(1,size_kreiselDatabase(2)+1).awayTracks(j)
            XlastmainRoadAwayTracks(j) = XRoadAwayPoints(j,1);
            YlastmainRoadAwayTracks(j)= YRoadAwayPoints(j,1);
        end
    end
end

% Outer circle to connect all the roads
for l=1:sizekreiselAngle(2)
    
    
    if (l==sizekreiselAngle(2))
        %Find the angle of end point of road with center
        initialAngle = atan2((YlastmainRoadToTracks(l) - y),(XlastmainRoadToTracks(l)-x));
        finalAngle = atan2((YlastmainRoadAwayTracks(1) - y),(XlastmainRoadAwayTracks(1)-x));
        clearanceAngle = 0.05;%(abs(finalAngle - initialAngle))/10;
        initialAngle = initialAngle + clearanceAngle;
        finalAngle = finalAngle - clearanceAngle;
        if (initialAngle < 0) && (finalAngle >0)
            allAngles = [initialAngle:0.01:0,0:0.01:finalAngle];
        elseif (initialAngle > 0) && (finalAngle <0)
            allAngles = [initialAngle:0.01:pi, -pi:0.01:finalAngle];
        else
            allAngles = initialAngle:0.01:finalAngle;
        end
        xMiddlePoint =x+kreiselDatabase(1,size_kreiselDatabase(2)+1).outerRadius*(cos(allAngles));
        yMiddlePoint = y+kreiselDatabase(1,size_kreiselDatabase(2)+1).outerRadius*(sin(allAngles));
        xPoints = [XlastmainRoadToTracks(l) xMiddlePoint XlastmainRoadAwayTracks(1)];
        yPoints = [YlastmainRoadToTracks(l) yMiddlePoint YlastmainRoadAwayTracks(1)];
        line(xPoints,yPoints);
    else
        initialAngle = atan2((YlastmainRoadToTracks(l) - y),(XlastmainRoadToTracks(l)-x));
        finalAngle = atan2((YlastmainRoadAwayTracks(l+1) - y),(XlastmainRoadAwayTracks(l+1)-x));
        clearanceAngle = 0.1;%(abs(finalAngle - initialAngle))/10;
        initialAngle = initialAngle + clearanceAngle;
        finalAngle = finalAngle - clearanceAngle;
        if (initialAngle < 0) && (finalAngle >0)
            allAngles = [initialAngle:0.01:0,0:0.01:finalAngle];
        elseif (initialAngle > 0) && (finalAngle <0)
            allAngles = [initialAngle:0.01:pi, -pi:0.01:finalAngle];
        else
            allAngles = initialAngle:0.01:finalAngle;
        end
        xMiddlePoint =x+ kreiselDatabase(1,size_kreiselDatabase(2)+1).outerRadius*(cos (allAngles));
        yMiddlePoint = y+kreiselDatabase(1,size_kreiselDatabase(2)+1).outerRadius*(sin (allAngles));
        xPoints = [XlastmainRoadToTracks(l) xMiddlePoint XlastmainRoadAwayTracks(l+1)];
        yPoints = [YlastmainRoadToTracks(l) yMiddlePoint YlastmainRoadAwayTracks(l+1)];
        plot(xPoints, yPoints,'b-');
    end
end


function edit61_Callback(hObject, eventdata, handles)
% hObject    handle to edit61 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit61 as text
%        str2double(get(hObject,'String')) returns contents of edit61 as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function edit61_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit61 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function roadAngles_Callback(hObject, eventdata, handles)
% hObject    handle to roadAngles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roadAngles as text
%        str2double(get(hObject,'String')) returns contents of roadAngles as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function roadAngles_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roadAngles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function widthKreisel_Callback(hObject, eventdata, handles)
% hObject    handle to widthKreisel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of widthKreisel as text
%        str2double(get(hObject,'String')) returns contents of widthKreisel as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function widthKreisel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to widthKreisel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outerRadiusKreisel_Callback(hObject, eventdata, handles)
% hObject    handle to outerRadiusKreisel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outerRadiusKreisel as text
%        str2double(get(hObject,'String')) returns contents of outerRadiusKreisel as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function outerRadiusKreisel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outerRadiusKreisel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function innerRadiusKreisel_Callback(hObject, eventdata, handles)
% hObject    handle to innerRadiusKreisel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of innerRadiusKreisel as text
%        str2double(get(hObject,'String')) returns contents of innerRadiusKreisel as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function innerRadiusKreisel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to innerRadiusKreisel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in loadScenario.
function loadScenario_Callback(hObject, eventdata, handles)
% hObject    handle to loadScenario (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% loading the mat file
[fileName,pathName,~] = uigetfile('.mat','Select the Scenario','F:\THI\Thesis\Fahrdynamik Modell code\Scenario');
wholeName = strcat(pathName,fileName);
load(wholeName);
global roadDatabase ;
global vehicleDatabase;
global kreuzungDatabase;
global kreiselDatabase;
global pedestrianDatabase;
global bicycleDatabase;

size_roadDatabase = size(roadDatabase);
size_vehicleDatabase = size(vehicleDatabase);
size_kreuzungDatabase = size(kreuzungDatabase);

% plotting the road
for i=1:size_roadDatabase(2)
    hold on;
    roadDatabase(1,i).mainHandle = plot(roadDatabase(1,i).mainXRoadPoints,roadDatabase(1,i).mainYRoadPoints,'b-','LineWidth',1.5);
    
    size_outerCurves = size(roadDatabase(1,i).xOuterCurvePoints);
    for j=1:size_outerCurves(1)
        roadDatabase(1,i).outerRoadHandles(j)=plot(roadDatabase(1,i).xOuterCurvePoints(j,:),roadDatabase(1,i).yOuterCurvePoints(j,:),'g-');
    end
    roadDatabase(1,i).outerRoadHandles(size_outerCurves(1))= plot(roadDatabase(1,i).xOuterEndPoints,roadDatabase(1,i).yOuterEndPoints,'g-','LineWidth',1.5);
    
    size_outerTrackHandles = size(roadDatabase(1,i).outerTrackHandles);
    for j=1:size_outerTrackHandles(2)
        roadDatabase(1,i).outerTrackHandles(j) = plot(roadDatabase(1,i).xOuterCurveTrackPoints(j,:),roadDatabase(1,i).yOuterCurveTrackPoints(j,:),'--g');
    end
    
    size_innerCurves = size(roadDatabase(1,i).xInnerCurvePoints);
    for j=1:size_innerCurves(1)
        roadDatabase(1,i).innerRoadHandles(j)=plot(roadDatabase(1,i).xInnerCurvePoints(j,:),roadDatabase(1,i).yInnerCurvePoints(j,:),'r-');
    end
    
    roadDatabase(1,i).innerRoadHandles(j+1)=plot(roadDatabase(1,i).xInnerEndPoints,roadDatabase(1,i).yInnerEndPoints,'r-','LineWidth',1.5);
    
    size_innerTrackHandles = size(roadDatabase(1,i).innerTrackHandles);
    for j=1:size_innerTrackHandles(2)
        roadDatabase(1,i).innerTrackHandles(j) = plot(roadDatabase(1,i).xInnerCurveTrackPoints(j,:),roadDatabase(1,i).yInnerCurveTrackPoints(j,:),'--r');
    end
    rcmenu = uicontextmenu;
    r1=uimenu(rcmenu,'label','Delete','callback',@rcb1);
    set(roadDatabase(1,i).mainHandle,'uicontextmenu',rcmenu);
    
end



% ploting the kreuzung
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

% plotting vehicles
for i=1:size_vehicleDatabase(2)
    ex=cos(vehicleDatabase(1,i).angleDir(1));
    ey=sin(vehicleDatabase(1,i).angleDir(1));
    exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
    eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
    scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
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
    
    
    h3=uimenu(hcmenu,'label','Delete','Callback',@hcb3);
    
    hcb1=['set(gco,''FaceColor'',[1 0 0])'];
    hcb2=['set(gco,''FaceColor'',[0 0 1])'];
    colorBlue = uimenu(h1,'label','Blue','callback',hcb2);
    colorRed = uimenu(h1,'label','Red','callback',hcb1);
    EGOVehicle=uimenu(h2,'label','EGO','callback',@hcb4);
    otherVehicle=uimenu(h2,'label','Other','callback',@hcb5);
    
    set(vehicleDatabase(1).handle,'uicontextmenu',hcmenu);
    if strcmp(vehicleDatabase(1,i).type,'EGO');
        set(vehicleDatabase(1).handle,'Facecolor','b');
    end
    
    
end

function scenarioName_Callback(hObject, eventdata, handles)
% hObject    handle to scenarioName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of scenarioName as text
%        str2double(get(hObject,'String')) returns contents of scenarioName as a double


% --- Executes on button press in define scenario.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.vehicleParameters,'Visible','off');
set(handles.bicycleParameters,'Visible','off');
set(handles.pedestrianParameters,'Visible','off');
set(handles.roadParameters,'Visible','off');
set(handles.roadInformation,'Visible','off');
set(handles.objectPanal,'Visible','off');
set(handles.squareInformation,'Visible','Off');
set(handles.roundaboutInformation,'Visible','Off');
set(handles.uipanel13,'Visible','On');
set(handles.scenarioPanel,'Visible','On');
global vehicleDatabase;
global pedestrianDatabase;
global bicycleDatabase;
global currentHandle;

axes  = findall(gcf,'Type','axes');  %%%finding all the axes in gui\
axisElements = get(axes,'Children');
size_axisElements = size(axisElements);
size_vehicleDatabase = size(vehicleDatabase);
size_pedestrianDatabase = size(pedestrianDatabase);
size_bicycleDatabase = size(bicycleDatabase);

str='';
str1='';
currentHandle=axisElements(1);

% finding the list of the objects in the scenario
for i=1:size_axisElements
    for j=1:size_vehicleDatabase(2)
        if vehicleDatabase(1,j).handle == axisElements(i)
            size_str = size(str);
            str{size_str(2)+1}=strcat('Vehicle',num2str(size_str(2)+1));
            str1{size_str(2)+1}=vehicleDatabase(1,j).handle;
        end
    end
    for j=1:size_pedestrianDatabase(2)
        if pedestrianDatabase(1,j).handle == axisElements(i)
            size_str = size(str);
            str{size_str(2)+1}=strcat('Pedestrian',num2str(size_str(2)+1));
            str1{size_str(2)+1}=pedestrianDatabase(1,j).handle;
        end
    end
    
    for j=1:size_bicycleDatabase(2)
        if bicycleDatabase(1,j).handle == axisElements(i)
            size_str = size(str);
            str{size_str(2)+1}=strcat('Bicycle',num2str(size_str(2)+1));
            str1{size_str(2)+1}=bicycleDatabase(1,j).handle;
        end
    end
    
    set(handles.listbox4,'String',cellstr(str));
    %set(handles.listbox6,'String',str1);
end
% --- Executes when selected radiobutton5 is changed in uipanel23.
function uipanel23_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected radiobutton5 in uipanel23
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected radiobutton5 or empty if none was selected
%	NewValue: handle of the currently selected radiobutton5
% handles    structure with handles and user data (see GUIDATA)
if (get(handles.roadRadiobutton,'Value') == 1.0)
    set(handles.roadInformation,'Visible','On');
    set(handles.squareInformation,'Visible','Off');
    set(handles.roundaboutInformation,'Visible','Off');
elseif (get(handles.squareRadiobutton,'Value') == 1.0)
    set(handles.squareInformation,'Visible','On');
    set(handles.roundaboutInformation,'Visible','Off');
    set(handles.roadInformation,'Visible','Off');
elseif (get(handles.roundaboutRadiobutton,'Value') == 1.0)
    set(handles.roundaboutInformation,'Visible','On');
    set(handles.roadInformation,'Visible','Off');
    set(handles.squareInformation,'Visible','Off');
end


% --- Executes on selection change in listbox for object list.
function listbox4_Callback(hObject, eventdata, handles)
% hObject    handle to listbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox4
global vehicleDatabase;
global pedestrianDatabase;
global bicycleDatabase;
global currentHandle;

axes  = findall(gcf,'Type','axes');  %%%finding all the axes in gui\

if length(axes)>1
    axisElements = get(axes(2),'Children');
else
    axisElements = get(axes,'Children');
end

size_axisElements = size(axisElements);
size_vehicleDatabase = size(vehicleDatabase);
size_pedestrianDatabase = size(pedestrianDatabase);
size_bicycleDatabase = size(bicycleDatabase);

str='';
str1='';

for i=1:size_axisElements
    for j=1:size_vehicleDatabase(2)
        if vehicleDatabase(1,j).handle == axisElements(i)
            size_str = size(str);
            str{size_str(2)+1}=strcat('Vehicle',num2str(size_str(2)+1));
            str1{size_str(2)+1}=vehicleDatabase(1,j).handle;
        end
    end
    for j=1:size_pedestrianDatabase(2)
        if pedestrianDatabase(1,j).handle == axisElements(i)
            size_str = size(str);
            str{size_str(2)+1}=strcat('Pedestrian',num2str(size_str(2)+1));
            str1{size_str(2)+1}=pedestrianDatabase(1,j).handle;
        end
    end
    
    for j=1:size_bicycleDatabase(2)
        if bicycleDatabase(1,j).handle == axisElements(i)
            size_str = size(str);
            str{size_str(2)+1}=strcat('Bicycle',num2str(size_str(2)+1));
            str1{size_str(2)+1}=bicycleDatabase(1,j).handle;
        end
    end
    set(handles.listbox4,'String',str);
    %     set(handles.listbox6,'String',str1);
end

index_selected = get(handles.listbox4,'Value');
%list = get(handles.listbox6,'String');%handle listbox
currentHandle = (str1{1,index_selected});


%highlight the selected item in listbox in yellow color
for j=1:size_vehicleDatabase(2)
    if vehicleDatabase(1,j).handle == currentHandle
        set(vehicleDatabase(1,j).handle,'Facecolor','y');
    elseif strcmp(vehicleDatabase(1,j).type,'EGO')
        set(vehicleDatabase(1,j).handle,'Facecolor','b');
    else
        set(vehicleDatabase(1,j).handle,'Facecolor','r');
    end
end

for j=1:size_pedestrianDatabase(2)
    if pedestrianDatabase(1,j).handle == currentHandle
        set(pedestrianDatabase(1,j).handle,'Facecolor','y');
    else
        set(pedestrianDatabase(1,j).handle,'Facecolor','b');
    end
end

for j=1:size_bicycleDatabase(2)
    if bicycleDatabase(1,j).handle == currentHandle
        set(bicycleDatabase(1,j).handle,'Facecolor','y');
    else
        set(bicycleDatabase(1,j).handle,'Facecolor','m');
    end
end

% --- Executes during radiobutton5 creation, after setting all properties.
function listbox4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu5.
function popupmenu5_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu5


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu6


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton30.
function pushbutton30_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in selectTracks.
function selectTracks_Callback(hObject, eventdata, handles)
% hObject    handle to selectTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vehicleDatabase;
global roadDatabase;
global kreuzungDatabase;
global bicycleDatabase;
global currentHandle;
size_vehicleDatabase = size(vehicleDatabase);
size_roadDatabase = size(roadDatabase);
size_kreuzungDatabase = size(kreuzungDatabase);
size_bicycleDatabase = size(bicycleDatabase);

index_selected = get(handles.listbox4,'Value');
list = get(handles.listbox6,'String');
%item_selected = list(index_selected);

isVehicle=0;
isBicycle=0;

%find the  vehicle number in database
for i=1:size_vehicleDatabase(2)
    if vehicleDatabase(1,i).handle==currentHandle
        noVehicle = i;
        isVehicle = 1;
        break;
    end
end

%find the  bicycle number in database
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
        set(gco,'Linewidth',1.5);
        
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
        k=waitforbuttonpress;
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
        set(gco,'Linewidth',1.5);
        
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
        k=waitforbuttonpress;
    end
    
end

% --- Executes on button press in defineManualPath.
function defineManualPath_Callback(hObject, eventdata, handles)
% hObject    handle to defineManualPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vehicleDatabase;
global roadDatabase;
global kreuzungDatabase;
global bicycleDatabase;
global pedestrianDatabase;
global currentHandle;

size_pedestrianDatabase= size(pedestrianDatabase);
size_vehicleDatabase = size(vehicleDatabase);
size_roadDatabase = size(roadDatabase);
size_kreuzungDatabase = size(kreuzungDatabase);
size_bicycleDatabase = size(bicycleDatabase);

index_selected = get(handles.listbox4,'Value');
list = get(handles.listbox6,'String');
%item_selected = list(index_selected);

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
    
end
% --- Executes on selection change in popupmenu7.
function popupmenu7_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu7 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu7


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu8.
function popupmenu8_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu8 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu8


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu9.
function popupmenu9_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu9 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu9


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu10.
function popupmenu10_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu10 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu10


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu11.
function popupmenu11_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu11 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu11


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu12.
function popupmenu12_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu12 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu12


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton31.
function pushbutton31_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton32.
function pushbutton32_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton33.
function pushbutton33_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in listbox5.
function listbox5_Callback(hObject, eventdata, handles)
% hObject    handle to listbox5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox5


% --- Executes during radiobutton5 creation, after setting all properties.
function listbox5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu13.
function popupmenu13_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu13 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu13


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu14.
function popupmenu14_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu14 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu14


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu15.
function popupmenu15_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu15 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu15


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu16.
function popupmenu16_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu16 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu16


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu17.
function popupmenu17_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu17 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu17


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu18.
function popupmenu18_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu18 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu18


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton34.
function pushbutton34_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton35.
function pushbutton35_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton36.
function pushbutton36_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu19.
function popupmenu19_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu19 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu19


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu20.
function popupmenu20_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu20 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu20


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu21.
function popupmenu21_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu21 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu21


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu22.
function popupmenu22_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu22 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu22


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu23.
function popupmenu23_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu23 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu23


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu23_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu24.
function popupmenu24_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu24 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu24


% --- Executes during radiobutton5 creation, after setting all properties.
function popupmenu24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton37.
function pushbutton37_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton37 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton38.
function pushbutton38_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton39.
function pushbutton39_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function trackWidthKreisel_Callback(hObject, eventdata, handles)
% hObject    handle to trackWidthKreisel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of trackWidthKreisel as text
%        str2double(get(hObject,'String')) returns contents of trackWidthKreisel as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function trackWidthKreisel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trackWidthKreisel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function kreiselToTracks_Callback(hObject, eventdata, handles)
% hObject    handle to kreiselToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of kreiselToTracks as text
%        str2double(get(hObject,'String')) returns contents of kreiselToTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function kreiselToTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kreiselToTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function kreiselAwayTracks_Callback(hObject, eventdata, handles)
% hObject    handle to kreiselAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of kreiselAwayTracks as text
%        str2double(get(hObject,'String')) returns contents of kreiselAwayTracks as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function kreiselAwayTracks_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kreiselAwayTracks (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in startSimulation.
function startSimulation_Callback(hObject, eventdata, handles)
% hObject    handle to startSimulation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% starting the simulation and generating data continuously by changing the scenario
axes  = findall(gcf,'Type','axes');  %%%finding all the axes in gui\
obj = findall(gcf,'Type','Patch');
global vehicleDatabase;
size_vehicleDatabase = size(vehicleDatabase);
global pedestrianDatabase;
size_pedestrianDatabase = size(pedestrianDatabase);
global bicycleDatabase;
size_bicycleDatabase = size(bicycleDatabase);
global roadDatabase;
size_roadDatabase = size(roadDatabase);
global kreuzungDatabase;
size_kreuzungDatabase = size(kreuzungDatabase);
global objectDatabase;
size_objectDatabase=size(objectDatabase);

global vehicleDatabase;
size_vehicleDatabase=size(vehicleDatabase);
global pedestrianDatabase;
currentStep=1;

% storing initial parameters
[~,idx]=min(sqrt((vehicleDatabase(1,1).xTravelPoints-vehicleDatabase(1,1).xCG(1)).^2+(vehicleDatabase(1,1).yTravelPoints-vehicleDatabase(1,1).yCG(1)).^2));
originalxCG=vehicleDatabase(1,1).xCG;
originalyCG=vehicleDatabase(1,1).yCG;
originalSpeed=vehicleDatabase(1,1).speed(1);
originalAccn=vehicleDatabase(1,1).longitudinalAccn(1);
count=1;

for i=1:3
    [~,idx]=min(sqrt((vehicleDatabase(1,1).xTravelPoints-vehicleDatabase(1,1).xCG(1)).^2+(vehicleDatabase(1,1).yTravelPoints-vehicleDatabase(1,1).yCG(1)).^2));
    vehicleDatabase(1,1).xCG=vehicleDatabase(1,1).xTravelPoints(idx+5);
    vehicleDatabase(1,1).yCG=vehicleDatabase(1,1).yTravelPoints(idx+5);
    % changing the position
    vehicleDatabase(1,1).angleDir(1)=atan2(vehicleDatabase(1,1).yTravelPoints(idx+7)-vehicleDatabase(1,1).yTravelPoints(idx+5),...
        (vehicleDatabase(1,1).xTravelPoints(idx+7)-vehicleDatabase(1,1).xTravelPoints(idx+5)));
    vehicleDatabase(1,1).updateVehiclePosition(1);
    for j=1:3
        vehicleDatabase(1,1).speed(1)=vehicleDatabase(1,1).speed(1)+1; % changing the velocity
        for k=1:1
            ax=0;%-4+k*2;
            vehicleDatabase(1,1).longitudinalAccn=ax;
            tempVehicleDatabase=vehicleDatabase;
            tempVehicleDatabase(:,1) = [];
            tempPedestrianDatabase=pedestrianDatabase;
            [count]=vehicle_para_update(tempVehicleDatabase,tempPedestrianDatabase,count);
            %%% simulate
            %flag=inters2(vehicleDatabase,pedestrianDatabase);
            if count>0
                simulateScenario(count); % recursive fuction to permute scenarios
            end
        end
    end
    vehicleDatabase(1,1).speed(1)=originalSpeed;
end

% changing the parameters of the objects back to original
vehicleDatabase(1,1).xCG=originalxCG;
vehicleDatabase(1,1).yCG=originalyCG;

vehicleDatabase(1,1).longitudinalAccn(1)=originalAccn;
Phi=atan(vehicleDatabase(1,1).yTravelPoints(idx)-vehicleDatabase(1,1).yTravelPoints(idx-2))/((vehicleDatabase(1,1).xTravelPoints(idx)-vehicleDatabase(1,1).xTravelPoints(idx-2)));
vehicleDatabase(1,1).updateVehiclePosition(1);

%% Just to simulate the given scenario
% count=1;
% simulateScenario(count);

% --- Executes on button press in connectSquareRoads.
function connectSquareRoads_Callback(hObject, eventdata, handles)
% hObject    handle to connectSquareRoads (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global kreuzungDatabase;
size_kreuzungDatabase = size(kreuzungDatabase);

[x,y] = ginput(2);
yParabola =0;
xParabola=0;
for k=1:size_kreuzungDatabase(2)
    noRoads  = size(kreuzungDatabase(1,k).XInnerRoadToTracksPoints);
    for l=1:noRoads(1)
        for m=1:noRoads(2)
            dist1 = sqrt((x(1)-kreuzungDatabase(1,k).XInnerRoadToTracksPoints(l,m,end)).^2 + (y(1)- kreuzungDatabase(1,k).YInnerRoadToTracksPoints(l,m,end)).^2);
            if dist1 < 2
                noKreuzung = k;
                x(1) = kreuzungDatabase(1,k).XInnerRoadToTracksPoints(l,m,end);
                y(1) = kreuzungDatabase(1,k).YInnerRoadToTracksPoints(l,m,end);
                slope_endpt = kreuzungDatabase(1,k).endPointsSlope(l);
                %if angle of road is between 60-120 or 240 - 300 -> Y parabola
                %else X parabola
                if ((kreuzungDatabase(1,k).squareAngle(l)>1.047 && kreuzungDatabase(1,k).squareAngle(l)<2.094) || (kreuzungDatabase(1,k).squareAngle(l)>4.188 && kreuzungDatabase(1,k).squareAngle(l)<5.235))
                    
                    yParabola = 1;
                else
                    xParabola = 1;
                end
            end
            dist2 = sqrt((x(end)-kreuzungDatabase(1,k).XInnerRoadAwayTracksPoints(l,m,1)).^2 + (y(end)- kreuzungDatabase(1,k).YInnerRoadAwayTracksPoints(l,m,1)).^2);
            if dist2 < 2
                x(end) = kreuzungDatabase(1,k).XInnerRoadAwayTracksPoints(l,m,1);
                y(end) = kreuzungDatabase(1,k).YInnerRoadAwayTracksPoints(l,m,1);
                slope_endpt2 = kreuzungDatabase(1,k).endPointsSlope(l);
            end
        end
    end
end

size_roadHandles = size(kreuzungDatabase(1,noKreuzung).roadJoiningHandles);

if xParabola == 1
    
    [new_xRoadPoints new_yRoadPoints slope_endpt] = XparabolaSpecPoints(x(1),y(1),x(2),y(2),slope_endpt,100);
    kreuzungDatabase(1,noKreuzung).xRoadJoiningPoints(size_roadHandles(2)+1,:) = new_xRoadPoints(1,:);
    kreuzungDatabase(1,noKreuzung).yRoadJoiningPoints(size_roadHandles(2)+1,:) = new_yRoadPoints(1,:);
    kreuzungDatabase(1,noKreuzung).roadJoiningHandles(1,size_roadHandles(2)+1) = plot(new_xRoadPoints,new_yRoadPoints,'b-');
else
    [new_xRoadPoints new_yRoadPoints slope_endpt] = YparabolaSpecPoints(x(1),y(1),x(2),y(2),slope_endpt,100);
    kreuzungDatabase(1,noKreuzung).xRoadJoiningPoints(size_roadHandles(2)+1,:)= new_xRoadPoints(1,:);
    kreuzungDatabase(1,noKreuzung).yRoadJoiningPoints(size_roadHandles(2)+1,:) = new_yRoadPoints(1,:);
    kreuzungDatabase(1,noKreuzung).roadJoiningHandles(1,size_roadHandles(2)+1) = plot(new_xRoadPoints,new_yRoadPoints,'m-');
end



function signalTime_Callback(hObject, eventdata, handles)
% hObject    handle to signalTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of signalTime as text
%        str2double(get(hObject,'String')) returns contents of signalTime as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function signalTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to signalTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox6.
function listbox6_Callback(hObject, eventdata, handles)
% hObject    handle to listbox6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox6


% --- Executes during radiobutton5 creation, after setting all properties.
function listbox6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function currentTime_Callback(hObject, eventdata, handles)
% hObject    handle to currentTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of currentTime as text
%        str2double(get(hObject,'String')) returns contents of currentTime as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function currentTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to currentTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during radiobutton5 creation, after setting all properties.
function connectSquareRoads_CreateFcn(hObject, eventdata, handles)
% hObject    handle to connectSquareRoads (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function speedBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to speedBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of speedBicycle as text
%        str2double(get(hObject,'String')) returns contents of speedBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function speedBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to speedBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function longAccnBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to longAccnBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of longAccnBicycle as text
%        str2double(get(hObject,'String')) returns contents of longAccnBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function longAccnBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to longAccnBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lateralAccnBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to lateralAccnBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lateralAccnBicycle as text
%        str2double(get(hObject,'String')) returns contents of lateralAccnBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lateralAccnBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lateralAccnBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function massBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to massBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of massBicycle as text
%        str2double(get(hObject,'String')) returns contents of massBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function massBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to massBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function heightCGBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to heightCGBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of heightCGBicycle as text
%        str2double(get(hObject,'String')) returns contents of heightCGBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function heightCGBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to heightCGBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function widthBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to widthBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of widthBicycle as text
%        str2double(get(hObject,'String')) returns contents of widthBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function widthBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to widthBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function steeringRatioBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to steeringRatioBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of steeringRatioBicycle as text
%        str2double(get(hObject,'String')) returns contents of steeringRatioBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function steeringRatioBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to steeringRatioBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function angleDirBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to angleDirBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angleDirBicycle as text
%        str2double(get(hObject,'String')) returns contents of angleDirBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function angleDirBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angleDirBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function handleBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to handleBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of handleBicycle as text
%        str2double(get(hObject,'String')) returns contents of handleBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function handleBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to handleBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lengthBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to lengthBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lengthBicycle as text
%        str2double(get(hObject,'String')) returns contents of lengthBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lengthBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lengthBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xLocBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to xLocBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xLocBicycle as text
%        str2double(get(hObject,'String')) returns contents of xLocBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function xLocBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xLocBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function YLocBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to YLocBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of YLocBicycle as text
%        str2double(get(hObject,'String')) returns contents of YLocBicycle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function YLocBicycle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YLocBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in saveBicycleState.
function saveBicycleState_Callback(hObject, eventdata, handles)
% hObject    handle to saveBicycleState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global bicycleDatabase;
global currentHandle;
size_bicycleDatabase = size(bicycleDatabase);

for i=1:size_bicycleDatabase(2)
    if  bicycleDatabase(1,i).handle==currentHandle
        bicycleDatabase(1,i).speed = str2double(get(handles.speedBicycle,'String'));
        bicycleDatabase(1,i).lateralAccn = str2double(get(handles.lateralAccnBicycle,'String'));
        bicycleDatabase(1,i).longitudinalAccn = str2double(get(handles.longAccnBicycle,'String'));
        bicycleDatabase(1,i).vehicleMass = str2double(get(handles.massBicycle,'String'));
        bicycleDatabase(1,i).heightCG = str2double(get(handles.heightCGBicycle,'String'));
        bicycleDatabase(1,i).lengthVehicle = str2double(get(handles.lengthBicycle,'String'));
        bicycleDatabase(1,i).widthVehicle = str2double(get(handles.widthBicycle,'String'));
        bicycleDatabase(1,i).steeringRatio = str2double(get(handles.steeringRatioBicycle,'String'));
        bicycleDatabase(1,i).angleDir = str2double(get(handles.angleDirBicycle,'String'))*pi/180;
        bicycleDatabase(1,i).xCG = str2double(get(handles.xLocBicycle,'String'));
        bicycleDatabase(1,i).yCG = str2double(get(handles.YLocBicycle,'String'));
        
        %% Rotate the bicycle with calculating the unit vector
        
        ex=cos((str2double(get(handles.angleDirBicycle,'String')))*pi/180);
        ey=sin((str2double(get(handles.angleDirBicycle,'String')))*pi/180);
        exOrtho=ex*cos(pi/2)+ey*sin(pi/2);
        eyOrtho=-ex*sin(pi/2)+ey*cos(pi/2);
        scalingUnit = str2double(get(handles.scalingUnit,'String'));
        xCoordinate = zeros(4,1);
        yCoordinate = zeros(4,1);
        
        
        xCoordinate(1)= bicycleDatabase(1,i).xCG-ex*(((bicycleDatabase(1,i).lengthVehicle)/2000)/scalingUnit)-((bicycleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
        xCoordinate(2)= bicycleDatabase(1,i).xCG-ex*(((bicycleDatabase(1,i).lengthVehicle)/2000 )/scalingUnit)+((bicycleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
        xCoordinate(3)= bicycleDatabase(1,i).xCG+ex*(((bicycleDatabase(1,i).lengthVehicle)/2000)/scalingUnit)+((((bicycleDatabase(1,i).widthVehicle)/2000)/scalingUnit)*exOrtho);
        xCoordinate(4)= bicycleDatabase(1,i).xCG+ex*(((bicycleDatabase(1,i).lengthVehicle)/2000)/scalingUnit)-((bicycleDatabase(1,i).widthVehicle)/2000*exOrtho/scalingUnit);
        yCoordinate(1)= bicycleDatabase(1,i).yCG-ey*(((bicycleDatabase(1,i).widthVehicle)/2000)/scalingUnit)-((bicycleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
        yCoordinate(2)= bicycleDatabase(1,i).yCG-ey*(((bicycleDatabase(1,i).widthVehicle)/2000)/scalingUnit)+((bicycleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
        yCoordinate(3)= bicycleDatabase(1,i).yCG+ey*(((bicycleDatabase(1,i).widthVehicle)/2000)/scalingUnit)+((bicycleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
        yCoordinate(4)= bicycleDatabase(1,i).yCG+ey*(((bicycleDatabase(1,i).widthVehicle)/2000)/scalingUnit)-((bicycleDatabase(1,i).widthVehicle)/2000*eyOrtho/scalingUnit);
        
        bicycleDatabase(1,i).xCoordinates(1,:) = [xCoordinate(1) xCoordinate(2) xCoordinate(3) xCoordinate(4) xCoordinate(1)];
        bicycleDatabase(1,i).yCoordinates(1,:)= [yCoordinate(1) yCoordinate(2) yCoordinate(3) yCoordinate(4) yCoordinate(1)];
        set(bicycleDatabase(1,i).handle,'xData',xCoordinate,'yData',yCoordinate);
    end
    
    
end
% --- Executes on button press in pushbutton47.
function pushbutton47_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function xLocationVehicle_Callback(hObject, eventdata, handles)
% hObject    handle to xLocationVehicle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xLocationVehicle as text
%        str2double(get(hObject,'String')) returns contents of xLocationVehicle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function xLocationVehicle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xLocationVehicle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yLocationVehicle_Callback(hObject, eventdata, handles)
% hObject    handle to yLocationVehicle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yLocationVehicle as text
%        str2double(get(hObject,'String')) returns contents of yLocationVehicle as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function yLocationVehicle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yLocationVehicle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function speedPedestrian_Callback(hObject, eventdata, handles)
% hObject    handle to speedPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of speedPedestrian as text
%        str2double(get(hObject,'String')) returns contents of speedPedestrian as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function speedPedestrian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to speedPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function accnPedestrian_Callback(hObject, eventdata, handles)
% hObject    handle to accnPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of accnPedestrian as text
%        str2double(get(hObject,'String')) returns contents of accnPedestrian as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function accnPedestrian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to accnPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xLocationPedestrian_Callback(hObject, eventdata, handles)
% hObject    handle to xLocationPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xLocationPedestrian as text
%        str2double(get(hObject,'String')) returns contents of xLocationPedestrian as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function xLocationPedestrian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xLocationPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yLocationPedestrian_Callback(hObject, eventdata, handles)
% hObject    handle to yLocationPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yLocationPedestrian as text
%        str2double(get(hObject,'String')) returns contents of yLocationPedestrian as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function yLocationPedestrian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yLocationPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function maxSpeedPedestrian_Callback(hObject, eventdata, handles)
% hObject    handle to maxSpeedPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maxSpeedPedestrian as text
%        str2double(get(hObject,'String')) returns contents of maxSpeedPedestrian as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function maxSpeedPedestrian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maxSpeedPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function minSpeedPedestrian_Callback(hObject, eventdata, handles)
% hObject    handle to minSpeedPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of minSpeedPedestrian as text
%        str2double(get(hObject,'String')) returns contents of minSpeedPedestrian as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function minSpeedPedestrian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to minSpeedPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in savePedestrianState.
function savePedestrianState_Callback(hObject, eventdata, handles)
% hObject    handle to savePedestrianState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global pedestrianDatabase;
size_pedestrianDatabase = size(pedestrianDatabase);

for i=1:size_pedestrianDatabase(2)
    if abs(str2double(get(handles.handlePedestrian,'String')) - pedestrianDatabase(1,i).handle) < 0.01
        pedestrianDatabase(1,i).speed = str2double(get(handles.speedPedestrian,'String'));
        pedestrianDatabase(1,i).accn = str2double(get(handles.accnPedestrian,'String'));
        pedestrianDatabase(1,i).angleDir = str2double(get(handles.dirPedestrian,'String'));
        pedestrianDatabase(1,i).maxSpeed = str2double(get(handles.maxSpeedPedestrian,'String'));
        pedestrianDatabase(1,i).minSpeed = str2double(get(handles.minSpeedPedestrian,'String'));
        pedestrianDatabase(1,i).xPos = str2double(get(handles.xLocationPedestrian,'String'));
        pedestrianDatabase(1,i).yPos = str2double(get(handles.yLocationPedestrian,'String'));
        
        % Update the position of Pedestrian
        set(pedestrianDatabase(1,i).handle,'Position',[ pedestrianDatabase(1,i).xPos,pedestrianDatabase(1,i).yPos,0.8,0.8]);
        
    end
    
end


function dirPedestrian_Callback(hObject, eventdata, handles)
% hObject    handle to dirPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dirPedestrian as text
%        str2double(get(hObject,'String')) returns contents of dirPedestrian as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function dirPedestrian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dirPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function handlePedestrian_Callback(hObject, eventdata, handles)
% hObject    handle to handlePedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of handlePedestrian as text
%        str2double(get(hObject,'String')) returns contents of handlePedestrian as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function handlePedestrian_CreateFcn(hObject, eventdata, handles)
% hObject    handle to handlePedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in newBicycle.
function newBicycle_Callback(hObject, eventdata, handles)
% hObject    handle to newBicycle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(gcf,'Pointer','crosshair','doublebuffer','on');
k=0;
while(k==0)
    %Get the initial point
    [xs,ys,zs] = ginput(1);
    hold on
    if (get(handles.radiobutton4,'Value')~=1.0 || isempty(xs))
        k=1;
        set(gcf,'Pointer','arrow');
        break;
    end
    
    if (k==1)
        break;
    end
    global bicycleDatabase;
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
    end
end

% --- Executes on button press in newPedestrian.
function newPedestrian_Callback(hObject, eventdata, handles)
% hObject    handle to newPedestrian (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(gcf,'Pointer','crosshair','doublebuffer','on');
k=0;
while k==0
    %Get the initial point
    [xs,ys,zs] = ginput(1);
    hold on;
    % if enter is pressed xs is empty and it is an indication to stop
    % creating new vehicle
    if (get(handles.radiobutton2,'Value')~=1.0 || isempty(xs))
        k=1;
        set(gcf,'Pointer','arrow');
        break;
    end
    
    if (k==1)
        break;
    end
    global pedestrianDatabase;
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
    % k=waitforbuttonpress;
    %
    % while k==0
    % handle = gco;
    % type=get(handle,'Type')
    % k=waitforbuttonpress;
    % end
    
    %Contextmenu for pedestrin
    hcmenu=uicontextmenu;
    hcb1=['set(gco,''FaceColor'',[1 0 0])'];
    hcb2=['set(gco,''FaceColor'',[0 0 1])'];
    item1=uimenu(hcmenu,'Label','Red','Callback',hcb1);
    item2=uimenu(hcmenu,'Label','Blue','Callback',hcb2);
    
    
    hRectangles = findall(gca,'Type','Rectangle');
    
    for Rectangle=1:length(hRectangles)
        set(hRectangles(Rectangle),'uicontextmenu',hcmenu);
    end
    
end

% --- Executes on button press in newVehicle.
function newVehicle_Callback(hObject, eventdata, handles)
% hObject    handle to newVehicle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vehicleDatabase;
%Contextmenu for vehicle
hcmenu=uicontextmenu;
h2=uimenu(hcmenu,'label','Vehicle Type');
h1=uimenu(hcmenu,'label','Color');


h3=uimenu(hcmenu,'label','Delete','Callback',@hcb3);

hcb1=['set(gco,''FaceColor'',[1 0 0])'];
hcb2=['set(gco,''FaceColor'',[0 0 1])'];
colorBlue = uimenu(h1,'label','Blue','callback',hcb2);
colorRed = uimenu(h1,'label','Red','callback',hcb1);
EGOVehicle=uimenu(h2,'label','EGO','callback',@hcb4);
otherVehicle=uimenu(h2,'label','Other','callback',@hcb5);
set(gcf,'Pointer','crosshair','doublebuffer','on');

k=0;
while k==0
    %Get the initial point
    
    [xs,ys,zs] = ginput(1);
    hold on;
    % if enter is pressed xs is empty and it is an indication to stop
    % creating new vehicle
    if (get(handles.radiobutton1,'Value')~=1.0 || isempty(xs))
        k=1;
        set(gcf,'Pointer','arrow');
        break;
    end
    
    if (k==1)
        break;
    end
    
    scalingUnit =1;% str2double(get(handles.scalingUnit,'String'));
    size_veh = size(vehicleDatabase);
    
    
    %% assigning vehicle radiobutton5 and calculating the dimention of vehicle and plotting into graph
    if isempty(vehicleDatabase)
        vehicleDatabase=Vehicle;
        xCoordinate(1)= xs-(((vehicleDatabase(1,(size_veh(2)+1)).lr + vehicleDatabase(1,(size_veh(2)+1)).lrToRear)/1000)/scalingUnit);
        xCoordinate(2)= xs-(((vehicleDatabase(1,(size_veh(2)+1)).lr + vehicleDatabase(1,(size_veh(2)+1)).lrToRear)/1000)/scalingUnit);
        xCoordinate(3)= xs+(((vehicleDatabase(1,(size_veh(2)+1)).lf + vehicleDatabase(1,(size_veh(2)+1)).lfToFront)/1000)/scalingUnit);
        xCoordinate(4)= xs+(((vehicleDatabase(1,(size_veh(2)+1)).lf + vehicleDatabase(1,(size_veh(2)+1)).lfToFront)/1000)/scalingUnit);
        yCoordinate(1)= ys-(((vehicleDatabase(1,(size_veh(2)+1)).widthVehicle)/2000)/scalingUnit);
        yCoordinate(2)= ys+(((vehicleDatabase(1,(size_veh(2)+1)).widthVehicle)/2000)/scalingUnit);
        yCoordinate(3)=ys+(((vehicleDatabase(1,(size_veh(2)+1)).widthVehicle)/2000)/scalingUnit);
        yCoordinate(4)=ys-(((vehicleDatabase(1,(size_veh(2)+1)).widthVehicle)/2000)/scalingUnit);
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
        %  [vehicleDatabase(1).xCenter,vehicleDatabase(1).yCenter] = polyxpoly(xDiagonal1,yDiagonal1,xDiagonal2,yDiagonal2);
        %  vehicleDatabase(1).radiusCollisionControl = sqrt((xDiagonal1-vehicleDatabase(1).xCenter).^2 + (yDiagonal1-vehicleDatabase(1).yCenter).^2);
        %             plot(vehicleDatabase(1).xCenter,vehicleDatabase(1).yCenter,'bo');
        
        set(vehicleDatabase(1).handle,'uicontextmenu',hcmenu);
    else
        vehicleDatabase(1,(size_veh(2)+1)) = Vehicle;
        xCoordinate(1)= xs-(((vehicleDatabase(1,(size_veh(2)+1)).lr + vehicleDatabase(1,(size_veh(2)+1)).lrToRear)/1000)/scalingUnit);
        xCoordinate(2)= xs-(((vehicleDatabase(1,(size_veh(2)+1)).lr + vehicleDatabase(1,(size_veh(2)+1)).lrToRear)/1000)/scalingUnit);
        xCoordinate(3)= xs+(((vehicleDatabase(1,(size_veh(2)+1)).lf + vehicleDatabase(1,(size_veh(2)+1)).lfToFront)/1000)/scalingUnit);
        xCoordinate(4)= xs+(((vehicleDatabase(1,(size_veh(2)+1)).lf + vehicleDatabase(1,(size_veh(2)+1)).lfToFront)/1000)/scalingUnit);
        yCoordinate(1)= ys-(((vehicleDatabase(1,(size_veh(2)+1)).widthVehicle)/2000)/scalingUnit);
        yCoordinate(2)= ys+(((vehicleDatabase(1,(size_veh(2)+1)).widthVehicle)/2000)/scalingUnit);
        yCoordinate(3)=ys+(((vehicleDatabase(1,(size_veh(2)+1)).widthVehicle)/2000)/scalingUnit);
        yCoordinate(4)=ys-(((vehicleDatabase(1,(size_veh(2)+1)).widthVehicle)/2000)/scalingUnit);
        vehicleObj = patch(xCoordinate,yCoordinate,'r');
        vehicleDatabase(1,(size_veh(2)+1)).handle=vehicleObj;
        vehicleDatabase(1,(size_veh(2)+1)).xCG=xs;
        vehicleDatabase(1,(size_veh(2)+1)).yCG=ys;
        vehicleDatabase(1,(size_veh(2)+1)).xCoordinates(1,:)= [xCoordinate(1) xCoordinate(2) xCoordinate(3) xCoordinate(4) xCoordinate(1)];
        vehicleDatabase(1,(size_veh(2)+1)).yCoordinates(1,:) = [yCoordinate(1) yCoordinate(2) yCoordinate(3) yCoordinate(4) yCoordinate(1)];
        xDiagonal1 = [xCoordinate(1) xCoordinate(3)];
        xDiagonal2 = [xCoordinate(2) xCoordinate(4)];
        yDiagonal1 = [yCoordinate(1) yCoordinate(3)];
        yDiagonal2 = [yCoordinate(2) yCoordinate(4)];
        % [ vehicleDatabase(1,(size_veh(2)+1)).xCenter, vehicleDatabase(1,(size_veh(2)+1)).yCenter] = polyxpoly(xDiagonal1,yDiagonal1,xDiagonal2,yDiagonal2);
        set(vehicleDatabase(1,(size_veh(2)+1)).handle,'uicontextmenu',hcmenu);
        % vehicleDatabase(1,(size_veh(2)+1)).radiusCollisionControl = sqrt((xDiagonal1-vehicleDatabase(1,(size_veh(2)+1)).xCenter).^2 + (yDiagonal1-vehicleDatabase(1,(size_veh(2)+1)).yCenter).^2);
        %                         plot(vehicleDatabase(1,(size_veh(2)+1)).xCenter,vehicleDatabase(1,(size_veh(2)+1)).yCenter,'bo');
        
    end
    
    
    
    
end


% --- Executes on button press in newObject.
function newObject_Callback(hObject, eventdata, handles)
% hObject    handle to newObject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global objectDatabase;
%Contextmenu for vehicle
hcmenu=uicontextmenu;

h3=uimenu(hcmenu,'label','Delete','Callback',@ocb3);

set(gcf,'Pointer','crosshair','doublebuffer','on');
hold on;

%Get the initial point

[xs,ys,zs] = ginput(1);

size_objectDatabase = size(objectDatabase);

scalingUnit=1;

if isempty(objectDatabase)
    objectDatabase=StationaryObject;
else
    objectDatabase(1,size_objectDatabase(2)+1)=StationaryObject;
end
objectDatabase(1,size_objectDatabase(2)+1).length = str2double(get(handles.lengthObject,'String'));
objectDatabase(1,size_objectDatabase(2)+1).breadth = str2double(get(handles.breadthObject,'String'));
objectDatabase(1,size_objectDatabase(2)+1).angle = str2double(get(handles.angleObject,'String'));
objectDatabase(1,size_objectDatabase(2)+1).xCenter = xs;
objectDatabase(1,size_objectDatabase(2)+1).yCenter = ys;

ex=cos((str2double(get(handles.angleObject,'String')))*pi/180);
ey=sin((str2double(get(handles.angleObject,'String')))*pi/180);
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


function lengthObject_Callback(hObject, eventdata, handles)
% hObject    handle to lengthObject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lengthObject as text
%        str2double(get(hObject,'String')) returns contents of lengthObject as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function lengthObject_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lengthObject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function breadthObject_Callback(hObject, eventdata, handles)
% hObject    handle to breadthObject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of breadthObject as text
%        str2double(get(hObject,'String')) returns contents of breadthObject as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function breadthObject_CreateFcn(hObject, eventdata, handles)
% hObject    handle to breadthObject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function angleObject_Callback(hObject, eventdata, handles)
% hObject    handle to angleObject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angleObject as text
%        str2double(get(hObject,'String')) returns contents of angleObject as a double


% --- Executes during radiobutton5 creation, after setting all properties.
function angleObject_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angleObject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in signalController.
function signalController_Callback(hObject, eventdata, handles)
% hObject    handle to signalController (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of signalController


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2


% --- Executes on button press in longitudinalController.
function longitudinalController_Callback(hObject, eventdata, handles)
% hObject    handle to longitudinalController (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of longitudinalController


% --- Executes on button press in panButton.
function panButton_Callback(hObject, eventdata, handles)
% hObject    handle to panButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in togglebutton2.
function togglebutton2_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton2
button_state = get(hObject,'Value');
if button_state == get(hObject,'Max')
    pan on;
elseif button_state == get(hObject,'Min')
    pan off;
end


% --- Executes on button press in togglebutton3.
function togglebutton3_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton3
button_state = get(hObject,'Value');
if button_state == get(hObject,'Max')
    zoom on;
elseif button_state == get(hObject,'Min')
    zoom off;
end



function yDimensionMax_Callback(hObject, eventdata, handles)
% hObject    handle to yDimensionMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yDimensionMax as text
%        str2double(get(hObject,'String')) returns contents of yDimensionMax as a double


% --- Executes during object creation, after setting all properties.
function yDimensionMax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yDimensionMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yDimensionMin_Callback(hObject, eventdata, handles)
% hObject    handle to yDimensionMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yDimensionMin as text
%        str2double(get(hObject,'String')) returns contents of yDimensionMin as a double


% --- Executes during object creation, after setting all properties.
function yDimensionMin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yDimensionMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in clearScenario.
function clearScenario_Callback(hObject, eventdata, handles)
% hObject    handle to clearScenario (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of clearScenario
global vehicleDatabase;
global bicycleDatabase;
global pedestrianDatabase;
global objectDatabase;
global roadDatabase;
global kreuzungDatabase;
global kreiselDatabase;


size_veh_data = size(vehicleDatabase);
for i=1:size_veh_data(2)
    gvh=vehicleDatabase(1,i).handle ;
    delete (gvh);
end
vehicleDatabase = [];

size_veh_data = size(bicycleDatabase);
for i=1:size_veh_data(2)
    gvh=bicycleDatabase(1,i).handle ;
    delete (gvh);
end
bicycleDatabase = [];

size_veh_data = size(pedestrianDatabase);
for i=1:size_veh_data(2)
    gvh=pedestrianDatabase(1,i).handle ;
    delete (gvh);
end
pedestrianDatabase = [];

size_veh_data = size(objectDatabase);
for i=1:size_veh_data(2)
    gvh=objectDatabase(1,i).handle ;
    delete (gvh);
end
objectDatabase = [];


size_roadDatabase = size(roadDatabase);
for i=1:size_roadDatabase(2)
    
    delete(roadDatabase(1,i).mainHandle);
    delete(roadDatabase(1,i).innerRoadHandles);
    delete(roadDatabase(1,i).outerRoadHandles);
    delete(roadDatabase(1,i).innerTrackHandles);
    delete(roadDatabase(1,i).outerTrackHandles);
    
end

size_kreuzungDatabase=size(kreuzungDatabase);
for i=1:size_kreuzungDatabase(2)
    delete(kreuzungDatabase(1,i).mainRoadHandles);
    delete(kreuzungDatabase(1,i).toTracksHandles);
    delete(kreuzungDatabase(1,i).awayTrackHandles);
    delete(kreuzungDatabase(1,i).innerRoadToTracksHandles);
    delete(kreuzungDatabase(1,i).innerRoadAwayTracksHandles);
    delete(kreuzungDatabase(1,i).roadJoiningHandles);
    delete(kreuzungDatabase(1,i).kreuzungCornerHandles);
    delete(kreuzungDatabase(1,i).trafficSignal);
end

function timeStep_Callback(hObject, eventdata, handles)
% hObject    handle to timeStep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timeStep as text
%        str2double(get(hObject,'String')) returns contents of timeStep as a double


% --- Executes during object creation, after setting all properties.
function timeStep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timeStep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function simulationTime_Callback(hObject, eventdata, handles)
% hObject    handle to simulationTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of simulationTime as text
%        str2double(get(hObject,'String')) returns contents of simulationTime as a double


% --- Executes during object creation, after setting all properties.
function simulationTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to simulationTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
