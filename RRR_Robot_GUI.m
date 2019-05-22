
function varargout = RRR_Robot_GUI(varargin)
% RRR_ROBOT_GUI MATLAB code for RRR_Robot_GUI.fig
%      RRR_ROBOT_GUI, by itself, creates a new RRR_ROBOT_GUI or raises the existing
%      singleton*.
%
%      H = RRR_ROBOT_GUI returns the handle to a new RRR_ROBOT_GUI or the handle to
%      the existing singleton*.
%
%      RRR_ROBOT_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RRR_ROBOT_GUI.M with the given input arguments.
%
%      RRR_ROBOT_GUI('Property','Value',...) creates a new RRR_ROBOT_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RRR_Robot_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RRR_Robot_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RRR_Robot_GUI

% Last Modified by GUIDE v2.5 05-May-2019 13:34:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RRR_Robot_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @RRR_Robot_GUI_OutputFcn, ...
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


% --- Executes just before RRR_Robot_GUI is made visible.
function RRR_Robot_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RRR_Robot_GUI (see VARARGIN)

% Choose default command line output for RRR_Robot_GUI
handles.output = hObject;

% Create the global variables
handles.q_old = [0 0 0];

% Update handles structure
guidata(hObject, handles);



% UIWAIT makes RRR_Robot_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RRR_Robot_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_1 as text
%        str2double(get(hObject,'String')) returns contents of Theta_1 as a double


% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_2 as text
%        str2double(get(hObject,'String')) returns contents of Theta_2 as a double


% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_3 as text
%        str2double(get(hObject,'String')) returns contents of Theta_3 as a double


% --- Executes during object creation, after setting all properties.
function Theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_forward.
function btn_forward_Callback(hObject, eventdata, handles)
% hObject    handle to btn_forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get the joint values from the GUI
Th_1 = str2double(handles.Theta_1.String)*pi/180;
Th_2 = str2double(handles.Theta_2.String)*pi/180;
Th_3 = str2double(handles.Theta_3.String)*pi/180;

% Get the links lengths from the GUI
L_1 = str2double(handles.L_1.String);
L_2 = str2double(handles.L_2.String);
L_3 = str2double(handles.L_3.String);

% Create the links for the Robot
L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);

% Create the Robot
Robot = SerialLink(L);

% Define the robot's name
Robot.name = 'RRR';

% Create a variable to store the joint values
q = [Th_1 Th_2 Th_3];

%Create the trajectorie
q_trj = jtraj(handles.q_old, q, 10);

% Plot the Trajectorie
Robot.plot(q_trj);
%Robot.plot(q_trj, 'movie', 'images');

% Store the old joint values
handles.q_old = q;

% Update handles structure
guidata(hObject, handles);

% Transformation Matrix from the Forward Kinematics
T = Robot.fkine(q);

% Extract the X, Y, Z positions from the Transformation Matrix
Pos_X = T.t(1);
Pos_Y = T.t(2);
Pos_Z = T.t(3);

% Pass the positions to the GUI
handles.Pos_X.String = num2str(floor(Pos_X));
handles.Pos_Y.String = num2str(floor(Pos_Y));
handles.Pos_Z.String = num2str(floor(Pos_Z));



function Pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_X as text
%        str2double(get(hObject,'String')) returns contents of Pos_X as a double


% --- Executes during object creation, after setting all properties.
function Pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Y as text
%        str2double(get(hObject,'String')) returns contents of Pos_Y as a double


% --- Executes during object creation, after setting all properties.
function Pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Z as text
%        str2double(get(hObject,'String')) returns contents of Pos_Z as a double


% --- Executes during object creation, after setting all properties.
function Pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_inverse.
function btn_inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get the desired positions from the GUI
PX = str2double(handles.Pos_X.String);
PY = str2double(handles.Pos_Y.String);
PZ = str2double(handles.Pos_Z.String);

% Get the links lengths from the GUI
L_1 = str2double(handles.L_1.String);
L_2 = str2double(handles.L_2.String);
L_3 = str2double(handles.L_3.String);

% Define the Links for the Robot
L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);

% Create the Robot
Robot = SerialLink(L);

% Define the robot's name
Robot.name = 'RRR';

% Create the Transform matrix for the desired Positions
T = [1 0 0 PX;
     0 1 0 PY;
     0 0 1 PZ;
     0 0 0 1];
 
%J = Robot.ikine(T, [0 0 0], [1 1 1 0 0 0])*180/pi;
J = Robot.ikine(T, 'mask', [1 1 1 0 0 0])*180/pi;

if size(J,1)==0 && size(J,2)==0
    disp('The robot can not reach the Position')
    handles.Theta_1.String = 'Error';
    handles.Theta_2.String = 'Error';
    handles.Theta_3.String = 'Error';
else
    handles.Theta_1.String = num2str(floor(J(1)));
    handles.Theta_2.String = num2str(floor(J(2)));
    handles.Theta_3.String = num2str(floor(J(3)));
    
    Robot.plot(J*pi/180);
end



function L_1_Callback(hObject, eventdata, handles)
% hObject    handle to L_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_1 as text
%        str2double(get(hObject,'String')) returns contents of L_1 as a double


% --- Executes during object creation, after setting all properties.
function L_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function L_2_Callback(hObject, eventdata, handles)
% hObject    handle to L_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_2 as text
%        str2double(get(hObject,'String')) returns contents of L_2 as a double


% --- Executes during object creation, after setting all properties.
function L_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function L_3_Callback(hObject, eventdata, handles)
% hObject    handle to L_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L_3 as text
%        str2double(get(hObject,'String')) returns contents of L_3 as a double


% --- Executes during object creation, after setting all properties.
function L_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
