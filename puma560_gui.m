function varargout = puma560_gui(varargin)
% PUMA560_GUI MATLAB code for puma560_gui.fig
%      PUMA560_GUI, by itself, creates a new PUMA560_GUI or raises the existing
%      singleton*.
%
%      H = PUMA560_GUI returns the handle to a new PUMA560_GUI or the handle to
%      the existing singleton*.
%
%      PUMA560_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PUMA560_GUI.M with the given input arguments.
%
%      PUMA560_GUI('Property','Value',...) creates a new PUMA560_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before puma560_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to puma560_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help puma560_gui

% Last Modified by GUIDE v2.5 18-May-2019 01:02:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @puma560_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @puma560_gui_OutputFcn, ...
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


% --- Executes just before puma560_gui is made visible.
function puma560_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to puma560_gui (see VARARGIN)

% Choose default command line output for puma560_gui
handles.output = hObject;

% Create the global variables
handles.q_old = [0 0 0 0 0 0];

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes puma560_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = puma560_gui_OutputFcn(hObject, eventdata, handles) 
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



function Theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_4 as text
%        str2double(get(hObject,'String')) returns contents of Theta_4 as a double


% --- Executes during object creation, after setting all properties.
function Theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_5_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_5 as text
%        str2double(get(hObject,'String')) returns contents of Theta_5 as a double


% --- Executes during object creation, after setting all properties.
function Theta_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_6_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_6 as text
%        str2double(get(hObject,'String')) returns contents of Theta_6 as a double


% --- Executes during object creation, after setting all properties.
function Theta_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_6 (see GCBO)
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
% Display a message
disp('Forward Kinematics for Puma 560')
mdl_puma560
% Get the joint values from the GUI
Th_1 = str2double(handles.Theta_1.String); % deg
Th_2 = str2double(handles.Theta_2.String); % deg
Th_3 = str2double(handles.Theta_3.String); % deg
Th_4 = str2double(handles.Theta_4.String); % deg
Th_5 = str2double(handles.Theta_5.String); % deg
Th_6 = str2double(handles.Theta_6.String); % deg

force_x = str2double(handles.f_x.String);
force_y = str2double(handles.f_y.String);
force_z = str2double(handles.f_z.String);
torque_x = str2double(handles.tor_x.String);
torque_y = str2double(handles.tor_y.String);
torque_z = str2double(handles.tor_z.String);

% Create the force vector
force = [force_x; force_y; force_z; torque_x; torque_y; torque_z];

% Display a message
disp(['Joint Values - q : ', mat2str([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]), ' deg'])

% Create a variable to store the joint values
q = [Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]*pi/180; % rad

%Create the trajectory
disp(['Trayectory (rad)'])
q_trj = jtraj(handles.q_old, q, 10)

% Create a variable for the Jacobian's determiants
det_trj = zeros(1, size(q_trj, 1));

% Create a variable for the Jacobian's ranks
rank_trj = zeros(1, size(q_trj, 1));

for i = 1:size(q_trj, 1)
    % Check for singular value
    % Calculate the Jacobian Matrix
    J = p560.jacobe(q_trj(i,:));
    
    % Calculate the determinant
    det_trj(1, i) = det(J);
    
    % Calculate the Jacobian's Rank
    rank_trj(1, i) = rank(J);
      
    % Calculate the aditional q needed to achieve the forces
    q_add = J'*force;
    
    % Add the qvel to the trayectory's q
    q_trj(i,:) = q_trj(i,:)+q_add';
end

disp(['Trayectory (rad) (with aditional pairs)'])
q_trj
disp(['Jacobian Matrix Ranks : ', mat2str(rank_trj)])
disp(['Jacobian Matrix Determinats : ', mat2str(det_trj)])

p560.plot3d(q_trj, 'movie', 'images');
% p560.plot3d(q_trj);
%p560.plot(q_trj, 'movie', 'images');

% Store the old joint values
handles.q_old = q;

% Update handles structure
guidata(hObject, handles);

% Transformation Matrix from the Forward Kinematics
disp(['Transformation Matrix for q : ', mat2str(q)])
T = p560.fkine(q)

% Extract the X, Y, Z positions from the Transformation Matrix
Pos_X = T.t(1);
Pos_Y = T.t(2);
Pos_Z = T.t(3);

% Get the roll, pitch and yaw from the transformation matrix
rpy = tr2rpy(T)*180/pi;

% Display a message
disp(['End effector [Roll,Pitch,Yaw] - rpy : ', mat2str(rpy), ' deg'])

% Extract the roll, pitch and yaw from the rpy matrix
Roll_X = rpy(1);
Pitch_Y = rpy(2);
Yaw_Z = rpy(3);

% Pass the positions to the GUI
handles.Pos_X.String = num2str(Pos_X);
handles.Pos_Y.String = num2str(Pos_Y);
handles.Pos_Z.String = num2str(Pos_Z);
handles.Roll_X.String = num2str(fix(Roll_X));
handles.Pitch_Y.String = num2str(fix(Pitch_Y));
handles.Yaw_Z.String = num2str(fix(Yaw_Z));
handles.jacobian_ranks.String = sprintf('Jacobian Ranks (Trajectory):  %s',mat2str(rank_trj)) ;



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



function Roll_X_Callback(hObject, eventdata, handles)
% hObject    handle to Roll_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Roll_X as text
%        str2double(get(hObject,'String')) returns contents of Roll_X as a double


% --- Executes during object creation, after setting all properties.
function Roll_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Roll_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pitch_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pitch_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pitch_Y as text
%        str2double(get(hObject,'String')) returns contents of Pitch_Y as a double


% --- Executes during object creation, after setting all properties.
function Pitch_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pitch_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Yaw_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Yaw_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Yaw_Z as text
%        str2double(get(hObject,'String')) returns contents of Yaw_Z as a double


% --- Executes during object creation, after setting all properties.
function Yaw_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Yaw_Z (see GCBO)
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
disp('Inverse Kinematics for Puma 560')
mdl_puma560
% Get the desired positions from the GUI
PX = str2double(handles.Pos_X.String);
PY = str2double(handles.Pos_Y.String);
PZ = str2double(handles.Pos_Z.String);
Roll_X = str2double(handles.Roll_X.String)*180/pi;
Pitch_Y = str2double(handles.Pitch_Y.String)*180/pi;
Yaw_Z = str2double(handles.Yaw_Z.String)*180/pi;

force_x = str2double(handles.f_x.String);
force_y = str2double(handles.f_y.String);
force_z = str2double(handles.f_z.String);
torque_x = str2double(handles.tor_x.String);
torque_y = str2double(handles.tor_y.String);
torque_z = str2double(handles.tor_z.String);

% Create the force vector
force = [force_x; force_y; force_z; torque_x; torque_y; torque_z];

T = rpy2tr(Roll_X, Pitch_Y, Yaw_Z);
T(1,4) = PX;
T(2,4) = PY;
T(3,4) = PZ;

disp(['Transformation Matrix'])
T

q = p560.ikine(T);

%Create the trajectory
disp(['Trayectory (rad)'])

%Create the trajectory
q_trj = jtraj(handles.q_old, q, 10)

% Create a variable for the Jacobian's determiants
det_trj = zeros(1, size(q_trj, 1));

% Create a variable for the Jacobian's ranks
rank_trj = zeros(1, size(q_trj, 1));

for i = 1:size(q_trj, 1)
    % Check for singular value
    % Calculate the Jacobian Matrix
    J = p560.jacobe(q_trj(i,:));
    
    % Calculate the determinant
    det_trj(1, i) = det(J);
    
    % Calculate the Jacobian's Rank
    rank_trj(1, i) = rank(J);
    
    % Calculate the aditional q needed to achieve the forces
    q_add = J'*force;
    
    % Add the qvel to the trayectory's q
    q_trj(i,:) = q_trj(i,:)+q_add';

end

disp(['Trayectory (rad) (with aditional pairs)'])
q_trj
disp(['Jacobian Matrix Ranks : ', mat2str(rank_trj)])
disp(['Jacobian Matrix Determinats : ', mat2str(det_trj)])


p560.plot3d(q_trj, 'movie', 'images')
%p560.plot(q_trj);
%p560.plot(q_trj, 'movie', 'images');

% Store the old joint values
handles.q_old = q;

% Update handles structure
guidata(hObject, handles);

q = q*180/pi;

handles.Theta_1.String = num2str(fix(q(1)));
handles.Theta_2.String = num2str(fix(q(2)));
handles.Theta_3.String = num2str(fix(q(3)));
handles.Theta_4.String = num2str(fix(q(4)));
handles.Theta_5.String = num2str(fix(q(5)));
handles.Theta_6.String = num2str(fix(q(6)));
handles.jacobian_ranks.String = sprintf('Jacobian Ranks (Trajectory):  %s',mat2str(rank_trj)) ;



function f_x_Callback(hObject, eventdata, handles)
% hObject    handle to f_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of f_x as text
%        str2double(get(hObject,'String')) returns contents of f_x as a double


% --- Executes during object creation, after setting all properties.
function f_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to f_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function f_y_Callback(hObject, eventdata, handles)
% hObject    handle to f_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of f_y as text
%        str2double(get(hObject,'String')) returns contents of f_y as a double


% --- Executes during object creation, after setting all properties.
function f_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to f_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function f_z_Callback(hObject, eventdata, handles)
% hObject    handle to f_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of f_z as text
%        str2double(get(hObject,'String')) returns contents of f_z as a double


% --- Executes during object creation, after setting all properties.
function f_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to f_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tor_x_Callback(hObject, eventdata, handles)
% hObject    handle to tor_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tor_x as text
%        str2double(get(hObject,'String')) returns contents of tor_x as a double


% --- Executes during object creation, after setting all properties.
function tor_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tor_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tor_y_Callback(hObject, eventdata, handles)
% hObject    handle to tor_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tor_y as text
%        str2double(get(hObject,'String')) returns contents of tor_y as a double


% --- Executes during object creation, after setting all properties.
function tor_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tor_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tor_z_Callback(hObject, eventdata, handles)
% hObject    handle to tor_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tor_z as text
%        str2double(get(hObject,'String')) returns contents of tor_z as a double


% --- Executes during object creation, after setting all properties.
function tor_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tor_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
