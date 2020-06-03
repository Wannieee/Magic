%Magic cube in Matlab
%by望断飘渺<Wannieee>,2019,9
%使用方法：FinalDesign.m和FinalDesign.fig必须放在同一路径，再运行m文件。
function varargout = FinalDesign(varargin)
% FINALDESIGN MATLAB code for FinalDesign.fig
%      FINALDESIGN, by itself, creates a new FINALDESIGN or raises the existing
%      singleton*.
%
%      H = FINALDESIGN returns the handle to a new FINALDESIGN or the handle to
%      the existing singleton*.
%
%      FINALDESIGN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FINALDESIGN.M with the given input arguments.
%
%      FINALDESIGN('Property','Value',...) creates a new FINALDESIGN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before FinalDesign_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to FinalDesign_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help FinalDesign

% Last Modified by GUIDE v2.5 05-Sep-2019 13:45:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @FinalDesign_OpeningFcn, ...
                   'gui_OutputFcn',  @FinalDesign_OutputFcn, ...
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


% --- Executes just before FinalDesign is made visible.
function FinalDesign_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to FinalDesign (see VARARGIN)
%1白F 2黄B 3橙L 4红R 5蓝U 6绿D，a按该视角编号
global h;
global a;
global inface;
global b;
global state;
global state2;
global G;
global vv;
global flag;
flag=0;
vv=[5,5,5];
state=[1,0,0,0];
state2=1;%1为up，2为down
G=[1,4,2,3;2,3,1,4;3,1,4,2;4,2,3,1];
h=cell(1,54);
a=zeros(3,3,3);
a(:,:,1)=[1,2,3;4,5,6;7,8,9];
a(:,:,2)=[10,11,12;13,0,14;15,16,17];
a(:,:,3)=[18,19,20;21,22,23;24,25,26];
inface=cell(1,6);
b=[3,2,3,2,1,2,3,2,3,2,1,2,1,1,2,1,2,3,2,3,2,1,2,3,2,3];
c=[1,3,5;1,5,0;1,4,5;
   1,3,0;1,0,0;1,4,0;
   1,3,6;1,6,0;1,4,6;
   3,5,0;5,0,0;4,5,0;
   3,0,0;      4,0,0;
   3,6,0;6,0,0;4,6,0;
   2,3,5;2,5,0;2,4,5;
   2,3,0;2,0,0;2,4,0;
   2,3,6;2,6,0;2,4,6];
d={'w',[1,0.381,0],'b';'w','b',0;'w','r','b';
   'w',[1,0.381,0],0;'w',0,0;'w','r',0;
   'w',[1,0.381,0],'g';'w','g',0;'w','r','g';
   [1,0.381,0],'b',0;'b',0,0;'r','b',0;
   [1,0.381,0],0,0;           'r',0,0;
   [1,0.381,0],'g',0;'g',0,0;'r','g',0;
   'y',[1,0.381,0],'b';'y','b',0;'y','r','b';
   'y',[1,0.381,0],0;'y',0,0;'y','r',0;
   'y',[1,0.381,0],'g';'y','g',0;'y','r','g'};
face{1}=[ 1,-1, 1; 1,-1,-1; 1, 1,-1; 1, 1, 1];inface{1}=[face{1}(:,1),3*face{1}(:,2:3)];
face{2}=[-1,-1, 1;-1,-1,-1;-1, 1,-1;-1, 1, 1];inface{2}=[face{2}(:,1),3*face{2}(:,2:3)];
face{3}=[ 1,-1, 1;-1,-1, 1;-1,-1,-1; 1,-1,-1];inface{3}=[3*face{3}(:,1),face{3}(:,2),3*face{3}(:,3)];
face{4}=[ 1, 1, 1;-1, 1, 1;-1, 1,-1; 1, 1,-1];inface{4}=[3*face{4}(:,1),face{4}(:,2),3*face{4}(:,3)];
face{5}=[ 1, 1, 1;-1, 1, 1;-1,-1, 1; 1,-1, 1];inface{5}=[3*face{5}(:,1),3*face{5}(:,2),face{5}(:,3)];
face{6}=[ 1, 1,-1;-1, 1,-1;-1,-1,-1; 1,-1,-1];inface{6}=[3*face{6}(:,1),3*face{6}(:,2),face{6}(:,3)];
view(vv);
axis equal
axis off
hold on
xlabel('x')
ylabel('y')
zlabel('z')
u=1;v=1;
axis([-5 5 -5 5 -5 5])
for ii=1:3%x
    for kk=1:3%z
        for jj=1:3%y
            i=-2*ii+4;
            j=2*jj-4;
            k=-2*kk+4;
            if(i==0&&j==0&&k==0)
               continue
            end
            p=[i*ones(4,1),j*ones(4,1),k*ones(4,1)];
            for t=1:b(u)
                h{v}=patch('Faces',[1 2 3 4],'Vertices',face{c(u,t)}+p,'FaceColor',d{u,t});
                v=v+1;
            end
            u=u+1;
        end
    end
end
set(handles.pushbutton_up,'String','上')
set(handles.pushbutton_down,'String','下')
set(handles.pushbutton_left,'String','左')
set(handles.pushbutton_right,'String','右')
% Choose default command line output for FinalDesign
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes FinalDesign wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = FinalDesign_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_up.
function pushbutton_up_Callback(~,~,~)
% hObject    handle to pushbutton_up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global vv;
global state2;
if(state2==1)
    flag=0;
    return
end
state2=1;
theta=acos(vv(3)/norm(vv));
theta1=theta;
fai=atan2(vv(2),vv(1));
n=20;
for i=1:n
    theta1=theta1-(2*theta-pi)/n;
    x=5*sqrt(3)*sin(theta1)*cos(fai);
    y=5*sqrt(3)*sin(theta1)*sin(fai);
    z=5*sqrt(3)*cos(theta1);
    pause(0.01)
    view([x,y,z]);
end
vv=[vv(1),vv(2),-vv(3)];
view(vv)
flag=0;

% --- Executes on button press in pushbutton_right.
function pushbutton_right_Callback(~,~,~)
% hObject    handle to pushbutton_right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global state;
global vv;
state=[state(end),state(1:end-1)];
theta=acos(vv(3)/norm(vv));
fai=atan2(vv(2),vv(1));
fai1=fai;
n=20;
z=5*sqrt(3)*cos(theta);
for i=1:n
    fai1=fai1+pi/2/n;
    x=5*sqrt(3)*sin(theta)*cos(fai1);
    y=5*sqrt(3)*sin(theta)*sin(fai1);
    pause(0.01)
    view([x,y,z])
end
vv=[x,y,z];
flag=0;

% --- Executes on button press in pushbutton_left.
function pushbutton_left_Callback(~,~,~)
% hObject    handle to pushbutton_left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global state;
global vv;
state=[state(2:end),state(1)];
theta=acos(vv(3)/norm(vv));
fai=atan2(vv(2),vv(1));
fai1=fai;
n=20;
z=5*sqrt(3)*cos(theta);
for i=1:n
    fai1=fai1-pi/2/n;
    x=5*sqrt(3)*sin(theta)*cos(fai1);
    y=5*sqrt(3)*sin(theta)*sin(fai1);
    pause(0.01)
    view([x,y,z])
end
vv=[x,y,z];
flag=0;

% --- Executes on button press in pushbutton_down.
function pushbutton_down_Callback(~,~,~)
% hObject    handle to pushbutton_down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global vv;
global state2;
if(state2==2)
    flag=0;
    return
end
state2=2;
theta=acos(vv(3)/norm(vv));
theta1=theta;
fai=atan2(vv(2),vv(1));
n=20;
for i=1:n
    theta1=theta1-(2*theta-pi)/n;
    x=5*sqrt(3)*sin(theta1)*cos(fai);
    y=5*sqrt(3)*sin(theta1)*sin(fai);
    z=5*sqrt(3)*cos(theta1);
    pause(0.01)
    view([x,y,z]);
end
vv=[vv(1),vv(2),-vv(3)];
view(vv)
flag=0;

% --- Executes on button press in pushbutton_front1.
function pushbutton_front1_Callback(~, ~, ~)
% hObject    handle to pushbutton_front1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
global state;
global G;
option=[1,0,0,0]*G*state';
a=zhuan(a,h,inface,option,1);
flag=0;


% --- Executes on button press in pushbutton_before1.
function pushbutton_before1_Callback(~,~,~)
% hObject    handle to pushbutton_before1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
global state;
global G;
option=[0,1,0,0]*G*state';
a=zhuan(a,h,inface,option,1);
flag=0;

% --- Executes on button press in pushbutton_left1.
function pushbutton_left1_Callback(~,~,~)
% hObject    handle to pushbutton_left1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
global state;
global G;
option=[0,0,1,0]*G*state';
a=zhuan(a,h,inface,option,1);
flag=0;

% --- Executes on button press in pushbutton_right1.
function pushbutton_right1_Callback(~,~,~)
% hObject    handle to pushbutton_right1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
global state;
global G;
option=[0,0,0,1]*G*state';
a=zhuan(a,h,inface,option,1);
flag=0;

% --- Executes on button press in pushbutton_up1.
function pushbutton_up1_Callback(~,~,~)
% hObject    handle to pushbutton_up1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
option=5;
a=zhuan(a,h,inface,option,1);
flag=0;

% --- Executes on button press in pushbutton_down1.
function pushbutton_down1_Callback(~,~,~)
% hObject    handle to pushbutton_down1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
option=6;
a=zhuan(a,h,inface,option,1);
flag=0;

% --- Executes on button press in pushbutton_front2.
function pushbutton_front2_Callback(~,~,~)
% hObject    handle to pushbutton_front2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
global state;
global G;
option=[1,0,0,0]*G*state';
a=zhuan(a,h,inface,option,-1);
flag=0;

% --- Executes on button press in pushbutton_before2.
function pushbutton_before2_Callback(~,~,~)
% hObject    handle to pushbutton_before2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
global state;
global G;
option=[0,1,0,0]*G*state';
a=zhuan(a,h,inface,option,-1);
flag=0;

% --- Executes on button press in pushbutton_left2.
function pushbutton_left2_Callback(~,~,~)
% hObject    handle to pushbutton_left2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
global state;
global G;
option=[0,0,1,0]*G*state';
a=zhuan(a,h,inface,option,-1);
flag=0;

% --- Executes on button press in pushbutton_right2.
function pushbutton_right2_Callback(~,~,~)
% hObject    handle to pushbutton_right2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
global state;
global G;
option=[0,0,0,1]*G*state';
a=zhuan(a,h,inface,option,-1);
flag=0;

% --- Executes on button press in pushbutton_up2.
function pushbutton_up2_Callback(~,~,~)
% hObject    handle to pushbutton_up2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
option=5;
a=zhuan(a,h,inface,option,-1);
flag=0;

% --- Executes on button press in pushbutton_down2.
function pushbutton_down2_Callback(~,~,~)
% hObject    handle to pushbutton_down2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global flag
if(flag==1)
    return
end
flag=1;
global h;
global a;
global inface;
option=6;
a=zhuan(a,h,inface,option,-1);
flag=0;

function y=zhuan(x,h,inface,option,clockwise)
b=[3,2,3,2,1,2,3,2,3,2,1,2,1,1,2,1,2,3,2,3,2,1,2,3,2,3];
n=10;
theta=pi/2/n*clockwise;
y=x;
ffind=zeros(1,26);
check=zeros(1,54);
v=1;
hh=patch('Faces',[1 2 3 4],'Vertices',inface{option},'FaceColor','k');
gg=patch('Faces',[1 2 3 4],'Vertices',inface{option},'FaceColor','k');
switch option
    case 1
        rotation=[1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
        for i=1:3
            for k=1:3
                for j=1:3
                    if(i==2&&j==2&&k==2)
                        continue
                    end
                    if(i==1)
                        ffind(x(k,j,i))=1;
                    end
                end
            end
        end   
        if(clockwise==1)
            y=transition(x,option);
        end
        if(clockwise==-1)
            y=transition2(x,option);
        end
    case 2
        rotation=[1,0,0;0,cos(theta),sin(theta);0,-sin(theta),cos(theta)];
        for i=1:3
            for k=1:3
                for j=1:3
                    if(i==2&&j==2&&k==2)
                        continue
                    end
                    if(i==3)
                        ffind(x(k,j,i))=1;
                    end
                end
            end
        end
        if(clockwise==1)
            y=transition(x,option);
        end
        if(clockwise==-1)
            y=transition2(x,option);
        end
    case 3
        rotation=[cos(theta),0,-sin(theta);0,1,0;sin(theta),0,cos(theta)];
        for i=1:3
            for k=1:3
                for j=1:3
                    if(i==2&&j==2&&k==2)
                        continue
                    end
                    if(j==1)
                        ffind(x(k,j,i))=1;
                    end
                end
            end
        end
        if(clockwise==1)
            y=transition(x,option);
        end
        if(clockwise==-1)
            y=transition2(x,option);
        end
    case 4
        rotation=[cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)];
        for i=1:3
            for k=1:3
                for j=1:3
                    if(i==2&&j==2&&k==2)
                        continue
                    end
                    if(j==3)
                        ffind(x(k,j,i))=1;
                    end
                end
            end
        end
        if(clockwise==1)
            y=transition(x,option);
        end
        if(clockwise==-1)
            y=transition2(x,option);
        end
    case 5
        rotation=[cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];
        for i=1:3
            for k=1:3
                for j=1:3
                    if(i==2&&j==2&&k==2)
                        continue
                    end
                    if(k==1)
                        ffind(x(k,j,i))=1;
                    end
                end
            end
        end
        if(clockwise==1)
            y=transition(x,option);
        end
        if(clockwise==-1)
            y=transition2(x,option);
        end
    case 6
        rotation=[cos(theta),sin(theta),0;-sin(theta),cos(theta),0;0,0,1];
        for i=1:3
            for k=1:3
                for j=1:3
                    if(i==2&&j==2&&k==2)
                        continue
                    end
                    if(k==3)
                        ffind(x(k,j,i))=1;
                    end
                end
            end
        end
        if(clockwise==1)
            y=transition(x,option);
        end
        if(clockwise==-1)
            y=transition2(x,option);
        end
end
for e=1:length(b)
    for w=1:b(e)
        if(ffind(e)==1)
            check(v)=1;
        end
        v=v+1;
    end
end
for s=1:n
    for r=1:length(check)
        if(check(r)==1)
            h{r}.Vertices=(h{r}.Vertices)*rotation;
        end
    end
    hh.Vertices=(hh.Vertices)*rotation;
    pause(0.01)
    drawnow
end
gg.Vertices=[]; 
hh.Vertices=[];
drawnow

function y=transition(x,option)
y=x;
switch option
    case 1
        y(1,1,1)=x(3,1,1);
        y(3,1,1)=x(3,3,1);
        y(3,3,1)=x(1,3,1);
        y(1,3,1)=x(1,1,1);
        y(1,2,1)=x(2,1,1);
        y(2,1,1)=x(3,2,1);
        y(3,2,1)=x(2,3,1);
        y(2,3,1)=x(1,2,1);
    case 2
        y(3,1,3)=x(1,1,3);
        y(3,3,3)=x(3,1,3);
        y(1,3,3)=x(3,3,3);
        y(1,1,3)=x(1,3,3);
        y(2,1,3)=x(1,2,3);
        y(3,2,3)=x(2,1,3);
        y(2,3,3)=x(3,2,3);
        y(1,2,3)=x(2,3,3);
    case 3
        y(1,1,1)=x(1,1,3);
        y(1,1,3)=x(3,1,3);
        y(3,1,3)=x(3,1,1);
        y(3,1,1)=x(1,1,1);
        y(1,1,2)=x(2,1,3);
        y(2,1,3)=x(3,1,2);
        y(3,1,2)=x(2,1,1);
        y(2,1,1)=x(1,1,2);
    case 4
        y(1,3,3)=x(1,3,1);
        y(3,3,3)=x(1,3,3);
        y(3,3,1)=x(3,3,3);
        y(1,3,1)=x(3,3,1);
        y(2,3,3)=x(1,3,2);
        y(3,3,2)=x(2,3,3);
        y(2,3,1)=x(3,3,2);
        y(1,3,2)=x(2,3,1);
    case 5
        y(1,1,1)=x(1,3,1);
        y(1,3,1)=x(1,3,3);
        y(1,3,3)=x(1,1,3);
        y(1,1,3)=x(1,1,1);
        y(1,2,1)=x(1,3,2);
        y(1,3,2)=x(1,2,3);
        y(1,2,3)=x(1,1,2);
        y(1,1,2)=x(1,2,1);
    case 6
        y(3,3,1)=x(3,1,1);
        y(3,3,3)=x(3,3,1);
        y(3,1,3)=x(3,3,3);
        y(3,1,1)=x(3,1,3);
        y(3,3,2)=x(3,2,1);
        y(3,2,3)=x(3,3,2);
        y(3,1,2)=x(3,2,3);
        y(3,2,1)=x(3,1,2);
end

function y=transition2(x,option)
y=x;
switch option
    case 1
        y(3,1,1)=x(1,1,1);
        y(3,3,1)=x(3,1,1);
        y(1,3,1)=x(3,3,1);
        y(1,1,1)=x(1,3,1);
        y(2,1,1)=x(1,2,1);
        y(3,2,1)=x(2,1,1);
        y(2,3,1)=x(3,2,1);
        y(1,2,1)=x(2,3,1);
    case 2
        y(1,1,3)=x(3,1,3);
        y(3,1,3)=x(3,3,3);
        y(3,3,3)=x(1,3,3);
        y(1,3,3)=x(1,1,3);
        y(1,2,3)=x(2,1,3);
        y(2,1,3)=x(3,2,3);
        y(3,2,3)=x(2,3,3);
        y(2,3,3)=x(1,2,3);
    case 3
        y(1,1,3)=x(1,1,1);
        y(3,1,3)=x(1,1,3);
        y(3,1,1)=x(3,1,3);
        y(1,1,1)=x(3,1,1);
        y(2,1,3)=x(1,1,2);
        y(3,1,2)=x(2,1,3);
        y(2,1,1)=x(3,1,2);
        y(1,1,2)=x(2,1,1);
    case 4
        y(1,3,1)=x(1,3,3);
        y(1,3,3)=x(3,3,3);
        y(3,3,3)=x(3,3,1);
        y(3,3,1)=x(1,3,1);
        y(1,3,2)=x(2,3,3);
        y(2,3,3)=x(3,3,2);
        y(3,3,2)=x(2,3,1);
        y(2,3,1)=x(1,3,2);
    case 5
        y(1,3,1)=x(1,1,1);
        y(1,3,3)=x(1,3,1);
        y(1,1,3)=x(1,3,3);
        y(1,1,1)=x(1,1,3);
        y(1,3,2)=x(1,2,1);
        y(1,2,3)=x(1,3,2);
        y(1,1,2)=x(1,2,3);
        y(1,2,1)=x(1,1,2);
    case 6
        y(3,1,1)=x(3,3,1);
        y(3,3,1)=x(3,3,3);
        y(3,3,3)=x(3,1,3);
        y(3,1,3)=x(3,1,1);
        y(3,2,1)=x(3,3,2);
        y(3,3,2)=x(3,2,3);
        y(3,2,3)=x(3,1,2);
        y(3,1,2)=x(3,2,1);
end



% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(~,~, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'Value',1)
set(handles.radiobutton2,'Value',0)
set(handles.radiobutton3,'Value',0)
set(handles.radiobutton4,'Value',0)
set(handles.radiobutton5,'Value',0)
set(handles.radiobutton6,'Value',0)
set(handles.radiobutton7,'Value',0)
% Hint: get(hObject,'Value') returns toggle state of radiobutton1


% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(~,~, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'Value',0)
set(handles.radiobutton2,'Value',1)
set(handles.radiobutton3,'Value',0)
set(handles.radiobutton4,'Value',0)
set(handles.radiobutton5,'Value',0)
set(handles.radiobutton6,'Value',0)
set(handles.radiobutton7,'Value',0)
% Hint: get(hObject,'Value') returns toggle state of radiobutton2


% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(~,~, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'Value',0)
set(handles.radiobutton2,'Value',0)
set(handles.radiobutton3,'Value',1)
set(handles.radiobutton4,'Value',0)
set(handles.radiobutton5,'Value',0)
set(handles.radiobutton6,'Value',0)
set(handles.radiobutton7,'Value',0)
% Hint: get(hObject,'Value') returns toggle state of radiobutton3


% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(~,~, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'Value',0)
set(handles.radiobutton2,'Value',0)
set(handles.radiobutton3,'Value',0)
set(handles.radiobutton4,'Value',1)
set(handles.radiobutton5,'Value',0)
set(handles.radiobutton6,'Value',0)
set(handles.radiobutton7,'Value',0)
% Hint: get(hObject,'Value') returns toggle state of radiobutton4


% --- Executes on button press in radiobutton5.
function radiobutton5_Callback(~,~, handles)
% hObject    handle to radiobutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'Value',0)
set(handles.radiobutton2,'Value',0)
set(handles.radiobutton3,'Value',0)
set(handles.radiobutton4,'Value',0)
set(handles.radiobutton5,'Value',1)
set(handles.radiobutton6,'Value',0)
set(handles.radiobutton7,'Value',0)
% Hint: get(hObject,'Value') returns toggle state of radiobutton5


% --- Executes on button press in radiobutton6.
function radiobutton6_Callback(~,~, handles)
% hObject    handle to radiobutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'Value',0)
set(handles.radiobutton2,'Value',0)
set(handles.radiobutton3,'Value',0)
set(handles.radiobutton4,'Value',0)
set(handles.radiobutton5,'Value',0)
set(handles.radiobutton6,'Value',1)
set(handles.radiobutton7,'Value',0)
% Hint: get(hObject,'Value') returns toggle state of radiobutton6


% --- Executes on button press in radiobutton7.
function radiobutton7_Callback(~,~, handles)
% hObject    handle to radiobutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'Value',0)
set(handles.radiobutton2,'Value',0)
set(handles.radiobutton3,'Value',0)
set(handles.radiobutton4,'Value',0)
set(handles.radiobutton5,'Value',0)
set(handles.radiobutton6,'Value',0)
set(handles.radiobutton7,'Value',1)
% Hint: get(hObject,'Value') returns toggle state of radiobutton7


% --- Executes on button press in push.
function push_Callback(~,~, handles)
% hObject    handle to push (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=(get(handles.radiobutton1,'Value')==1)*1;
a=a+(get(handles.radiobutton2,'Value')==1)*2;
a=a+(get(handles.radiobutton3,'Value')==1)*3;
a=a+(get(handles.radiobutton4,'Value')==1)*4;
a=a+(get(handles.radiobutton5,'Value')==1)*5;
a=a+(get(handles.radiobutton6,'Value')==1)*6;
a=a+(get(handles.radiobutton7,'Value')==1)*7;
switch a
    case 1
        pushbutton_right1_Callback
        pushbutton_up1_Callback
        pushbutton_right2_Callback
        pushbutton_up2_Callback
    case 2
        pushbutton_up1_Callback
        pushbutton_right1_Callback
        pushbutton_up2_Callback
        pushbutton_right2_Callback
        pushbutton_up2_Callback
        pushbutton_front2_Callback
        pushbutton_up1_Callback
        pushbutton_front1_Callback
    case 3
        pushbutton_up2_Callback
        pushbutton_front2_Callback
        pushbutton_up1_Callback
        pushbutton_front1_Callback
        pushbutton_up1_Callback
        pushbutton_right1_Callback
        pushbutton_up2_Callback
        pushbutton_right2_Callback
    case 4
        pushbutton_front1_Callback
        pushbutton_right1_Callback
        pushbutton_up1_Callback
        pushbutton_right2_Callback
        pushbutton_up2_Callback
        pushbutton_front2_Callback
    case 5
        pushbutton_right1_Callback
        pushbutton_up1_Callback
        pushbutton_right2_Callback
        pushbutton_up1_Callback
        pushbutton_right1_Callback
        pushbutton_up1_Callback
        pushbutton_up1_Callback
        pushbutton_right2_Callback
    case 6
        pushbutton_up1_Callback
        pushbutton_right1_Callback
        pushbutton_up2_Callback
        pushbutton_left2_Callback
        pushbutton_up1_Callback
        pushbutton_right2_Callback
        pushbutton_up2_Callback
        pushbutton_left1_Callback
    case 7
        pushbutton_right2_Callback
        pushbutton_down2_Callback
        pushbutton_right1_Callback
        pushbutton_down1_Callback
end


% --- Executes on button press in randon.
function randon_Callback(~,~,~)
% hObject    handle to randon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
n=15;
s={'front','before','left','right','up','down'};
jj=100;
for i=1:n
    w=60*rand;
    j=floor(w/10)+1;
    if(jj==j)
        continue
    end
    jj=j;
    k=floor(mod(w,2))+1;
    ss=['pushbutton_',s{j},num2str(k),'_Callback'];
    eval(ss)
end
