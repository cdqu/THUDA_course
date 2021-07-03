function varargout = snap(varargin)
% SNAP MATLAB code for snap.fig
%      SNAP, by itself, creates a new SNAP or raises the existing
%      singleton*.
%
%      H = SNAP returns the handle to a new SNAP or the handle to
%      the existing singleton*.
%
%      SNAP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SNAP.M with the given input arguments.
%
%      SNAP('Property','Value',...) creates a new SNAP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before snap_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to snap_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help snap

% Last Modified by GUIDE v2.5 11-Jan-2020 12:03:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @snap_OpeningFcn, ...
                   'gui_OutputFcn',  @snap_OutputFcn, ...
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


% --- Executes just before snap is made visible.
function snap_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to snap (see VARARGIN)

% Choose default command line output for snap
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
global path I foresub backsub;
foresub=[];
backsub=[];
path='..\data\输入\';
I=imread('..\data\输入\targetImage1.jpg');
axes(handles.axes1);
imshow(I);
% UIWAIT makes snap wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = snap_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in ppm_pic.
function ppm_pic_Callback(hObject, eventdata, handles)
% hObject    handle to ppm_pic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ppm_pic contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ppm_pic


% --- Executes during object creation, after setting all properties.
function ppm_pic_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ppm_pic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in ppm_ccs.
function ppm_ccs_Callback(hObject, eventdata, handles)
% hObject    handle to ppm_ccs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ppm_ccs contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ppm_ccs


% --- Executes during object creation, after setting all properties.
function ppm_ccs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ppm_ccs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in ppm_time.
function ppm_time_Callback(hObject, eventdata, handles)
% hObject    handle to ppm_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ppm_time contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ppm_time


% --- Executes during object creation, after setting all properties.
function ppm_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ppm_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_slic.
function btn_slic_Callback(hObject, eventdata, handles)
% hObject    handle to btn_slic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global I idx Ic I_orig clus_center;
I_orig=I;
%[M,N,dig]=size(I);
[Ic,num]=superpixels(I,3000);
BW = boundarymask(Ic); 
axes(handles.axes2);
imshow(imoverlay(I_orig,BW,'cyan'));
% I=rgb2lab(I);
%初始化聚类中心
% num=get(handles.ppm_ccs,'value');
% switch num
%     case 1
%         K=100;
%     case 2
%         K=400;
%     case 3
%         K=800;
%     case 4
%         K=1000;
%     case 5
%         K=1600;
% end
% s=sqrt(M*N/K);
% ms=floor(M/s);
% ns=floor(N/s);
% clus=zeros(M,N);
% for i=1:ms
%     for j=1:ns
%         clus(floor(s*i),floor(s*j))=1;
%     end
% end
% idx=find(clus);
% %找最小梯度
% g=zeros(3,3);
% for id=1:size(idx,1)
%     [r_,c_]=ind2sub([M N],idx(id));
%     for r=r_-1:r_+1
%         for c=c_-1:c_+1
%             v1=[I(r+1,c,1),I(r+1,c,2),I(r+1,c,3),r+1,c];
%             v2=[I(r-1,c,1),I(r-1,c,2),I(r-1,c,3),r-1,c];
%             v3=[I(r,c+1,1),I(r,c+1,2),I(r,c+1,3),r,c+1];
%             v4=[I(r,c-1,1),I(r,c-1,2),I(r,c-1,3),r,c-1];
%             g(r-r_+2,c-c_+2)=norm(v1-v2)+norm(v3-v4);
%         end
%     end
%     [rmin,cmin]=ind2sub([3 3],find(g==min(min((g)))));
%     clus(r_,c_)=0;
%     clus(r_+rmin(1)-2,c_+cmin(1)-2)=1;
% end
% %迭代次数
% num=get(handles.ppm_time,'value');
% switch num
%     case 1
%         t=3;
%     case 2
%         t=5;
%     case 3
%         t=10;
%     case 4
%         t=15;
% end
% %xy参数占比
% if get(handles.radiobutton1,'value')
%     para1=0.2;
% elseif get(handles.radiobutton2,'value')
%     para1=0.5;
% elseif get(handles.radiobutton3,'value')
%     para1=0.7;
% elseif get(handles.radiobutton3,'value')
%     para1=1;
% end
% 
% %是否加入gabor优化
% ifgab=get(handles.cb_gab,'value');
% if ifgab==0  %否
%     idx=find(clus);
%     %初始化聚类中心信息
%     clus_center=zeros(ms*ns,5);
%     for id=1:size(idx,1)
%         [r,c]=ind2sub([M N],idx(id));
%         clus_center(id,1)=I(r,c,1);
%         clus_center(id,2)=I(r,c,2);
%         clus_center(id,3)=I(r,c,3);
%         clus_center(id,4)=r;
%         clus_center(id,5)=c;
%     end
%     %迭代更新聚类
%     %初始化
%     Ic=zeros(M,N);  %类别
%     Ic(:)=-1;
%     Id=zeros(M,N);  %距离
%     Id(:)=inf;
% 
%     for times=1:t  %迭代次数
%         for id=1:size(idx,1)
%             r=clus_center(id,4);
%             c=clus_center(id,5);
%             %在周围2S*2S计算距离
%             rst=floor(r-s); cst=floor(c-s);
%             ren=floor(r+s); cen=floor(c+s);
%             if rst<1
%                 rst=1;
%             end
%             if cst<1
%                 cst=1;
%             end
%             if ren>M
%                 ren=M;
%             end
%             if cen>N
%                 cen=N;
%             end
%             for i=rst:ren
%                 for j=cst:cen
%                     %lab距离和xy距离按比例para相加
%                     d_lab=sqrt((I(i,j,1)-clus_center(id,1))^2+(I(i,j,2)-clus_center(id,2))^2+(I(i,j,3)-clus_center(id,3))^2);
%                     d_xy=sqrt((i-clus_center(id,4))^2+(j-clus_center(id,5))^2);
%                     dis=d_lab+para1*d_xy;
%                     if dis<Id(i,j)
%                         Id(i,j)=dis;
%                         Ic(i,j)=id;
%                     end
%                 end
%             end
%         end
%         %更新聚类中心
%         for id=1:size(idx)
%             pix=find(Ic==id);
%             num=size(pix,1);
%             lnew=sum(I(pix))/num;
%             anew=sum(I(pix+M*N))/num;
%             bnew=sum(I(pix+2*M*N))/num;
%             [x,y]=ind2sub([M,N],pix);
%             xnew=sum(x)/num;
%             ynew=sum(y)/num;
%             clus_center(id,1)=lnew;
%             clus_center(id,2)=anew;
%             clus_center(id,3)=bnew;
%             clus_center(id,4)=xnew;
%             clus_center(id,5)=ynew; 
%         end
%         %显示
%         BW = boundarymask(Ic);
%         axes(handles.axes2);
%         cla reset;
%         imshow(imoverlay(I_orig,BW,'cyan'));
%         hold on,
%         plot(int64(clus_center(:,5)),int64(clus_center(:,4)),'b.');
%         pause(1);
%     end
% else  %是
%     %加入gabor优化
%     %生成滤波器
%     wavelength = 2.^(0:5) * 3;
%     orientation = 0:45:135;
%     g = gabor(wavelength,orientation);
%     Ig = rgb2gray(im2single(I_orig));
%     gabormag = imgaborfilt(Ig,g);
% 
%     idx=find(clus);
%     %初始化聚类中心信息
%     clus_center=zeros(ms*ns,5);
%     gab_center=zeros(ms*ns,8);
%     for id=1:size(idx,1)
%         [r,c]=ind2sub([M N],idx(id));
%         clus_center(id,1)=I(r,c,1);
%         clus_center(id,2)=I(r,c,2);
%         clus_center(id,3)=I(r,c,3);
%         clus_center(id,4)=r;
%         clus_center(id,5)=c;
%         for m=1:24
%             gab_center(id,m)=gabormag(r,c,m);
%         end
%     end
% 
%     %迭代更新聚类
%     %初始化
%     Ic=zeros(M,N);  %类别
%     Ic(:)=-1;
%     Id=zeros(M,N);  %距离
%     Id(:)=inf;
% 
%     if get(handles.radiobutton5,'value')
%         para2=0.1;
%     elseif get(handles.radiobutton6,'value')
%         para2=0.2;
%     elseif get(handles.radiobutton7,'value')
%         para2=0.5;
%     elseif get(handles.radiobutton8,'value')
%         para2=0.7;
%     end
%     for times=1:t  %迭代次数
%         for id=1:size(idx,1)
%             r=clus_center(id,4);
%             c=clus_center(id,5);
%             %在周围2S*2S计算距离
%             rst=floor(r-s); cst=floor(c-s);
%             ren=floor(r+s); cen=floor(c+s);
%             if rst<1
%                 rst=1;
%             end
%             if cst<1
%                 cst=1;
%             end
%             if ren>M
%                 ren=M;
%             end
%             if cen>N
%                 cen=N;
%             end
%             for i=rst:ren
%                 for j=cst:cen
%                     %lab距离和xy距离按比例para相加
%                     d_lab=sqrt((I(i,j,1)-clus_center(id,1))^2+(I(i,j,2)-clus_center(id,2))^2+(I(i,j,3)-clus_center(id,3))^2);
%                     d_xy=sqrt((i-clus_center(id,4))^2+(j-clus_center(id,5))^2);
%                     s_um=0;
%                     for m=1:24
%                         s_um=s_um+(gabormag(i,j,m)-gab_center(id,m)).^2;
%                     end
%                     d_gab=sqrt(s_um);
%                     dis=d_lab+para1*d_xy+para1*para2*d_gab;
%                     if dis<Id(i,j)
%                         Id(i,j)=dis;
%                         Ic(i,j)=id;
%                     end
%                 end
%             end
%         end
%         %更新聚类中心
%         for id=1:size(idx)
%             pix=find(Ic==id);
%             num=size(pix,1);
%             lnew=sum(I(pix))/num;
%             anew=sum(I(pix+M*N))/num;
%             bnew=sum(I(pix+2*M*N))/num;
%             [x,y]=ind2sub([M,N],pix);
%             xnew=sum(x)/num;
%             ynew=sum(y)/num;
%             clus_center(id,1)=lnew;
%             clus_center(id,2)=anew;
%             clus_center(id,3)=bnew;
%             clus_center(id,4)=xnew;
%             clus_center(id,5)=ynew; 
%             for m=1:24
%                 gab_center(id,m)=sum(gabormag(pix+(m-1)*M*N))/num;
%             end
%         end
%         %显示
%         BW = boundarymask(Ic);
%         axes(handles.axes2);
%         imshow(imoverlay(I_orig,BW,'cyan'));
%         hold on,
%         plot(int64(clus_center(:,5)),int64(clus_center(:,4)),'b.');
%     end
% end
% %显示 
% BW = boundarymask(Ic); axes(handles.axes2);
% imshow(imoverlay(I_orig,BW,'cyan'));
% hold on,
% plot(int64(clus_center(:,5)),int64(clus_center(:,4)),'b.');

% --- Executes on button press in btn_orig.
function btn_orig_Callback(hObject, eventdata, handles)  %选择图片
% hObject    handle to btn_orig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global I path;
delete(allchild(handles.axes2));
val=get(handles.ppm_pic,'value');
switch val
    case 1
        I=imread(strcat(path,'targetImage1.jpg'));
    case 2
        I=imread(strcat(path,'targetImage2.jpg'));
    case 3
        I=imread(strcat(path,'targetImage3.jpg'));
end
axes(handles.axes1);
imshow(I);


% --- Executes on button press in btn_lty.
function btn_lty_Callback(hObject, eventdata, handles)
% hObject    handle to btn_lty (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Ic I_orig idx clus_center;
[M,N]=size(Ic);
Is=zeros(M,N);
%获得每个点的连通域大小
for j=1:size(idx,1)
    mt=zeros(M,N);
    mt(Ic==j)=1;
    [L,n]=bwlabel(mt,4);
    for i=1:n
        poi=find(L==i);
        Is(poi)=size(poi,1);
    end
end
%遍历，将连通域小于一定阈值的点类型设为邻居点
for i=1:M
    for j=1:N
        if Is(i,j)<100
            a=i-1;
            b=j-1;
            if a<1
                a=1;
            end
            if b<1
                b=1;
            end
            Ic(i,j)=Ic(a,b);
        end
    end
end
%显示
BW = boundarymask(Ic);
axes(handles.axes2);
imshow(imoverlay(I_orig,BW,'cyan'));
hold on,
plot(int64(clus_center(:,5)),int64(clus_center(:,4)),'b.');

% --- Executes on button press in btn_fore.
function btn_fore_Callback(hObject, eventdata, handles)
% hObject    handle to btn_fore (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global I_orig foresub;
axes(handles.axes1);
imshow(I_orig);
h1=drawpolyline('color','b');
foresub=[foresub;h1.Position];
foresub=int64(foresub);

% --- Executes on button press in btn_back.
function btn_back_Callback(hObject, eventdata, handles)
% hObject    handle to btn_back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global I_orig backsub;
axes(handles.axes1);
imshow(I_orig);
h2=drawpolyline('color','r');
backsub=[backsub;h2.Position];
backsub=int64(backsub);

% --- Executes on button press in btn_ack.
function btn_ack_Callback(hObject, eventdata, handles)
% hObject    handle to btn_ack (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Ic I_orig foregroundInd backgroundInd foresub backsub maskedImage;
foregroundInd = sub2ind(size(I_orig),foresub(:,2),foresub(:,1));
backgroundInd = sub2ind(size(I_orig),backsub(:,2),backsub(:,1));
BW = lazysnapping(I_orig,Ic,foregroundInd,backgroundInd);
maskedImage = I_orig;
maskedImage(repmat(~BW,[1 1 3])) = 0;
axes(handles.axes2);
imshow(maskedImage);

% --- Executes on button press in cb_gab.
function cb_gab_Callback(hObject, eventdata, handles)
% hObject    handle to cb_gab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cb_gab


% --- Executes on button press in btn_save.
function btn_save_Callback(hObject, eventdata, handles)
% hObject    handle to btn_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global maskedImage path;
imwrite(maskedImage,strcat(path,'a.jpg'));
