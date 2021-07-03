%读入图片
I=imread('..\data\输入\77_8.bmp');
%如果是彩色，转灰度
if numel(size(I))==3
    I=rgb2gray(I);
end
% imshow(I);
I=im2double(I);
%长宽
[r,c]=size(I);
% for i=1:r
%     for j=1:c
%         if I(i,j)>0.99
%             I(i,j)=1;
%         end
%     end
% end
%补全边界
rn=ceil(r/8);
cn=ceil(c/8);
Ip=I;  %补全图片
if 8*rn>r
    wr=ones([8*rn-r,c]);
    Ip=[Ip;wr];
end
if 8*cn>c
    wc=ones([8*rn,8*cn-c]);
    Ip=[Ip,wc];
end
%白边拼接
w1=ones([12,8*cn+24]);
w2=ones([8*rn,12]);
Ip=[w1;w2,Ip,w2;w1];

Id=zeros([32*rn,32*cn]);  %存放dft图像
flag=zeros([8*rn,8*cn]);  %是否是指纹区域
Iang=zeros([rn,cn]);  %存角度
If=zeros([rn,cn]);  %频率图

for i=1:rn
    for j=1:cn
        icrop=imcrop(Ip,[8*(j-1)+1,8*(i-1)+1,31,31]);  %切割
        iabs=abs(fftshift(fft2(icrop)));  %计算dft
        [x,y]=find(iabs==max(max(iabs)));  %去掉中心直流
        iabs(x,y)=0;
        [x,y]=find(iabs==max(max(iabs)));  %找最大两个值
        if size(x,1)==2 
            %频率（距中心距离）
            dis=(sqrt((x(1)-16.5)^2+(y(1)-16.5)^2)+sqrt((x(1)-16.5)^2+(y(1)-16.5)^2))/2;
            if dis<14.5086 && dis>1 %认为是指纹区域
                io=ones(32,32);
                io(x(1),y(1))=0;
                io(x(2),y(2))=0;
                Id(32*(i-1)+1:32*i,32*(j-1)+1:32*j)=io;
                %计算角度
                if (x(1)<17&&y(1)<17)||(x(2)<17&&y(2)<17)
                    flag(i,j)=1;
                    Iang(rn-i+1,j)=atan(abs(y(2)-y(1))/abs(x(2)-x(1)));
                else
                    flag(i,j)=2;
                    Iang(rn-i+1,j)=pi-atan(abs(y(2)-y(1))/abs(x(2)-x(1)));
                end
                If(i,j)=dis;
            end
        end
    end
end

iw=zeros([8,8]);
%前景图
for i=1:rn
    for j=1:cn
        if flag(i,j)==0
            I(8*(i-1)+1:8*i,8*(j-1)+1:8*j)=iw;
        end
    end
end
figure;
imshow(I);

%dft幅度图
% figure(1);
% imshow(Id);

%频率图
%平滑
fun1 = @(x) mean2(x);
I_f = nlfilter(If,[3 3],fun1);
If=1.-If./(max(max(If)));
figure(2);
imshow(If);
figure(3);
imshow(1.-I_f./(max(max(I_f))));

%方向图
figure(4);
[x,y]=meshgrid(1:cn,1:rn);
h=quiver(x,y,cos(Iang),sin(Iang));
set(h,'ShowArrowHead','off')
%平滑
Ia=Iang.*2;
Ias=sin(Ia);
Iac=cos(Ia);
gsf=fspecial('Gaussian',[3,3],3);
Ias = imfilter(Ias,gsf,'replicate');
Iac = imfilter(Iac,gsf,'replicate');
Ia=atan2(Ias,Iac)/2;
figure;
[x,y]=meshgrid(1:cn,1:rn);
h=quiver(x,y,cos(Ia),sin(Ia));
set(h,'ShowArrowHead','off')

%陷波滤波
Ifilt1=zeros(size(Ip));
for i=1:rn
    for j=1:cn
        icrop=imcrop(Ip,[8*(j-1)+1,8*(i-1)+1,31,31]);  %切割
        iabs=(fftshift(fft2(icrop)));  %计算dft
        if flag(i,j)~=0
            if flag(i,j)==1
                x1=floor(16.5-abs(I_f(i,j)*cos(pi/2-Ia(rn-i+1,j)))+0.5);
                y1=floor(16.5-abs(I_f(i,j)*sin(pi/2-Ia(rn-i+1,j)))+0.5);
            else
                x1=floor(16.5-abs(I_f(i,j)*cos(Ia(rn-i+1,j)-pi/2))+0.5);
                y1=floor(16.5+abs(I_f(i,j)*sin(Ia(rn-i+1,j)-pi/2))+0.5);
            end
            p = [x1 y1];
            H = ones(size(iabs));
            [DX, DY] = meshgrid(1:32,1:32);
            D0 = 3;
            n = 250;
            D1 = sqrt((DX-p(1)).^2+(DY-p(2)).^2);
            D2 = sqrt((DX-32-2+p(1)).^2+(DY-32-2+p(2)).^2);
            H1 = 1./(1+(D0./D1).^(2*n));
            H2 = 1./(1+(D0./D2).^(2*n));
            H = H.*H1.*H2;
            H(17,17)=0;
            iabs = (1-H).*iabs;
            g = real(ifft2(ifftshift(iabs)));
            Ifilt1(8*(i-1)+1:8*i,8*(j-1)+1:8*j)=imcrop(g,[13,13,7,7]);
        end
    end
end
figure;
Ifilt1=imcrop(Ifilt1,[1,1,c-1,r-1]);
Ifilt1=imbinarize(Ifilt1,0.85);
imshow(Ifilt1);

%gabor
Ifilt2=zeros(size(Ip));
for i=1:rn
    for j=1:cn
        icrop=imcrop(Ip,[8*(j-1)+1,8*(i-1)+1,31,31]);  %切割
        if flag(i,j)~=0
            if I_f(i,j)>=2
            wavelength = I_f(i,j)*2;
            orientation = Ia(rn-i+1,j)*180/pi+90;
            [mag,phase] = imgaborfilt(icrop,wavelength,orientation);
            Ifilt2(8*(i-1)+1:8*i,8*(j-1)+1:8*j)=imcrop(mag.*cos(phase),[13,13,7,7]);
            end
        end
        
    end
end
figure;
Ifilt2=imcrop(Ifilt2,[1,1,c-1,r-1]);
imshow(Ifilt2);
