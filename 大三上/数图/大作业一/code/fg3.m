%����ͼƬ
I=imread('..\data\����\3.bmp');
%����ǲ�ɫ��ת�Ҷ�
if numel(size(I))==3
    I=rgb2gray(I);
end
I=im2double(I);

%����
[r,c]=size(I);
%��ȫ�߽�
rn=ceil(r/8);
cn=ceil(c/8);
Ip=I;  %��ȫͼƬ
if 8*rn>r
    wr=ones([8*rn-r,c]);
    Ip=[Ip;wr];
end
if 8*cn>c
    wc=ones([8*rn,8*cn-c]);
    Ip=[Ip,wc];
end
%�ױ�ƴ��
w1=ones([12,8*cn+24]);
w2=ones([8*rn,12]);
Ip=[w1;w2,Ip,w2;w1];
I_p=Ip;
%ǰ���ָ�
iw=ones([8,8]);
iw32=ones([48,48]);
iw_32=zeros([48,48]);

mask=zeros(size(Ip));
mk=zeros([rn,cn]);
Idis=zeros([rn,cn]);  %����
M=zeros([rn,cn]);  %��ֵ
S=zeros([rn,cn]);  %����
for i=3:rn-4
    for j=3:cn-4
        icrop=imcrop(I_p,[8*(j-1)+1,8*(i-1)+1,31,31]);  %�и�
        iabs=abs(fftshift(fft2(icrop)));  %����dft
        [x,y]=find(iabs==max(max(iabs)));  %ȥ������ֱ��
        iabs(x,y)=0;
        [x,y]=find(iabs==max(max(iabs)));  %���������ֵ
        if size(x,1)==2
            dis=(sqrt((x(1)-17)^2+(y(1)-17)^2)+sqrt((x(1)-17)^2+(y(1)-17)^2))/2;
            Idis(i,j)=dis;
            if dis<3 || dis>7
                
            else
                imean=mean2(icrop);
                istd=std2(icrop);
                if imean<0.5 && istd>0.01  %��Ϊ��ָ������
                    M(i,j)=imean;
                    S(i,j)=istd;
                    mask(8*(i-1)-7:8*(i-1)+40,8*(j-1)-7:8*(j-1)+40)=iw32;  %��Χ�������Ϊ��ָ������
                    mk(i-1:i+5,j-1:j+5)=1;
                end
            end
        end
    end
end

%ȥ���Ϻڲ���
Ip(Ip<0.05)=1;
I_p=Ip;

% ��̬ѧ�任
B=[0,1,0;1,1,1;0,1,0];
I_p=imdilate(I_p,B);
I_p=imdilate(I_p,B);
I_p=imdilate(I_p,B);
I_p=imerode(I_p,B);
Ip(I_p<0.2)=1;

fun2 = @(x) std2(x);
std_local = nlfilter(Ip,[3 3],fun2);
Ip(std_local>0.2)=1;
Ip(Ip>0.55)=1;

Ip=imbinarize(Ip,0.3);
% figure;
% imshow(Ip);

%ǰ��ͼ
for i=1:r
    for j=1:c
        if mask(i,j)==0
            I(i,j)=0;
        end
    end
end
figure;
imshow(I);

Id=zeros([32*rn,32*cn]);  %���dftͼ��
Iang=zeros([rn,cn]);  %��Ƕ�
If=zeros([rn,cn]);  %Ƶ��ͼ
Iflag=zeros([rn,cn]);
for i=1:rn
    for j=1:cn
        if mk(i,j)==1
            icrop=imcrop(Ip,[8*(j-1)+1,8*(i-1)+1,31,31]);  %�и�
            iabs=abs(fftshift(fft2(icrop)));  %����dft
            [x,y]=find(iabs==max(max(iabs)));  %ȥ������ֱ��
            iabs(x,y)=0;
            [x,y]=find(iabs==max(max(iabs)));  %���������ֵ
            io=ones(32,32);
            io(x(1),y(1))=0;
            io(x(2),y(2))=0;
            Id(32*(i-1)+1:32*i,32*(j-1)+1:32*j)=io;
            dis=(sqrt((x(1)-17)^2+(y(1)-17)^2)+sqrt((x(1)-17)^2+(y(1)-17)^2))/2;
            If(i,j)=dis;

            if (x(1)<17&&y(1)<17)||(x(2)<17&&y(2)<17)
                Iflag(i,j)=1;
                Iang(rn-i+1,j)=atan(abs(y(2)-y(1))/abs(x(2)-x(1)));
            else
                Iflag(i,j)=2;
                Iang(rn-i+1,j)=pi-atan(abs(y(2)-y(1))/abs(x(2)-x(1)));
            end
        end
    end
end

%dft����ͼ
% figure;
% imshow(Id);

%Ƶ��ͼ
%ƽ��
fun1 = @(x) mean2(x);
I_f = nlfilter(If,[5 5],fun1);
If=1.-If./(max(max(If)));
figure;
imshow(If);
figure;
imshow(1.-I_f./(max(max(I_f))));

% ����ͼ
figure;
[x,y]=meshgrid(1:cn,1:rn);
h=quiver(x,y,cos(Iang),sin(Iang));
set(h,'ShowArrowHead','off')
% ƽ��
%ƽ������ͼ
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
%��������
for i=1:rn
    flag=0;
    for j=1:cn
        if Ia(rn-i+1,j)<0
            flag=1;
        end
        if flag==1
            if sin(Ia(rn-i+1,j))<-0.9 || sin(Ia(rn-i+1,j))==1
                Ia(rn-i+1,j)=-30/180*pi;
            end
            if Ia(rn-i+1,j)>0.1
                Ia(rn-i+1,j)=Ia(rn-i+1,j)+90/180*pi;
            end
        end
    end
end
h=quiver(x,y,cos(Ia),sin(Ia));
set(h,'ShowArrowHead','off')
%�ٴ�ƽ��
Ia=Ia.*2;
Ias=sin(Ia);
Iac=cos(Ia);
gsf=fspecial('Gaussian',[3,3],3);
Ias = imfilter(Ias,gsf,'replicate');
Iac = imfilter(Iac,gsf,'replicate');
Ia=atan2(Ias,Iac)/2;
h=quiver(x,y,cos(Ia),sin(Ia));
set(h,'ShowArrowHead','off')

%�ݲ��˲�
Ifilt1=zeros(size(Ip));  %���˲���ָ��
for i=1:rn
    for j=1:cn
        if mk(i,j)~=0
            icrop=imcrop(Ip,[8*(j-1)+1,8*(i-1)+1,31,31]);  %�и�
            iabs=(fftshift(fft2(icrop)));  %����dft
            if Iflag(i,j)==1
                x1=floor(16.5-abs(I_f(i,j)*cos(pi/2-Ia(rn-i+1,j)))+0.5);
                y1=floor(16.5-abs(I_f(i,j)*sin(pi/2-Ia(rn-i+1,j)))+0.5);
            else
                x1=floor(16.5-abs(I_f(i,j)*cos(Ia(rn-i+1,j)-pi/2))+0.5);
                y1=floor(16.5+abs(I_f(i,j)*sin(Ia(rn-i+1,j)-pi/2))+0.5);
            end
            p = [x1 y1];
            H = ones(size(iabs));
            [DX, DY] = meshgrid(1:32,1:32);
            D0 = 6;
            n = 250;
            D1 = sqrt((DX-p(1)).^2+(DY-p(2)).^2);
            D2 = sqrt((DX-32-2+p(1)).^2+(DY-32-2+p(2)).^2);
            H1 = 1./(1+(D0./D1).^(2*n));
            H2 = 1./(1+(D0./D2).^(2*n));
            H = H.*H1.*H2;
            H(17,17)=0;
            iabs = (1-H).*iabs;
        end
        g = real(ifft2(ifftshift(iabs)));
        Ifilt1(8*(i-1)+1:8*i,8*(j-1)+1:8*j)=imcrop(g,[1,1,7,7]);
    end
end
for i=1:8*rn
    for j=1:8*cn
        if mask(i,j)==0
            Ifilt1(i,j)=0;
        end
    end
end
Ifilt1=imcrop(Ifilt1,[1,1,c-1,r-1]);
Ifilt1 = imbinarize(Ifilt1,'adaptive','ForegroundPolarity','dark','Sensitivity',0.5);
figure;
imshow(Ifilt1);


%gabor
Ip=double(Ip);
Ifilt2=zeros(size(Ip));  %���˲���ָ��
for i=1:rn
    for j=1:cn
        if mk(i,j)==1  %ָ������
            icrop=imcrop(Ip,[8*(j-1)+1,8*(i-1)+1,31,31]);  %�и�
            if I_f(i,j)*2>=2
                wavelength = I_f(i,j)*2;
                orientation = Ia(rn-i+1,j)*180/pi+90;
                [mag,phase] = imgaborfilt(icrop,wavelength,orientation);
                Ifilt2(8*(i-1)+1:8*i,8*(j-1)+1:8*j)=imcrop(mag.*cos(phase),[1,1,7,7]);
            end
        end
        
    end
end
for i=1:8*rn
    for j=1:8*cn
        if mask(i,j)==0
            Ifilt2(i,j)=0;
        end
    end
end
Ifilt2=imcrop(Ifilt2,[1,1,c-1,r-1]);
Ifilt2 = nlfilter(Ifilt2,[3 3],fun1);
Ifilt2 = imbinarize(Ifilt2,'adaptive','ForegroundPolarity','dark','Sensitivity',0.55);
figure;
imshow(Ifilt2);