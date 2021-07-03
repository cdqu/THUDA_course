I=imread('..\pic\输入\sourceImage.jpg');
%选择风格
style=6;
switch style
    case 1
        I=youhua(I);  %油画
    case 2
        I=dongman(I);  %动漫
    case 3
        I_=jianbi(I);  %简笔画
        I(:,:,1)=I_;
        I(:,:,2)=I_;
        I(:,:,3)=I_;
    case 4
        I=qiangti(I);  %墙体
    case 5
        I=xingkong(I);  %叠加
    case 6
        I=fudiao(I);  %浮雕
        I_=imread('..\pic\输入\pic6.jpg');
        I(:,:,1)=I_;
        I(:,:,2)=I_;
        I(:,:,3)=I_;
end
% imwrite(I,'..\pic\6.jpg');
%选择图片
picnum=1;
switch picnum
    case 1
        output=pic1(I);
    case 2
        output=pic2(I);
    case 3
        output=pic3(I);
    case 4
        output=pic4(I);
    case 5
        output=pic5(I);
    case 6
        output=pic6(I);
end
% imwrite(output,'..\pic\out6.jpg');
%% 图片预处理函数
%图1
function resImg=pic1(srcImg)
resImg = imread('..\pic\输入\targetImage1.jpg');
%变换到目标图片
srcImg=imcrop(srcImg,[444.5 0.5 994 1200]);
[h1,w1,d1] = size(srcImg);
xs1 = [1 w1 1 w1]';
ys1 = [1 1 h1 h1]';

[h2,w2,d2] = size(resImg);
xs2=[185;667.5;185;667.5];
ys2=[180.5;180.5;797;797];

tform = fitgeotrans([xs1 ys1],[xs2 ys2],'projective');
src_registered = imwarp(srcImg,tform,'OutputView',imref2d(size(resImg)));
mask = sum(src_registered,3)~=0;
idx = find(mask);
resImg(idx) = src_registered(idx);
resImg(idx+h2*w2) = src_registered(idx+h2*w2);
resImg(idx+2*h2*w2) = src_registered(idx+2*h2*w2);

mask=imread('..\pic\输入\a.jpg');
%对蒙版做处理，去除人像边缘噪声
mask=rgb2hsv(mask);
for i=1:h2
    for j=1:w2
        if mask(i,j,2)==1
            mask(i,j,:)=0;
        end
    end
end
mask=hsv2rgb(mask);
mg=rgb2gray(mask);
for i=1:h2
    for j=1:w2
        if mg(i,j)~=0
            mg(i,j)=1;
        end
    end
end
se1 = strel('square',2);
mg=imerode(mg,se1);
mg=bwareaopen(mg,50);
mg=~bwareaopen(~mg,50);
%与mask结合
for j=187:(187+480)
    for i=180:797
        if mask(i,j,:)>=0.01 & mg(i,j)==1
            resImg(i,j,:)=mask(i,j,:)*255;
        end
    end
end
figure,
imshow(resImg);
end

%图2
function resImg=pic2(srcImg)
[h1,w1,d1] = size(srcImg);
xs1 = [1 w1 1 w1]';
ys1 = [1 1 h1 h1]';

resImg = imread('..\pic\输入\targetImage2.jpg');
[h2,w2,d2] = size(resImg);
xs2=[46;543;18;592];
ys2=[46;50;372;413];
%变换到目标
tform = fitgeotrans([xs1 ys1],[xs2 ys2],'projective');
src_registered = imwarp(srcImg,tform,'OutputView',imref2d(size(resImg)));
mask = sum(src_registered,3)~=0;
idx = find(mask);
resImg(idx) = src_registered(idx);
resImg(idx+h2*w2) = src_registered(idx+h2*w2);
resImg(idx+2*h2*w2) = src_registered(idx+2*h2*w2);

m1=imread('..\pic\输入\b.jpg');
m1=rgb2hsv(m1);
for i=1:h2
    for j=1:w2
        if m1(i,j,2)==1
            m1(i,j,:)=0;
        end
    end
end
m1=hsv2rgb(m1);
mg=rgb2gray(m1);
for i=1:h2
    for j=1:w2
        if mg(i,j)~=0
            mg(i,j)=1;
        end
    end
end
se1 = strel('square',2);
mg=imerode(mg,se1);
mg=bwareaopen(mg,20);
mg=~bwareaopen(~mg,20);

for j=501:(501+42)
    for i=366:(366+45)
        if m1(i,j,:)~=0 & mg(i,j)==1
            resImg(i,j,:)=m1(i,j,:)*255;
        end
    end
end
figure,imshow(resImg);
end

%图3
function resImg=pic3(srcImg)
srcImg=imcrop(srcImg,[444.5 0.5 994 1200]);
[h1,w1,d1] = size(srcImg);
xs1 = [1 w1 1 w1]';
ys1 = [1 1 h1 h1]';

resImg = imread('..\pic\输入\targetImage3.jpg');
[h2,w2,d2] = size(resImg);
xs2=[355;568;394;600];
ys2=[119;95;646;541];

tform = fitgeotrans([xs1 ys1],[xs2 ys2],'projective');
src_registered = imwarp(srcImg,tform,'OutputView',imref2d(size(resImg)));
mask = sum(src_registered,3)~=0;
idx = find(mask);
resImg(idx) = src_registered(idx);
resImg(idx+h2*w2) = src_registered(idx+h2*w2);
resImg(idx+2*h2*w2) = src_registered(idx+2*h2*w2);

m1=imread('..\pic\输入\c.jpg');
m1=rgb2hsv(m1);
for i=1:h2
    for j=1:w2
        if m1(i,j,2)==1
            m1(i,j,:)=0;
        end
    end
end
m1=hsv2rgb(m1);
mg=rgb2gray(m1);
for i=1:h2
    for j=1:w2
        if mg(i,j)~=0
            mg(i,j)=1;
        end
    end
end
se1 = strel('square',2);
mg=imerode(mg,se1);
mg=bwareaopen(mg,20);
mg=~bwareaopen(~mg,20);
for j=448:(448+48)
    for i=102:(102+45)
        if m1(i,j,:)~=0 & mg(i,j)==1
            resImg(i,j,:)=m1(i,j,:)*255;
        end
    end
end
figure,imshow(resImg);
end

%图4
function resImg=pic4(srcImg)
srcImg=imcrop(srcImg,[444.5 0.5 994 1200]);
[h1,w1,d1] = size(srcImg);
xs1 = [1 w1 1 w1]';
ys1 = [1 1 h1 h1]';

resImg = imread('..\pic\输入\targetImage4.jpg');
[h2,w2,d2] = size(resImg);
xs2=[370;795;243;653];
ys2=[80;185;706;794];

tform = fitgeotrans([xs1 ys1],[xs2 ys2],'projective');
src_registered = imwarp(srcImg,tform,'OutputView',imref2d(size(resImg)));
mask = sum(src_registered,3)~=0;
idx = find(mask);
resImg(idx) = src_registered(idx);
resImg(idx+h2*w2) = src_registered(idx+h2*w2);
resImg(idx+2*h2*w2) = src_registered(idx+2*h2*w2);

m1=imread('..\pic\输入\e.jpg');
m1=rgb2hsv(m1);
for i=1:h2
    for j=1:w2
        if m1(i,j,2)==1
            m1(i,j,:)=0;
        end
    end
end

m1=hsv2rgb(m1);

mg=rgb2gray(m1);
for i=1:h2
    for j=1:w2
        if mg(i,j)~=0
            mg(i,j)=1;
        end
    end
end
se1 = strel('square',3);
mg=imerode(mg,se1);
mg=imdilate(mg,se1);
mg=bwareaopen(mg,50);
mg=~bwareaopen(~mg,50);
for j=202:202+606
    for i=265:265+509
        if m1(i,j,:)~=0 & mg(i,j)==1
            resImg(i,j,:)=m1(i,j,:)*255;
        end
    end
end
figure,imshow(resImg);
end

%图5
function resImg=pic5(srcImg)
[h1,w1,d1] = size(srcImg);
xs1 = [1 w1 1 w1]';
ys1 = [1 1 h1 h1]';

resImg = imread('..\pic\输入\targetImage5.jpg');
[h2,w2,d2] = size(resImg);
xs2=[98;708;96;719];
ys2=[107;174;560;577];

tform = fitgeotrans([xs1 ys1],[xs2 ys2],'projective');
src_registered = imwarp(srcImg,tform,'OutputView',imref2d(size(resImg)));
mask = sum(src_registered,3)~=0;
idx = find(mask);
resImg(idx) = src_registered(idx);
resImg(idx+h2*w2) = src_registered(idx+h2*w2);
resImg(idx+2*h2*w2) = src_registered(idx+2*h2*w2);
figure,
imshow(resImg);
end

%图6
function resImg=pic6(srcImg)
srcImg=imcrop(srcImg,[400 0.5 1050 1200]);
[h1,w1,d1] = size(srcImg);
xs1 = [1 w1 1 w1]';
ys1 = [1 1 h1 h1]';

resImg = imread('..\pic\输入\targetImage6.jpg');
[h2,w2,d2] = size(resImg);
xs2=[627;974;662;1018];
ys2=[112;78;495;458];

tform = fitgeotrans([xs1 ys1],[xs2 ys2],'projective');
src_registered = imwarp(srcImg,tform,'OutputView',imref2d(size(resImg)));
mask = sum(src_registered,3)~=0;
idx = find(mask);
resImg(idx) = src_registered(idx);
resImg(idx+h2*w2) = src_registered(idx+h2*w2);
resImg(idx+2*h2*w2) = src_registered(idx+2*h2*w2);
figure,
imshow(resImg);
end
%% 风格函数
%油画风格
function resImg=youhua(I)
[L,N] = superpixels(I,2000);
resImg = zeros(size(I),'like',I);
idx = label2idx(L);
R=size(I,1);
C=size(I,2);
for label = 1:N
    redIdx = idx{label};
    greenIdx = idx{label}+R*C;
    blueIdx = idx{label}+2*R*C;
    resImg(redIdx) = mean(I(redIdx));
    resImg(greenIdx) = mean(I(greenIdx));
    resImg(blueIdx) = mean(I(blueIdx));
end
h=fspecial('disk',3);
resImg=imfilter(resImg,h);
figure
imshow(resImg,'InitialMagnification',67);
end

% 动漫风格
function resImg=dongman(I)
imLAB = rgb2lab(I);
degreeOfSmoothing = 15;
spatialSigma = 5;
nFigure = 1;
for k = 1:15
    fprintf('%d,',k)
    imLAB = imbilatfilt(imLAB,degreeOfSmoothing,spatialSigma);
    if k==1 || mod(k,5)==0
        smImg = lab2rgb(imLAB,'Out','uint8');
        nFigure = nFigure + 1;
    end
end
h1=[1 1 1; 1 -8 1; 1 1 1];
J=imfilter(smImg,h1);
resImg=smImg-J;

figure,
imshow(resImg);
end

%简笔画风格
function resImg=jianbi(I)
[PPG]=colorgrad(I);
ppg=im2uint8(PPG);
ppgf=255-ppg;
[M,N]=size(ppgf);
T=140;
resImg=zeros(M,N);
for i=1:M
    for j=1:N
        if ppgf(i,j)<T
            resImg(i,j)=0;
        else
            resImg(i,j)=235/(255-T)*(ppgf(i,j)-T);
        end
    end
end
resImg=uint8(resImg);
figure,
imshow(resImg);
end

function [PPG] = colorgrad(f)
sh = fspecial('sobel');
sv = sh';
Rx = imfilter(double(f(:,:,1)), sh, 'replicate');
Ry = imfilter(double(f(:,:,1)), sv, 'replicate');
Gx = imfilter(double(f(:,:,2)), sh, 'replicate');
Gy = imfilter(double(f(:,:,2)), sv, 'replicate');
Bx = imfilter(double(f(:,:,3)), sh, 'replicate');
By = imfilter(double(f(:,:,3)), sv, 'replicate');
 
RG = sqrt(Rx.^2 + Ry.^2);
GG = sqrt(Gx.^2 + Gy.^2);
BG = sqrt(Bx.^2 + By.^2);
 
PPG = mat2gray(RG + GG + BG);
end

%墙体风格
function resImg=qiangti(I)
[r,c,dig]=size(I);
resImg=I;
w=floor(r/18);
for i=w/2:w:18*w
    for j=1:c
        for z=-3:0
           resImg(i+z,j,:)=I(i-1,j,:)-50;
        end
        for z=1:4
           resImg(i+z,j,:)=I(i-1,j,:)+50;
        end
    end
end
figure;
imshow(resImg);
end

%星空
function resImg=xingkong(I)
[r,c,dig]=size(I);
J=imread('..\pic\输入\timg.jpg');
J=imresize(J,[r,c],'Method','bicubic');
mask=imread('..\pic\输入\d.jpg');
resImg = imlincomb(0.7,I,0.3,J);
for i=1:r
    for j=1:c
        if mask(i,j,:)==0
            resImg(i,j,:)=I(i,j,:);
        end
    end
end
figure;
imshow(resImg);
end

%浮雕
function resImg=fudiao(I)
f1=rgb2gray(I);
h1=[1 1 1; 1 -8 1; 1 1 1];
J=imfilter(f1,h1);
f1=f1-J;
f1=fftshift(fft2(f1));
P1 = angle(f1);

F1_phase = cos(P1) + 1i*sin(P1);
resImg = real((ifft2(ifftshift(F1_phase))));
resImg=imadjust(resImg,[0,max(max(resImg))],[0.2,0.95]);
figure,imshow(resImg);
imwrite(resImg,'..\pic\输入\pic6.jpg');
end