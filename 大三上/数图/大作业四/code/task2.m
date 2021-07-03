video = VideoReader('..\pic\输入\targetVideo.MP4');
nFrames=video.NumberOfFrames;
vh=video.Height;
vw=video.Width;
%生成的视频
aviobj = VideoWriter('..\pic\输出\newvideo.avi');
%设置帧率
aviobj.FrameRate = 25;
open(aviobj);
%新封面
cover=imread('..\pic\输入\cover.jpg');
cover=im2double(cover);
[h1,w1,d1] = size(cover);
xs1 = [1 w1 w1 1]';
ys1 = [1 1 h1 h1]';
%在第一帧取角点
frame1=read(video,1);
frame1=im2double(frame1);
imshow(frame1);
[x,y]=ginput(4);  %左上 右上 右下 左下

[h2,w2,d2] = size(frame1);
xs2=double(x);
ys2=double(y);
tf = estimateGeometricTransform([xs1 ys1],[xs2 ys2],'projective');
src_registered = imwarp(cover,tf,'OutputView',imref2d(size(frame1)));
mask= sum(src_registered,3)~=0;
idx = find(mask);
frame1(idx) = src_registered(idx);
frame1(idx+h2*w2) = src_registered(idx+h2*w2);
frame1(idx+2*h2*w2) = src_registered(idx+2*h2*w2);
figure;
imshow(frame1);
% hold on,
line([x(1),x(2)],[y(1),y(2)],'Color','r','LineWidth',2);
line([x(2),x(3)],[y(2),y(3)],'Color','r','LineWidth',2);
line([x(3),x(4)],[y(3),y(4)],'Color','r','LineWidth',2);
line([x(4),x(1)],[y(4),y(1)],'Color','r','LineWidth',2);

srcp=[y,x];

%从第265帧获取手部颜色，便于之后抠图
pic1=read(video,305);
pic1=im2double(pic1);
pic1=rgb2hsv(pic1);

R1=55/255;
pos1=[827 638];
pos2=[816 573];
pos3=[895 558];
a1 = [pic1(pos1(2),pos1(1),1) pic1(pos1(2),pos1(1),2) pic1(pos1(2),pos1(1),3)];
a2 = [pic1(pos2(2),pos2(1),1) pic1(pos2(2),pos2(1),2) pic1(pos2(2),pos2(1),3)];
a3=[pic1(pos3(2),pos3(1),1) pic1(pos3(2),pos3(1),2) pic1(pos3(2),pos3(1),3)];

for k=1:nFrames-1
    % 获得相邻两帧
    frame1=read(video,k);
    frame1=im2double(frame1);
    frame2=read(video,k+1);
    frame2=im2double(frame2);
    % 求取两帧的SURF特征点
    fg1=rgb2gray(frame1);
    p1=detectSURFFeatures(fg1);
    [f1,p1]=extractFeatures(fg1, p1);

    fg2=rgb2gray(frame2);
    p2 = detectSURFFeatures(fg2);
    [f2, p2] = extractFeatures(fg2, p2);

    %特征点匹配
    pair=matchFeatures(f1, f2);

    point1 = p1.Location;
    point2 = p2.Location;
    % 求解变换矩阵
    tform = estimateGeometricTransform([point1(pair(:,1),2) point1(pair(:,1),1)],[point2(pair(:,2),2) point2(pair(:,2),1)],'projective');
    % 使用变换矩阵求得下一帧角点位置
    destp=transformPointsForward(tform,srcp);
    
    if k>=250 && mod(k,5)==0  % 霍夫变换矫正
        I=frame2;
        [M,N,dig]=size(I);
        %对图片做处理，方便后续直线检测
        I=rgb2hsv(I);
        R2=70/255;
        color1=[0.533333333333333 0.724637681159420 0.270588235294118];
        color2=[0.552757793764988 0.615044247787611 0.886274509803922];
        D1 = (I(:,:,1)-color1(1)).^2+(I(:,:,2)-color1(2)).^2+(I(:,:,3)-color1(3)).^2;
        mask1 = D1<=R2*R2;
        D2 = (I(:,:,1)-color2(1)).^2+(I(:,:,2)-color2(2)).^2+(I(:,:,3)-color2(3)).^2;
        mask2 = D2<=R2*R2;
        m_ask=mask1|mask2;
        m_ask=~bwareaopen(~m_ask,10000);
        I=hsv2rgb(I);
        %霍夫变换
        bw=edge(m_ask,'sobel');
        [H,theta,rho] = hough(bw);
        P = houghpeaks(H,7,'threshold',ceil(0.3*max(H(:))));
        lines = houghlines(bw,theta,rho,P);
        max_len = 0; count =0;
        for kp = 1:length(lines)
           xy = [lines(kp).point1; lines(kp).point2];
        end
        cross=[];
        %找交点
        for i=1:length(lines)-1
            for j=i+1:length(lines)
                xy1 = [lines(i).point1; lines(i).point2];
                aa1 = xy1(1,2) -xy1(2,2);
                bb1 = xy1(2,1) -xy1(1,1);
                cc1 = xy1(1,1) *xy1(2,2) - xy1(2,1) * xy1(1,2);
                xy2 = [lines(j).point1; lines(j).point2];
                aa2 = xy2(1,2) -xy2(2,2);
                bb2 = xy2(2,1) -xy2(1,1);
                cc2 = xy2(1,1) *xy2(2,2) - xy2(2,1) * xy2(1,2);
                d = aa1*bb2 - aa2*bb1;

                if d~=0
                    cross_x = (bb1*cc2 - bb2*cc1)/d;
                    cross_y = (aa2*cc1 - aa1*cc2)/d;
                    cross=[cross;cross_y,cross_x];
                end
            end
        end
        %矫正当前destp位置，移至最近交点
        dis=pdist2(destp,cross);
        [min_dis,index]=min(dis,[],2);
        %有距离小于10的，矫正
        for q=1:4
            if min_dis(q,1)<10
                destp(q,:)=cross(index(q,1),:);
            end
        end
    end
    %换封面
    if k>=265 && k<=348  %需要对手部做处理
        %获得mask
        frame2=rgb2hsv(frame2);
        D1 = (frame2(:,:,1)-a1(1)).^2+(frame2(:,:,2)-a1(2)).^2+(frame2(:,:,3)-a1(3)).^2;
        mask1 = D1<=R1*R1;
        D2 = (frame2(:,:,1)-a2(1)).^2+(frame2(:,:,2)-a2(2)).^2+(frame2(:,:,3)-a2(3)).^2;
        mask2 = D2<=R1*R1;
        D3 = (frame2(:,:,1)-a3(1)).^2+(frame2(:,:,2)-a3(2)).^2+(frame2(:,:,3)-a3(3)).^2;
        mask3 = D3<=R1*R1;
        mm=mask1|mask2|mask3;
        frame2=hsv2rgb(frame2);
        pic2=frame2;
    end
    %对新的封面仿射变换至原封面
    xs2=double(destp(:,2));
    ys2=double(destp(:,1));

    tf = fitgeotrans([xs1 ys1],[xs2 ys2],'projective');
    src_registered = imwarp(cover,tf,'OutputView',imref2d(size(frame2)));
    mask= sum(src_registered,3)~=0;
    idx = find(mask);
    frame2(idx) = src_registered(idx);
    frame2(idx+h2*w2) = src_registered(idx+h2*w2);
    frame2(idx+2*h2*w2) = src_registered(idx+2*h2*w2);
    
    if k>=265 && k<=348  %对手部处理
        %抠图
        ind=find(mm);
        frame2(ind)=pic2(ind);
        frame2(ind+h2*w2)=pic2(ind+h2*w2);
        frame2(ind+2*h2*w2)=pic2(ind+2*h2*w2);
    end
    %展示
    imshow(frame2);
    line([destp(1,2),destp(2,2)],[destp(1,1),destp(2,1)],'Color','r','LineWidth',2);
    line([destp(2,2),destp(3,2)],[destp(2,1),destp(3,1)],'Color','r','LineWidth',2);
    line([destp(3,2),destp(4,2)],[destp(3,1),destp(4,1)],'Color','r','LineWidth',2);
    line([destp(4,2),destp(1,2)],[destp(4,1),destp(1,1)],'Color','r','LineWidth',2);
    pause(0.01);
    writeVideo(aviobj,frame2);
    %更新
    srcp=destp;
end
close(aviobj);