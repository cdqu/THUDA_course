I=imread('C:\Users\12516\Desktop\学习\数图\数字图象处理_综合作业3\data\3.jpg');
I_orig=I;
[M,N,dig]=size(I);
I=rgb2lab(I);
%初始化聚类中心
K=1000;
s=sqrt(M*N/K);
ms=floor(M/s);
ns=floor(N/s);
clus=zeros(M,N);
for i=1:ms
    for j=1:ns
        clus(floor(s*i),floor(s*j))=1;
    end
end
% figure;
% imshow(clus);
idx=find(clus);
%找最小梯度
g=zeros(3,3);
for id=1:size(idx,1)
    [r_,c_]=ind2sub([M N],idx(id));
    for r=r_-1:r_+1
        for c=c_-1:c_+1
            v1=[I(r+1,c,1),I(r+1,c,2),I(r+1,c,3),r+1,c];
            v2=[I(r-1,c,1),I(r-1,c,2),I(r-1,c,3),r-1,c];
            v3=[I(r,c+1,1),I(r,c+1,2),I(r,c+1,3),r,c+1];
            v4=[I(r,c-1,1),I(r,c-1,2),I(r,c-1,3),r,c-1];
            g(r-r_+2,c-c_+2)=norm(v1-v2)+norm(v3-v4);
        end
    end
    [rmin,cmin]=ind2sub([3 3],find(g==min(min((g)))));
    clus(r_,c_)=0;
    clus(r_+rmin(1)-2,c_+cmin(1)-2)=1;
end
% figure;
% imshow(clus);

%加入gabor优化
%生成滤波器
wavelength = 2.^(0:5) * 3;
orientation = 0:45:135;
g = gabor(wavelength,orientation);

Ig = rgb2gray(im2single(I_orig));
gabormag = imgaborfilt(Ig,g);

idx=find(clus);
%初始化聚类中心信息
clus_center=zeros(ms*ns,5);
gab_center=zeros(ms*ns,8);
for id=1:size(idx,1)
    [r,c]=ind2sub([M N],idx(id));
    clus_center(id,1)=I(r,c,1);
    clus_center(id,2)=I(r,c,2);
    clus_center(id,3)=I(r,c,3);
    clus_center(id,4)=r;
    clus_center(id,5)=c;
%     for m=1:24
%         gab_center(id,m)=gabormag(r,c,m);
%     end
end

%迭代更新聚类
%初始化
Ic=zeros(M,N);  %类别
Ic(:)=-1;
Id=zeros(M,N);  %距离
Id(:)=inf;

para1=0.8;
para2=0.3;
for times=1:10  %迭代次数
    for id=1:size(idx,1)
        r=clus_center(id,4);
        c=clus_center(id,5);
        %在周围2S*2S计算距离
        rst=floor(r-s); cst=floor(c-s);
        ren=floor(r+s); cen=floor(c+s);
        if rst<1
            rst=1;
        end
        if cst<1
            cst=1;
        end
        if ren>M
            ren=M;
        end
        if cen>N
            cen=N;
        end
        for i=rst:ren
            for j=cst:cen
                %lab距离和xy距离按比例para相加
                d_lab=sqrt((I(i,j,1)-clus_center(id,1))^2+(I(i,j,2)-clus_center(id,2))^2+(I(i,j,3)-clus_center(id,3))^2);
                d_xy=sqrt((i-clus_center(id,4))^2+(j-clus_center(id,5))^2);
                
%                 d_gab=sqrt((gabormag(i,j,1)-gab_center(id,1))^2+(gabormag(i,j,2)-gab_center(id,2))^2+...
%                     (gabormag(i,j,3)-gab_center(id,3))^2+(gabormag(i,j,4)-gab_center(id,4))^2+...
%                     (gabormag(i,j,5)-gab_center(id,5))^2+(gabormag(i,j,6)-gab_center(id,6))^2+...
%                     (gabormag(i,j,7)-gab_center(id,7))^2+(gabormag(i,j,8)-gab_center(id,8))^2);
                s_um=0;
                for m=1:24
                    s_um=s_um+(gabormag(i,j,m)-gabormag(int64(clus_center(id,4)),int64(clus_center(id,5)),m)).^2;
                end
                d_gab=sqrt(s_um);
                dis=d_lab+para1*d_xy+para1*para2*d_gab;
                if dis<Id(i,j)
                    Id(i,j)=dis;
                    Ic(i,j)=id;
                end
            end
        end
    end
    %更新聚类中心
    for id=1:size(idx)
        pix=find(Ic==id);
        num=size(pix,1);
        lnew=sum(I(pix))/num;
        anew=sum(I(pix+M*N))/num;
        bnew=sum(I(pix+2*M*N))/num;
        [x,y]=ind2sub([M,N],pix);
        xnew=sum(x)/num;
        ynew=sum(y)/num;
        clus_center(id,1)=lnew;
        clus_center(id,2)=anew;
        clus_center(id,3)=bnew;
        clus_center(id,4)=xnew;
        clus_center(id,5)=ynew; 
%         
%         for m=1:24
%             gab_center(id,m)=sum(gabormag(pix+(m-1)*M*N))/num;
%         end
    end
end
% 
%连通域处理
Is=zeros(M,N);
%获得每个点的连通域大小
% for j=1:size(idx,1)
%     tic;
%     mt=zeros(M,N);
%     mt(Ic==j)=1;
%     L=bwlabel(mt,4);
%     poi=find(mt);
%     for i=1:size(poi,1)
%         [r,c]=ind2sub([M,N],poi(i));
%         s=sum(L(:)==L(r,c));
%         Is(r,c)=s;
%     end
%     toc;
% end
% for j=1:size(idx,1)
%     mt=zeros(M,N);
%     mt(Ic==j)=1;
%     [L,n]=bwlabel(mt,4);
%     for i=1:n
%         poi=find(L==i);
%         Is(poi)=size(poi,1);
%     end
% end
% 
% %遍历，将连通域小于一定阈值的点类型设为邻居点
% yz=M*N/size(idx,1)/5;
% for i=1:M
%     for j=1:N
%         if Is(i,j)<100
%             a=i-1;
%             b=j-1;
%             if a<1
%                 a=1;
%             end
%             if b<1
%                 b=1;
%             end
%             Ic(i,j)=Ic(a,b);
%         end
%     end
% end
%显示
for id=1:size(idx)
    pix=find(Ic==id);
    I(pix)=clus_center(id,1);
    I(pix+M*N)=clus_center(id,2);
    I(pix+2*M*N)=clus_center(id,3);
end
I=lab2rgb(I);
figure;
imshow(I);

BW = boundarymask(Ic);
figure;
imshow(imoverlay(I_orig,BW,'cyan'));

% figure;
% imshow(I_orig);
% % [x,y]=ginput(20);
% % h1=impoly(gca,[x,y],'Closed',false);
% % foresub = getPosition(h1);
% h1=drawpolyline('color','b');
% a=h1.Position;
% foresub=[];
% foresub=[foresub;a];
% foregroundInd = int64(sub2ind(size(I_orig),foresub(:,2),foresub(:,1)));
% h2=drawpolyline('color','r');
% b=h2.Position;
% backsub=[];
% backsub=[backsub;b];
% backgroundInd = int64(sub2ind(size(I_orig),backsub(:,2),backsub(:,1)));
% BW = lazysnapping(I_orig,Ic,foregroundInd,backgroundInd);
% maskedImage = I_orig;
% maskedImage(repmat(~BW,[1 1 3])) = 0;
% figure; 
% imshow(maskedImage);
