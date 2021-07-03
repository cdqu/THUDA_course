I=imread('..\data\输入\r2_5.bmp');
I=im2double(I);
[M,N]=size(I);
figure;
imshow(I);
%二值化
I=imbinarize(I,0.5);
figure;
imshow(I);
%形态学处理
I=~bwareaopen(~I,50);
I=bwareaopen(I,50);
figure;
imshow(I);
%细化
Ibw = ~bwmorph(~I, 'thin', inf);
figure;
imshow(Ibw);
%细化后处理
C = ~Pruning(~Ibw, 5);
%去除桥接
B1=[0,1,1;1,0,1;1,1,0];
B2=[1,0,0;0,1,0;0,0,1];
hm=bwhitmiss(~Ibw,B1,B2);
[x,y]=find(hm==1);
C(x-1:x+1,y-1:y+1)=1;
figure;
imshow(C);
%细节点检测
Icn=zeros(size(I));
for i=2:M-1
    for j=2:N-1
        if C(i,j)==0
            cn=abs(C(i-1,j)-C(i-1,j-1))+abs(C(i-1,j+1)-C(i-1,j))+abs(C(i,j+1)-C(i-1,j+1))...
            +abs(C(i+1,j+1)-C(i,j+1))+abs(C(i+1,j)-C(i+1,j+1))+abs(C(i+1,j-1)-C(i+1,j))...
            +abs(C(i,j-1)-C(i+1,j-1))+abs(C(i-1,j-1)-C(i,j-1));
            Icn(i,j)=cn/2;
        end
    end
end
figure;
imshow(C);
%标注
[row,col]=find(Icn==1);
hold on, plot(col,row,'gs','MarkerSize',10)
[row,col]=find(Icn==3);
hold on, plot(col,row,'rs','MarkerSize',10)

%去掉边缘细节点 
fun2 = @(x) std2(x);
Is = nlfilter(I,[3 3],fun2);
Ibd=zeros(size(I));
for i=5:M-5
    for j=5:364
        if Is(i,j)~=0
            Ibd(i:i+1,j:j+10)=1;
            break;
        end
    end
end
for i=5:M-5
    for j=5:M-5
        if Is(M-i,N-j)~=0
            Ibd(M-i,N-j-7:N-j)=1;
            break;
        end
    end
end
Icn(Ibd==1)=0;
figure;
imshow(C);
%标注
[row,col]=find(Icn==1);
hold on, plot(col,row,'gs','MarkerSize',10)
[row,col]=find(Icn==3);
hold on, plot(col,row,'rs','MarkerSize',10)

%---------------------

function C = Pruning(A, len)

B = CreateEndpointSE();
X1 = A;
for k = 1:len
    endpoints = false(size(A));
    for m = 1:size(B,1)
        endpoints = endpoints | bwhitmiss(X1, B{m,1}, B{m,2});
    end
    X1(endpoints) = 0;
end

X2 = false(size(A));
for m = 1:size(B,1)
    endpoints = bwhitmiss(X1, B{m,1}, B{m,2});
    X2(endpoints) = 1;
end

se = strel(ones(3,3));
X3 = X2;
for k = 1:len
    X3 = imdilate(X3, se) & A;
end

C = X3 | X1; 
end

%--------------------
function B = CreateEndpointSE()
B{1,1} = [0 0 0; 1 1 0; 0 0 0];
B{1,2} = [0 1 1; 0 0 1; 0 1 1];
for k = 2:4
    B{k,1} = rot90(B{k-1,1});
    B{k,2} = rot90(B{k-1,2});
end
B{5,1} = [1 0 0; 0 1 0; 0 0 0];
B{5,2} = [0 1 1; 1 0 1; 1 1 1];
for k = 6:8
    B{k,1} = rot90(B{k-1,1});
    B{k,2} = rot90(B{k-1,2});
end
end