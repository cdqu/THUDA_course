% 为避免过多重复代码，本程序部分环节仅展示205ns下的求解
% 超声波顺流传播信号采样
% 初始化参数
f=1*10^6;
a=1*10^12;
tau=5*10^-6;
fs=1*10^9;
t=0:1/fs:2*tau;
s=sin(2*pi*f*t).*exp(-a*(t-tau).^2/2);
figure(1);
plot(t,s);
xlabel('t/s');
title('超声波顺流传播信号采样');
% 超声波逆流传播信号采样
dt1=205*10^-9;
dt2=210*10^-9;
dt3=215*10^-9;
s_1=sin(2*pi*f*(t-dt1)).*exp(-a*(t-dt1-tau).^2/2);
s_2=sin(2*pi*f*(t-dt2)).*exp(-a*(t-dt2-tau).^2/2);
s_3=sin(2*pi*f*(t-dt3)).*exp(-a*(t-dt3-tau).^2/2);
figure(2);
plot(t,s);
hold on
plot(t,s_1);
plot(t,s_2);
plot(t,s_3);
xlabel('t/s');
title('超声波逆流传播信号采样');
legend('顺流','dt=205ns','dt=210ns','dt=215ns');
% 相关法求时差 以205ns为例
[r,b]=xcorr(s,s_1);
figure(3);
plot(b/fs,r);
xlabel('t/s');
title('顺逆流相关性分析');
[~,x]=max(r);
delt=(x-length(t))/fs;  % 估计时差：delt
fprintf('相关法 205ns：\n');
fprintf('不加噪声 估计时差：%s\n',delt)
% 噪声对时差估计的影响
snr=20;  % 噪声强度
sn=awgn(s,snr);
s_n1=awgn(s_1,snr);
s_n2=awgn(s_2,snr);
s_n3=awgn(s_3,snr);
figure;
plot(t,sn);
hold on  
plot(t,s_n1);
plot(t,s_n2);
plot(t,s_n3);
xlabel('t/s');
title('信噪比=20db');
legend('顺流','dt=205ns','dt=210ns','dt=215ns');
[r,b]=xcorr(sn,s_n1);
figure;
plot(b/fs,r);
xlabel('t/s');
title('顺逆流相关性分析');
[~,x]=max(r);
delt_noise=(x-length(t))/fs;
fprintf('加噪声 估计时差：%s\n',delt_noise);

% % 采样频率降低
fs_low=50*10^6;
t_l=0:1/fs_low:2*tau;
s=sin(2*pi*f*t_l).*exp(-a*(t_l-tau).^2/2);
figure;
plot(t_l,s);
xlabel('t/s');
title('降低采样频率下的顺流传播信号');
s_1=sin(2*pi*f*(t_l-dt1)).*exp(-a*(t_l-dt1-tau).^2/2);
s_2=sin(2*pi*f*(t_l-dt2)).*exp(-a*(t_l-dt2-tau).^2/2);
s_3=sin(2*pi*f*(t_l-dt3)).*exp(-a*(t_l-dt3-tau).^2/2);
% 带噪声
snr=60;
sn=awgn(s,snr); % 顺流+噪声
s_n1=awgn(s_1,snr); % 逆流(205ns)+噪声
% 相关法
[r,b]=xcorr(sn,s_n1);
% plot(b/fs_low,r);
% xlabel('t/s');
% title('顺逆流相关性分析');
[max_y,x]=max(r);
n=(x-length(t_l))/fs_low;
fprintf('低采样频率：\n');
fprintf('加噪声 估计时差：%s\n',n);
% 校正 二次函数插值
PX=[n-1/fs_low n n+1/fs_low];
PY=[r(x-1) r(x) r(x+1)];
k=polyfit(PX,PY,2);
newt=-tau/10:1/fs:tau/10;
fx=polyval(k,newt);
% plot(PX,PY,'r*',newt,fx,'b-');
[max_fx,i]=max(fx);
nt=(i-1)/fs-tau/10;  % 校正后的估计时差：nt
fprintf('加噪声 插值校正后时差：%s\n',nt);

% % 选做：L1范数
% 此处使用不加噪声的信号，若要测试带噪声信号，将104行s改为sn，107行s_1改为s_n1，
% 再调整75行信噪比值即可
mx=-tau:1/fs_low:tau;
L=zeros(1,length(mx));
zer=zeros(1,250);
s_db=[zer s zer];   % 此处s改为sn（测试带噪声信号）
for m=0:1:length(mx)-1
    sm=s_db(1,[1+m:1:501+m]);
    minus=s_1-sm;  % 此处s_1改为sn_1（测试带噪声信号）
    L(1,m+1)=norm(minus,1);
end
% plot(mx,L);
% title('L1范数法');
[~,i]=min(L);
if L(i-1)<=L(i+1)
    X1=[mx(i-2),L(i-2)];
    Y1=[mx(i-1),L(i-1)];
    X2=[mx(i),L(i)];
    Y2=[mx(i+1),L(i+1)];
else
    X1=[mx(i-1),L(i-1)];
    Y1=[mx(i),L(i)];
    X2=[mx(i+1),L(i+1)];
    Y2=[mx(i+2),L(i+2)];
end
[ldt,min_y]=node(X1,Y1,X2,Y2);  % 插值校正后时差：ldt
fprintf('L1范数 205ns：\n');
fprintf('不加噪声 插值校正后时差：%s\n',ldt);


% 四点两条直线求交点
function [X,Y]= node( X1,Y1,X2,Y2 )
if X1(1)==Y1(1)
    X=X1(1);
    k2=(Y2(2)-X2(2))/(Y2(1)-X2(1));
    b2=X2(2)-k2*X2(1); 
    Y=k2*X+b2;
end
if X2(1)==Y2(1)
    X=X2(1);
    k1=(Y1(2)-X1(2))/(Y1(1)-X1(1));
    b1=X1(2)-k1*X1(1);
    Y=k1*X+b1;
end
if X1(1)~=Y1(1) & X2(1)~=Y2(1)
    k1=(Y1(2)-X1(2))/(Y1(1)-X1(1));
    k2=(Y2(2)-X2(2))/(Y2(1)-X2(1));
    b1=X1(2)-k1*X1(1);
    b2=X2(2)-k2*X2(1);
    if k1==k2
       X=[];
       Y=[];
    else
       X=(b2-b1)/(k1-k2);
       Y=k1*X+b1;
    end
end
end
