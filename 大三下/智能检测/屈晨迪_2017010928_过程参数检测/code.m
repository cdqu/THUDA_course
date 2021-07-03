% Ϊ��������ظ����룬�����򲿷ֻ��ڽ�չʾ205ns�µ����
% ������˳�������źŲ���
% ��ʼ������
f=1*10^6;
a=1*10^12;
tau=5*10^-6;
fs=1*10^9;
t=0:1/fs:2*tau;
s=sin(2*pi*f*t).*exp(-a*(t-tau).^2/2);
figure(1);
plot(t,s);
xlabel('t/s');
title('������˳�������źŲ���');
% ���������������źŲ���
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
title('���������������źŲ���');
legend('˳��','dt=205ns','dt=210ns','dt=215ns');
% ��ط���ʱ�� ��205nsΪ��
[r,b]=xcorr(s,s_1);
figure(3);
plot(b/fs,r);
xlabel('t/s');
title('˳��������Է���');
[~,x]=max(r);
delt=(x-length(t))/fs;  % ����ʱ�delt
fprintf('��ط� 205ns��\n');
fprintf('�������� ����ʱ�%s\n',delt)
% ������ʱ����Ƶ�Ӱ��
snr=20;  % ����ǿ��
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
title('�����=20db');
legend('˳��','dt=205ns','dt=210ns','dt=215ns');
[r,b]=xcorr(sn,s_n1);
figure;
plot(b/fs,r);
xlabel('t/s');
title('˳��������Է���');
[~,x]=max(r);
delt_noise=(x-length(t))/fs;
fprintf('������ ����ʱ�%s\n',delt_noise);

% % ����Ƶ�ʽ���
fs_low=50*10^6;
t_l=0:1/fs_low:2*tau;
s=sin(2*pi*f*t_l).*exp(-a*(t_l-tau).^2/2);
figure;
plot(t_l,s);
xlabel('t/s');
title('���Ͳ���Ƶ���µ�˳�������ź�');
s_1=sin(2*pi*f*(t_l-dt1)).*exp(-a*(t_l-dt1-tau).^2/2);
s_2=sin(2*pi*f*(t_l-dt2)).*exp(-a*(t_l-dt2-tau).^2/2);
s_3=sin(2*pi*f*(t_l-dt3)).*exp(-a*(t_l-dt3-tau).^2/2);
% ������
snr=60;
sn=awgn(s,snr); % ˳��+����
s_n1=awgn(s_1,snr); % ����(205ns)+����
% ��ط�
[r,b]=xcorr(sn,s_n1);
% plot(b/fs_low,r);
% xlabel('t/s');
% title('˳��������Է���');
[max_y,x]=max(r);
n=(x-length(t_l))/fs_low;
fprintf('�Ͳ���Ƶ�ʣ�\n');
fprintf('������ ����ʱ�%s\n',n);
% У�� ���κ�����ֵ
PX=[n-1/fs_low n n+1/fs_low];
PY=[r(x-1) r(x) r(x+1)];
k=polyfit(PX,PY,2);
newt=-tau/10:1/fs:tau/10;
fx=polyval(k,newt);
% plot(PX,PY,'r*',newt,fx,'b-');
[max_fx,i]=max(fx);
nt=(i-1)/fs-tau/10;  % У����Ĺ���ʱ�nt
fprintf('������ ��ֵУ����ʱ�%s\n',nt);

% % ѡ����L1����
% �˴�ʹ�ò����������źţ���Ҫ���Դ������źţ���104��s��Ϊsn��107��s_1��Ϊs_n1��
% �ٵ���75�������ֵ����
mx=-tau:1/fs_low:tau;
L=zeros(1,length(mx));
zer=zeros(1,250);
s_db=[zer s zer];   % �˴�s��Ϊsn�����Դ������źţ�
for m=0:1:length(mx)-1
    sm=s_db(1,[1+m:1:501+m]);
    minus=s_1-sm;  % �˴�s_1��Ϊsn_1�����Դ������źţ�
    L(1,m+1)=norm(minus,1);
end
% plot(mx,L);
% title('L1������');
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
[ldt,min_y]=node(X1,Y1,X2,Y2);  % ��ֵУ����ʱ�ldt
fprintf('L1���� 205ns��\n');
fprintf('�������� ��ֵУ����ʱ�%s\n',ldt);


% �ĵ�����ֱ���󽻵�
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
