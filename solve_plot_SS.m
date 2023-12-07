clc;clear all;
 %% ��������
%  ��������
m = 1.95;          % kg
Iz = 0.24;         % kg / m^2
lf = 0.125;          % m
lr = 0.125;         % m

BF=7.4;CF=1.25;
BR=7.4;CR=1.25;
u=0.23;Fz=10;
%����data 100 X 3�ľ������ڴ�������
%����̶��������ٶ�Ux ��x1
Ux=1.2;
%����ת���ȡֵ����������total_times (total_timesԽ�󣬵�Խ��)�� ��ת��Ǳ仯total_times��
total_times=40;
delta=linspace(-20,20,total_times);%��λdeg

%ͨ��vapsolve����ֵ����ܶ�ʧ�⣬���ֵȡֵ�й�.
%����趨���ȡ��ֵ�Ĵ���rand_num������Խ��Խ�����׶�ʧ�⣬�������Ϳ��ܳ����޽�����������,Ӱ�����ʱ��
rand_num=40;

%������άdataΪn x 5�ľ���5�����ζ�Ӧ״̬��[fRx,beta,r,fFy,fRy].
%��data����ҳ����ÿ��delta���ܶ�Ӧ��������� ���е�һҳ�����̥��δ���͵Ľ⣬����ҳ�����̥������ʱ�Ľ�

data=zeros(total_times,5,3);
 %% 

for i=1:total_times
    %% ��ȡ��̥��δ����ʱ�Ľ�
    [data(i,1,1),data(i,2,1),data(i,3,1),data(i,4,1),data(i,5,1)]=SSdrift_solve_zhangfang(Ux,delta(i)*pi/180);

    %% ��ȡ��̥������ʱ�Ľ�
    %��ʼ����ʱ�洢��ľ���data_fmax
    data_fmax=zeros(rand_num,5);
    for j=1:rand_num
        %��ȡ��
        [fRx,beta,r]=SSdrift_solve_zhangfang_fymax_Random(Ux,delta(i)*pi/180);
        %�жϴ�ʱ�Ƿ��޽⣬������֡��޷�ִ�и�ֵ����Ϊ���Ĵ�СΪ 1��1���Ҳ�Ĵ�СΪ 0��1���ı���
        if fRx~=0 & beta~0 & r~=0;
            data_fmax(j,1)=fRx;
            data_fmax(j,2)=beta;
            data_fmax(j,3)=r;
        end
    end

    %data_fmax���и�ʽ����
    %ȥ��data_fmax�е�ȫ����
    data_fmax (all(data_fmax == 0, 2),:) = [];
    %ȥ��data_fmax�е��ظ��У�����Ϊ2 x 3�ľ��󣬴���[fRx,beta,r]���Զ�Ӧ��������
    data_fmax= unique(data_fmax,'rows');
    

    %ȱ��ʱ�������Ϊ 2 x 5
    if i>2
        [row,col]=size(data_fmax);
        zero_row1=[data(i-1,1,2) data(i-1,2,2) data(i-1,3,2) data(i-1,4,2) data(i-1,5,2)];
        zero_row2=[data(i-1,1,3) data(i-1,2,3) data(i-1,3,3) data(i-1,4,3) data(i-1,5,3)];
        if row==1
            data_fmax=[data_fmax;zero_row2];
        elseif row==0
            data_fmax=[zero_row1;zero_row2];
        end
    end
    %���໬�Ǵ�����ķŵ���һ��
    if data_fmax(1,2) <0
        data_fmax([1,2],:) =  data_fmax([2,1],:);
    end
        %�����Ӧ�Ĳ�����
        for m=1:2
            aF=atan(data_fmax(m,2)+(lf*data_fmax(m,3))/Ux)-delta(i)*pi/180;
            aR=atan(data_fmax(m,2)-(lr*data_fmax(m,3))/Ux);
            fFy = -1*u*Fz*sin(CF*atan(BF*aF));
            fRy = -sqrt((u*Fz)^2-data_fmax(m,1)^2)*sign(aR);
            data_fmax(m,4)=fFy;
            data_fmax(m,5)=fRy;
        end
        %% ѭ�����  ���ݴ�data_fmax�еĽ��������data������
        for n=1:5 
            data(i,n,2)=data_fmax(1,n);
            data(i,n,3)=data_fmax(2,n);
        end

        

        disp(['progress :',num2str(i),'/',num2str(total_times)])

end

%% ��ͼ
figure('color',[1 1 1])
subplot(2,2,1)
plot(delta,data(:,1,1),'o');hold on
plot(delta,data(:,1,2),'r*');hold on
plot(delta,data(:,1,3),'g*');hold on
xlabel('ǰ��ת���(deg)'); ylabel('����������fRx(N)');
subplot(2,2,2)
plot(delta,data(:,2,1)*180/pi,'o');hold on
plot(delta,data(:,2,2)*180/pi,'r*');hold on
plot(delta,data(:,2,3)*180/pi,'g*');hold on

xlabel('ǰ��ת���(deg)'); ylabel('�໬��beta(deg)');
subplot(2,2,3)
plot(delta,data(:,3,1),'o');hold on
plot(delta,data(:,3,2),'r*');hold on
plot(delta,data(:,3,3),'g*');hold on
xlabel('ǰ��ת���(deg)'); ylabel('������ٶ�r(rad/s)');
figure('color',[1 1 1])
subplot(2,1,1)
plot(delta,data(:,4,1),'o');hold on
plot(delta,data(:,4,2),'r*');hold on
plot(delta,data(:,4,3),'g*');hold on
xlabel('ǰ��ת���(deg)'); ylabel('ǰ�ֲ�����fFy(N)');
subplot(2,1,2)
plot(delta,data(:,5,1),'o');hold on
plot(delta,data(:,5,2),'r*');hold on
plot(delta,data(:,5,3),'g*');hold on
xlabel('ǰ��ת���(deg)'); ylabel('���ֲ�����fRy(N)');
