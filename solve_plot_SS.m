clc;clear all;
 %% 参数定义
%  车辆参数
m = 1.95;          % kg
Iz = 0.24;         % kg / m^2
lf = 0.125;          % m
lr = 0.125;         % m

BF=7.4;CF=1.25;
BR=7.4;CR=1.25;
u=0.23;Fz=10;
%生成data 100 X 3的矩阵，用于储存数据
%定义固定的纵向速度Ux 即x1
Ux=1.2;
%定义转向角取值数，即行数total_times (total_times越大，点越密)， 让转向角变化total_times次
total_times=40;
delta=linspace(-20,20,total_times);%单位deg

%通过vapsolve求数值解可能丢失解，与初值取值有关.
%因此设定随机取初值的次数rand_num，次数越高越不容易丢失解，次数过低可能出现无解的情况而报错,影响计算时间
rand_num=40;

%定义三维data为n x 5的矩阵，5列依次对应状态量[fRx,beta,r,fFy,fRy].
%且data有三页，即每个delta可能对应最多三个解 其中第一页存放轮胎力未饱和的解，后两页存放轮胎力饱和时的解

data=zeros(total_times,5,3);
 %% 

for i=1:total_times
    %% 获取轮胎力未饱和时的解
    [data(i,1,1),data(i,2,1),data(i,3,1),data(i,4,1),data(i,5,1)]=SSdrift_solve_zhangfang(Ux,delta(i)*pi/180);

    %% 获取轮胎力饱和时的解
    %初始化暂时存储解的矩阵data_fmax
    data_fmax=zeros(rand_num,5);
    for j=1:rand_num
        %获取解
        [fRx,beta,r]=SSdrift_solve_zhangfang_fymax_Random(Ux,delta(i)*pi/180);
        %判断此时是否无解，避免出现“无法执行赋值，因为左侧的大小为 1×1，右侧的大小为 0×1”的报错
        if fRx~=0 & beta~0 & r~=0;
            data_fmax(j,1)=fRx;
            data_fmax(j,2)=beta;
            data_fmax(j,3)=r;
        end
    end

    %data_fmax进行格式处理
    %去掉data_fmax中的全零行
    data_fmax (all(data_fmax == 0, 2),:) = [];
    %去掉data_fmax中的重复行，最后变为2 x 3的矩阵，代表[fRx,beta,r]各自对应的两个解
    data_fmax= unique(data_fmax,'rows');
    

    %缺解时补齐矩阵为 2 x 5
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
    %将侧滑角大于零的放到第一行
    if data_fmax(1,2) <0
        data_fmax([1,2],:) =  data_fmax([2,1],:);
    end
        %计算对应的侧向力
        for m=1:2
            aF=atan(data_fmax(m,2)+(lf*data_fmax(m,3))/Ux)-delta(i)*pi/180;
            aR=atan(data_fmax(m,2)-(lr*data_fmax(m,3))/Ux);
            fFy = -1*u*Fz*sin(CF*atan(BF*aF));
            fRy = -sqrt((u*Fz)^2-data_fmax(m,1)^2)*sign(aR);
            data_fmax(m,4)=fFy;
            data_fmax(m,5)=fRy;
        end
        %% 循环五次  将暂存data_fmax中的解放入最终data矩阵中
        for n=1:5 
            data(i,n,2)=data_fmax(1,n);
            data(i,n,3)=data_fmax(2,n);
        end

        

        disp(['progress :',num2str(i),'/',num2str(total_times)])

end

%% 画图
figure('color',[1 1 1])
subplot(2,2,1)
plot(delta,data(:,1,1),'o');hold on
plot(delta,data(:,1,2),'r*');hold on
plot(delta,data(:,1,3),'g*');hold on
xlabel('前轮转向角(deg)'); ylabel('后轮驱动力fRx(N)');
subplot(2,2,2)
plot(delta,data(:,2,1)*180/pi,'o');hold on
plot(delta,data(:,2,2)*180/pi,'r*');hold on
plot(delta,data(:,2,3)*180/pi,'g*');hold on

xlabel('前轮转向角(deg)'); ylabel('侧滑角beta(deg)');
subplot(2,2,3)
plot(delta,data(:,3,1),'o');hold on
plot(delta,data(:,3,2),'r*');hold on
plot(delta,data(:,3,3),'g*');hold on
xlabel('前轮转向角(deg)'); ylabel('航向角速度r(rad/s)');
figure('color',[1 1 1])
subplot(2,1,1)
plot(delta,data(:,4,1),'o');hold on
plot(delta,data(:,4,2),'r*');hold on
plot(delta,data(:,4,3),'g*');hold on
xlabel('前轮转向角(deg)'); ylabel('前轮侧向力fFy(N)');
subplot(2,1,2)
plot(delta,data(:,5,1),'o');hold on
plot(delta,data(:,5,2),'r*');hold on
plot(delta,data(:,5,3),'g*');hold on
xlabel('前轮转向角(deg)'); ylabel('后轮侧向力fRy(N)');
