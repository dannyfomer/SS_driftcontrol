clc;clear;close all;
tic;
%% 01.初始参数设置
%-------------------------------------------------------------------------%
%----------------------- 设置问题的求解边界 ------------------------------%
%-------------------------------------------------------------------------%
% 设置时间
t0 = 0;
tmax = 20;
% 设置状态量初值

%x1:X   x2:Y  x3:航向角theta  x4:速度V  x5:侧滑角beta   x6:航向角速度r 
x11_0 =0;x12_0 =5;%x13_0=;      xij_0表示状态变量xi在第j阶段的初值，若某阶段初值被注释，则说明该阶段没有确切初值
x11_f=5;%x12_f=;x13_f=;         xij_f表示状态变量xi在第j阶段的终值
x21_0 =0.5;x23_0 =10;%x22_0 =;
x22_f =10; x23_f =15;%x21_f =;
x30 =0;x3f= pi/2;

% 设置状态量边界条件
x11_Min = 0;x12_Min = 5;x13_Min = 14;
x11_Max = 50;x12_Max = 16;x13_Max = 16;

x21_Min = -1;x22_Min = -1;x23_Min = 10;
x21_Max = 1;x22_Max = 10;x23_Max = 15;

x3Min = -pi/2;
x3Max = pi;
%road约束
c10 = 0;
c1Min = -2;
c1Max = 2;
% 设置控制量边界条件   u1为转向角  u2为后轮驱动力

u1Min  = -pi*30/180;
u1Max  = pi*30/180;
u2Min  = 0;
u2Max  = 2;
%% 02.边界条件设置
%-------------------------------------------------------------------------%
%------------------------ 将求解边界设置于问题中 -------------------------%
%-------------------------------------------------------------------------%
iphase=1;
bounds.phase(iphase).initialtime.lower  = t0; 
bounds.phase(iphase).initialtime.upper  = t0;
bounds.phase(iphase).finaltime.lower    = t0; 
bounds.phase(iphase).finaltime.upper    = tmax;
bounds.phase(iphase).initialstate.lower = [0 0 0 1 0 0]; 
bounds.phase(iphase).initialstate.upper = [0 0 0 1 0 0];
bounds.phase(iphase).state.lower        = [0 -2 x3Min 1 -1 -2]; 
bounds.phase(iphase).state.upper        = [20 2 x3Max 10 1 2];
bounds.phase(iphase).finalstate.lower   = [20 -2 x3Min 1 -1 -2];
bounds.phase(iphase).finalstate.upper   = [20 2 x3Max 10 1 2];
bounds.phase(iphase).control.lower      = [u1Min u2Min]; 
bounds.phase(iphase).control.upper      = [u1Max u2Max];
bounds.phase(iphase).path.lower        = c1Min;
bounds.phase(iphase).path.upper        = c1Max;

guess.phase(iphase).time     = [t0; tmax]; 
guess.phase(iphase).state    = [[10; 20],[-2; 2],[0; x3Max],[1;10],[-1;1],[0;2]];
guess.phase(iphase).control  = [[0; u1Max],[0; u2Max]];

iphase=2;
bounds.phase(iphase).initialtime.lower  = t0; 
bounds.phase(iphase).initialtime.upper  = tmax;
bounds.phase(iphase).finaltime.lower    = t0; 
bounds.phase(iphase).finaltime.upper    = tmax;

bounds.phase(iphase).initialstate.lower = [20 -2 x3Min 1 -1 -2]; 
bounds.phase(iphase).initialstate.upper = [20 2 x3Max 10 1 3];
bounds.phase(iphase).state.lower        = [20 -2 x3Min 1 -1 -2]; 
bounds.phase(iphase).state.upper        = [32 10 x3Max 10 1 2];
bounds.phase(iphase).finalstate.lower   = [28 10 x3Min 1 -1 -2];
bounds.phase(iphase).finalstate.upper   = [32 10 x3Max 10 1 2];

bounds.phase(iphase).control.lower      = [u1Min u2Min]; 
bounds.phase(iphase).control.upper      = [u1Max u2Max];
bounds.phase(iphase).path.lower        = c1Min;
bounds.phase(iphase).path.upper        = c1Max;

guess.phase(iphase).time     = [t0; tmax]; 
guess.phase(iphase).state    = [[20; 32],[-2; 10],[x3Min; x3Max],[1;10],[-1;1],[0;2]];
guess.phase(iphase).control  = [[0; u1Max],[0; u2Max]];

iphase=3;
bounds.phase(iphase).initialtime.lower  = t0; 
bounds.phase(iphase).initialtime.upper  = tmax;
bounds.phase(iphase).finaltime.lower    = t0; 
bounds.phase(iphase).finaltime.upper    = tmax;

bounds.phase(iphase).initialstate.lower = [28 10 x3Min 1 -1 -2]; 
bounds.phase(iphase).initialstate.upper = [32 10 x3Max 10 1 2];
bounds.phase(iphase).state.lower        = [28 10 x3Min 1 -1 -2]; 
bounds.phase(iphase).state.upper        = [32 20 x3Max 10 1 2];
bounds.phase(iphase).finalstate.lower   = [28 20 pi/2 1 -1 -2];
bounds.phase(iphase).finalstate.upper   = [32 20 pi/2 10 1 2];

bounds.phase(iphase).control.lower      = [u1Min u2Min]; 
bounds.phase(iphase).control.upper      = [u1Max u2Max];
bounds.phase(iphase).path.lower        = c1Min;
bounds.phase(iphase).path.upper        = c1Max;

guess.phase(iphase).time     = [t0; tmax]; 
guess.phase(iphase).state    = [[28; 32],[10; 20],[x3Min; x3Max],[1;10],[-1;1],[0;2]];
guess.phase(iphase).control  = [[0; u1Max],[0; u2Max]];
%bounds.phase.integral.lower     = 0; 
%bounds.phase.integral.upper     = 10000;
%-------------------------------------------------------------------------%
%------------- Set up Event Constraints That Link Phases -----------------%
%-------------------------------------------------------------------------%

%% 03.初值猜测
%-------------------------------------------------------------------------%
%------------------------------- 初值猜想 --------------------------------%
%-------------------------------------------------------------------------%
%{
guess.phase.time     = [t0; tmax]; 
guess.phase.state    = [[x10; x1fmax],[x20; x2f],[x30; x3f]];
guess.phase.control  = [[u1Min; u1Max],[u2Min; u2Max]];
guess.phase.path = [c10; c1Max];
%guess.phase.integral = 100;
%}
bounds.eventgroup(1).lower = [zeros(1,6),0];
bounds.eventgroup(1).upper = [zeros(1,6),0];
bounds.eventgroup(2).lower = [zeros(1,6),0];
bounds.eventgroup(2).upper = [zeros(1,6),0];
meshphase(1).colpoints = 4*ones(1,10);
meshphase(1).fraction = 0.1*ones(1,10);
meshphase(2).colpoints = 4*ones(1,10);
meshphase(2).fraction = 0.1*ones(1,10);
meshphase(3).colpoints = 4*ones(1,10);
meshphase(3).fraction = 0.1*ones(1,10);
%% 04.设置GPOPS求解器参数
%-------------------------------------------------------------------------%
%---------------------------- 设置求解器参数 -----------------------------%        
%-------------------------------------------------------------------------%
setup.name = 'Vehicle-Conering';
setup.functions.continuous  = @vsopcContinuous;
setup.functions.endpoint   	= @vsopcEndpoint;
setup.bounds                = bounds;
setup.guess                 = guess;
setup.nlp.solver            = 'snopt';
setup.derivatives.supplier  = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.mesh.method           = 'hp1';
setup.mesh.tolerance        = 0.01;
setup.mesh.maxiteration     = 2;
setup.mesh.colpointsmax     = 4;
setup.mesh.colpointsmin     = 10;
setup.mesh.phase = meshphase;
setup.method = 'RPMintegration';

%% 05.求解
%-------------------------------------------------------------------------%
%----------------------- 使用 GPOPS2 求解最优控制问题 --------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;
toc;

%% 06.画图

figure('Color',[1,1,1]);
iphase=1;
plot(solution.phase(iphase).state(:,1),solution.phase(iphase).state(:,2),'g');hold on;
axis equal;
  x_in0=linspace(0,20,100);
  y_in0=x_in0-x_in0+2;
  x_in1=linspace(20,28,100);
  y_in1=-sqrt(64-(x_in1-20).^2)+10;
  x_in=[x_in0 x_in1];
  y_in=[y_in0 y_in1];
  line([28 28],[10 20],'Color','r');hold on;
  plot(x_in, y_in,'r-'); hold on;
 iphase=2;
plot(solution.phase(iphase).state(:,1),solution.phase(iphase).state(:,2),'g');hold on;
 iphase=3;
plot(solution.phase(iphase).state(:,1),solution.phase(iphase).state(:,2),'g');hold on;
  x_out0=linspace(0,20,100);
  y_out0=x_out0-x_out0-2;
  x_out1=linspace(20,32,100);
  y_out1=-sqrt(144-(x_out1-20).^2)+10;
  x_out=[x_out0 x_out1];
  y_out=[y_out0 y_out1];
  line([32 32],[10 20],'Color','r');hold on;
  plot(x_out, y_out,'r-'); hold on;
xlabel('X(m)'); ylabel('Y(m)');
legend('实际轨迹','道路边缘线');

axis equal;

figure('Color',[1,1,1]);
for iphase=1:3
    plot(solution.phase(iphase).time,solution.phase(iphase).state(:,4),'g');hold on;
    xlabel('t(s)'); ylabel('V(m/s)');
end
figure('Color',[1,1,1]);
for iphase=1:3
    plot(solution.phase(iphase).time,solution.phase(iphase).state(:,5),'g');hold on;
    xlabel('t(s)'); ylabel('(侧滑角\beta（rad)');
end
figure('Color',[1,1,1]);
for iphase=1:3
    plot(solution.phase(iphase).time,solution.phase(iphase).state(:,6),'g');hold on;
    xlabel('t(s)'); ylabel('航向角速度r(rad/s)');
end
figure('Color',[1,1,1]);
for iphase=1:3
    plot(solution.phase(iphase).time,solution.phase(iphase).state(:,3),'g');hold on;
    xlabel('t(s)'); ylabel('航向角\theta(rad)');
end
figure('Color',[1,1,1]);
for iphase=1:3
    plot(solution.phase(iphase).time,solution.phase(iphase).control(:,1),'g');hold on;
    xlabel('t(s)'); ylabel('前轮转向角\delta(rad)');
end
figure('Color',[1,1,1]);
for iphase=1:3
    plot(solution.phase(iphase).time,solution.phase(iphase).control(:,2),'g');hold on;
    xlabel('t(s)'); ylabel('F(N)');
end
figure('Color',[1,1,1]);
for iphase=1:3
    plot(solution.phase(iphase).time,solution.phase(iphase).state(:,2),'g');hold on;
    xlabel('t(s)'); ylabel('Y(m)');
end
gpops_time=[solution.phase(1).time;solution.phase(2).time;solution.phase(3).time];
gpops_speed=[solution.phase(1).state(:,4);solution.phase(2).state(:,4);solution.phase(3).state(:,4)]*3.6;
gpops_steering=[solution.phase(1).control(:,1);solution.phase(2).control(:,1);solution.phase(3).control(:,1)]/pi*180*16.7785235;
gpops_excel=[gpops_time gpops_speed gpops_steering];
%% 函数模块部分

% ----------------------------------------------------------------------- %
% ------------------------- BEGIN: vsopcContinuous.m -------------------- %
% ----------------------------------------------------------------------- %
function phaseout = vsopcContinuous(input)

m = 1416;          % kg
Iz = 1523;         % kg / m^2
lf = 1.016;          % m
lr = 1.562;         % m
BF=14.55;CF=1.847;DF=3696;
BR=14.55;CR=1.847;DR=3696;
EF=0.1;ER=0.1;
iphase=1;
%t  = input.phase.time;
%x1  = input.phase(iphase).state(:,1);
x2  = input.phase(iphase).state(:,2);
x3  = input.phase(iphase).state(:,3);
x4  = input.phase(iphase).state(:,4);
x5  = input.phase(iphase).state(:,5);
x6  = input.phase(iphase).state(:,6);


u1   = input.phase(iphase).control(:,1);
u2   = input.phase(iphase).control(:,2)*1000;


%计算所需中间变量
aF=u1-atan((x4.*sin(x5)+lf*x6)./(x4.*cos(x5)));
aR=-atan((x4.*sin(x5)-lr*x6)./(x4.*cos(x5)));
fFy=2*DF*sin(CF*atan(BF*aF)-EF*(BF*aF-atan(BF*aF)));
fRy=2*DR*sin(CR*atan(BR*aR)-ER*(BR*aR-atan(BR*aR)));

%写微分方程
dx1 = x4.*cos(x3+x5);
dx2 = x4.*sin(x3+x5);
dx3 = x6;
dx4 = (fFy.*sin(x5-u1)+fRy.*sin(x5)+u2.*cos(x5))/m;
dx5 = -x6+(fFy.*cos(x5-u1)+fRy.*cos(x5)-u2.*sin(x5))./(m*x4);
dx6 = (lf*fFy.*cos(u1)-fRy*lr)/Iz;

%编写道路约束

phaseout(iphase).dynamics = [dx1,dx2,dx3,dx4,dx5,dx6];
phaseout(iphase).path=x2;

iphase=2;
%t  = input.phase.time;
x1  = input.phase(iphase).state(:,1);
x2  = input.phase(iphase).state(:,2);
x3  = input.phase(iphase).state(:,3);
x4  = input.phase(iphase).state(:,4);
x5  = input.phase(iphase).state(:,5);
x6  = input.phase(iphase).state(:,6);


u1   = input.phase(iphase).control(:,1);
u2   = input.phase(iphase).control(:,2)*1000;


%计算所需中间变量
aF=u1-atan((x4.*sin(x5)+lf*x6)./(x4.*cos(x5)));
aR=-atan((x4.*sin(x5)-lr*x6)./(x4.*cos(x5)));
fFy=2*DF*sin(CF*atan(BF*aF)-EF*(BF*aF-atan(BF*aF)));
fRy=2*DR*sin(CR*atan(BR*aR)-ER*(BR*aR-atan(BR*aR)));

%写微分方程
dx1 = x4.*cos(x3+x5);
dx2 = x4.*sin(x3+x5);
dx3 = x6;
dx4 = (fFy.*sin(x5-u1)+fRy.*sin(x5)+u2.*cos(x5))/m;
dx5 = -x6+(fFy.*cos(x5-u1)+fRy.*cos(x5)-u2.*sin(x5))./(m*x4);
dx6 = (lf*fFy.*cos(u1)-fRy*lr)/Iz;

%编写道路约束

phaseout(iphase).dynamics = [dx1,dx2,dx3,dx4,dx5,dx6];
phaseout(iphase).path=sqrt((x2-10).^2+(x1-20).^2)-10;

iphase=3;
%t  = input.phase.time;
x1  = input.phase(iphase).state(:,1);
%x2  = input.phase(iphase).state(:,2);
x3  = input.phase(iphase).state(:,3);
x4  = input.phase(iphase).state(:,4);
x5  = input.phase(iphase).state(:,5);
x6  = input.phase(iphase).state(:,6);


u1   = input.phase(iphase).control(:,1);
u2   = input.phase(iphase).control(:,2)*1000;


%计算所需中间变量
aF=u1-atan((x4.*sin(x5)+lf*x6)./(x4.*cos(x5)));
aR=-atan((x4.*sin(x5)-lr*x6)./(x4.*cos(x5)));
fFy=2*DF*sin(CF*atan(BF*aF)-EF*(BF*aF-atan(BF*aF)));
fRy=2*DR*sin(CR*atan(BR*aR)-ER*(BR*aR-atan(BR*aR)));

%写微分方程
dx1 = x4.*cos(x3+x5);
dx2 = x4.*sin(x3+x5);
dx3 = x6;
dx4 = (fFy.*sin(x5-u1)+fRy.*sin(x5)+u2.*cos(x5))/m;
dx5 = -x6+(fFy.*cos(x5-u1)+fRy.*cos(x5)-u2.*sin(x5))./(m*x4);
dx6 = (lf*fFy.*cos(u1)-fRy*lr)/Iz;

%编写道路约束

phaseout(iphase).dynamics = [dx1,dx2,dx3,dx4,dx5,dx6];
phaseout(iphase).path=x1-30;
%phaseout.integrand = 0.5*u.^2;
end
% ----------------------------------------------------------------------- %
% -------------------------- END: vsopcContinuous.m --------------------- %
% ----------------------------------------------------------------------- %

% ----------------------------------------------------------------------- %
% -------------------------- BEGIN: vsopcEndpoint.m --------------------- %
% ----------------------------------------------------------------------- %
function output = vsopcEndpoint(input)

% Variables at Start and Terminus of Phase 1

tf1 = input.phase(1).finaltime;

xf1 = input.phase(1).finalstate;
% Variables at Start and Terminus of Phase 2
t02 = input.phase(2).initialtime;
tf2 = input.phase(2).finaltime;
x02 = input.phase(2).initialstate;
xf2 = input.phase(2).finalstate;
% Variables at Start and Terminus of Phase 3
t03 = input.phase(3).initialtime;
x03 = input.phase(3).initialstate;

% Event Group 1: Linkage Constraints Between Phases 1 and 2
output.eventgroup(1).event = [x02(1:6)-xf1(1:6), t02-tf1];
% Event Group 2: Linkage Constraints Between Phases 2 and 3
output.eventgroup(2).event = [x03(1:6)-xf2(1:6), t03-tf2];

J  = input.phase(3).finaltime;

output.objective = J;
end
% ----------------------------------------------------------------------- %
% --------------------------- END: vsopcEndpoint.m ---------------------- %
% ----------------------------------------------------------------------- %
