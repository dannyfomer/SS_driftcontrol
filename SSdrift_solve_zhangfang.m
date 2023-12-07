function [fRx,beta,r,fFy,fRy] = SSdrift_solve_zhangfang(x1,u1)
%固定x1,u1求x2,x3,u2,即固定纵向速度，和前轮转向角，去求侧滑角、航向角速度、后轮驱动力
%    SSdrift_solve_zhangfang(1.2,10*pi/180)   

%  车辆参数
m = 1.95;          % kg
Iz = 0.24;         % kg / m^2
lf = 0.125;          % m
lr = 0.125;         % m

BF=7.4;CF=1.25;DF=2.1;
BR=7.4;CR=1.25;DR=2.1;


%定义状态量[Ux,beta,r]=[x1,x2,x3]
syms u2 x2 x3;                                     

%计算轮胎侧滑角和侧向力
aF=atan(x2+(lf*x3)/x1)-u1;
aR=atan(x2-(lr*x3)/x1);

fFy=-1*DF*sin(CF*atan(BF*aF));
fRy=-1*DR*sin(CR*atan(BR*aR));

%稳态转弯方程组

eqns1= (fFy+fRy)/(m*x1)-x3 ==0;
eqns2= (fFy*lf-fRy*lr)/Iz ==0;
eqns3= (u2-fFy*sin(u1))/m+x1*x2*x3== 0;
eqns = [eqns1,eqns2,eqns3];

vars = [u2 x2 x3];                                 %变量
[solu2, solx2,solx3] = vpasolve(eqns, vars);             %进行求解
 %精确到小数点后四位
u2=vpa(solu2,4);                           
x2=vpa(solx2,4);
x3=vpa(solx3,4);

fRx = u2;
beta = x2;
r = x3;



aF=atan(beta+(lf*r)/x1)-u1;
aR=atan(beta-(lr*r)/x1);
fFy =-1*DF*sin(CF*atan(BF*aF));
fRy =-1*DR*sin(CR*atan(BR*aR));
end




