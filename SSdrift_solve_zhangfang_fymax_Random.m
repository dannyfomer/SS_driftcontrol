function [fRx,beta,r] = SSdrift_solve_zhangfang_fymax_Random(x1,u1)
%�̶�x1,u1��x2,x3,u2,���̶������ٶȣ���ǰ��ת��ǣ�ȥ��໬�ǡ�������ٶȡ�����������
%    SSdrift_solve_zhangfang_fymax_Random(1.2,10*pi/180)   

%  ��������
m = 1.95;          % kg
Iz = 0.24;         % kg / m^2
lf = 0.125;          % m
lr = 0.125;         % m

BF=7.4;CF=1.25;
BR=7.4;CR=1.25;
u=0.23;Fz=10;
%����״̬��[Ux,beta,r]=[x1,x2,x3]
syms u2 x2 x3;                                     

%������̥�໬�ǺͲ�����
aF=atan(x2+(lf*x3)/x1)-u1;
aR=atan(x2-(lr*x3)/x1);

fFy=-1*u*Fz*sin(CF*atan(BF*aF));
fRy=-sqrt((u*Fz)^2-u2^2)*sign(aR);

%��̬ת�䷽����

eqns1= (fFy+fRy)/(m*x1)-x3 ==0;
eqns2= (fFy*lf-fRy*lr)/Iz ==0;
eqns3= (u2-fFy*sin(u1))/m+x1*x2*x3== 0;
eqns = [eqns1,eqns2,eqns3];

vars = [u2 x2 x3];                                 %����
%[solu2, solx2,solx3] = vpasolve(eqns, vars,'Random',true);
%�����ó�ֵ������Ӷ�������Զ������һ����
[solu2, solx2,solx3] = vpasolve(eqns, vars,'Random',true);             %�������,'Random',true
 %��ȷ��С�������λ 
u2=vpa(solu2,4);                           
x2=vpa(solx2,4);
x3=vpa(solx3,4);

fRx = u2;
beta = x2;
r = x3;
%{
aF=atan(beta+(lf*r)/x1)-u1;
aR=atan(beta-(lr*r)/x1);
fFy = -1*u*Fz*sin(CF*atan(BF*aF));
fRy = -sqrt((u*Fz)^2-fRx^2)*sign(aR);
%}
end



