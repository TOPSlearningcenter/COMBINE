function [sys,x0,str,ts]=LQR_sFunction(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 3
        sys=mdlOutputs(t,x,u);
    case {1, 2, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 5;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0=[];
str=[];
ts=[];

L=3;%车辆轴距，m

% 初始处理化处理计算整条轨迹的曲率车速和理论航向角
load("reference.mat")
% 一阶导数
dydx=diff(y_ref)./diff(x_ref);
% 二阶导数
d2ydx2 = diff(dydx)./diff(x_ref(1:end-1));

dydx=[dydx,dydx(end)];
d2ydx2=[d2ydx2(1),d2ydx2,d2ydx2(end)];

% 求曲率
Wq=zeros(size(t_ref));
for i=1:length(t_ref)
    Wq(i)=(d2ydx2(i))/((1+dydx(i)^2)^(1.5));
end

% 求参考前轮转角
delta_ref = atan(L.*Wq);


% 算参考横摆角
phi_ref = atan(dydx);

% 算参考车速
v_ref=zeros(size(t_ref));

for i=1:length(x_ref)-1
    Delta_dis(i) = norm([x_ref(i+1)-x_ref(i),y_ref(i+1)-y_ref(i)]);
end
v_ref(1:end-1) = Delta_dis./diff(t_ref);
v_ref(end)=v_ref(end-1);

% 参考加速度
ax_ref=zeros(size(t_ref));
ax_ref(1:end-1)=diff(v_ref)./diff(t_ref);
ax_ref(end)=ax_ref(end-1);


save("reference_processed.mat",'t_ref','x_ref',"y_ref","phi_ref","delta_ref","v_ref","ax_ref");

end


function sys=mdlOutputs(t,x,u)

% u(1) psi
% u(2) x_g
% u(3) y_g
% u(4) vx

phi=u(1);
x=u(2);
y=u(3);
v=u(4);


load("reference_processed.mat")


x_ref1 = interp1(t_ref,x_ref,t);
y_ref1 = interp1(t_ref,y_ref,t);
phi_ref1 = interp1(t_ref,phi_ref,t);
v_ref1 = interp1(t_ref,v_ref,t); 

ax_ref1 = interp1(t_ref,ax_ref,t);
delta_ref1 = interp1(t_ref,delta_ref,t);

[ax,delta]= lqr_control(x,y,phi,v,x_ref1,y_ref1,phi_ref1,v_ref1,ax_ref1,delta_ref1);

m=2000; % 整车质量；

h=0.35; %质心高度
a=1.4;
b=1.6;
L=a+b;
g=9.8;

Delta_Fz=ax_ref1*h/L;

Fz_f=m*g*b/L-Delta_Fz;
Fz_r=m*g*a/L+Delta_Fz;


Fx=m*ax;
% 按照垂向载荷分配纵向力
Fx_f = Fx*Fz_r/(m*g);
Fx_r = Fx*Fz_f/(m*g);


sys(1)=delta;
sys(2)=0.5.*Fx_f;
sys(3)=0.5.*Fx_f;
sys(4)=0.5.*Fx_r;
sys(5)=0.5.*Fx_r;

end


function [ax,delta]= lqr_control(x,y,phi,v,x_ref1,y_ref1,phi_ref1,v_ref1,ax_ref1,delta_ref1)
L=3;%车辆轴距，m

A=[0,0,-v_ref1*sin(phi_ref1),cos(phi_ref1);
   0,0,v_ref1*cos(phi_ref1),sin(phi_ref1);
   0,0,0,tan(delta_ref1)/L;
   0,0,0,0];
B=[0,0;
   0,0;
   0,v_ref1/(L*(cos(delta_ref1))^2);
   1,0];

C=[1,0,0,0;
   0,1,0,0;
   0,0,0,1];

Q=diag([100,100,10]);
R=diag([1,10]);

Q1=C'*Q*C;

[P,~,~]=care(A,B,Q1,R);

X=[x-x_ref1;
   y-y_ref1;
   phi-phi_ref1;
   v-v_ref1];

u_out = -R\B'*P*X;

ax = u_out(1)+ax_ref1;
delta = u_out(2)+delta_ref1;

end
