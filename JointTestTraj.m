% CRP14RH10机械臂逆动力学数值仿真模型
% 输入：6关节角度、角速度、角加速度
% 输出：关节驱动力矩

function [sys,x0,str,ts] = JointTestTraj(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % 调用初始化子函数
  case 1,
    sys=[];
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %计算输出子函数
  case 4,
    sys=[];   %计算下一仿真时刻子函数
  case 9,
    sys=[];    %终止仿真子函数
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %初始化子函数

sizes = simsizes;

sizes.NumContStates  = 0;  %连续状态变量个数
sizes.NumDiscStates  = 0;  %离散状态变量个数
sizes.NumOutputs     = 1;  %输出变量个数
sizes.NumInputs      = 0;   %输入变量个数
sizes.DirFeedthrough = 0;   %输入信号是否在输出子函数中出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';
global sumv
sumv = 0;

function sys=mdlOutputs(t,x,u)   %计算输出子函数
global sumv
v0 = 0;
v1 = sqrt(0.2^2+0.2^2+0.2^2)/(16*10/7);  %设置第一个加速度拐点速度v1

dt = 10/7;
t1 = dt;
t2 = 2*dt;
t3 = 3*dt;
t4 = 4*dt;
t5 = 5*dt;
t6 = 6*dt;
t7 = 7*dt;
J = 2*v1/(t1^2);    %加加速度
amax = J*t1;   %加速度最大值
v2 = v1 + amax*(t2 - t1);   
vmax = v1 + v2;

v = [];
dv = [];

if t < t1
    v = 0.5*J*t^2;
    dv = J*t;
end
if t < t2 & t >= t1
   v = 0.5*J*t1^2 + amax*(t - t1);
   dv = amax;
end
if t >= t2 & t < t3
   v = vmax - 0.5*J*(t3 - t)^2;
   dv = -J*(t - t3);
end
if t >= t3 & t < t4
  v = vmax;
  dv = 0;
end
if t >= t4 & t < t5
   v = vmax - 0.5*J*(t - t4)^2;
   dv = -J*(t - t4);
end
 if t >= t5 & t < t6
    v = vmax - 0.5*J*(t5 - t4)^2 - amax*(t - t5);
    dv = -amax;
 end
 if t >= t6 & t <= t7
     v = 0.5*J*(t7 - t)^2;
     dv = J*(t - t7);
 end
 if t > t7
     v = 0;
     dv = 0;
 end
 
 sumv = sumv + v*0.001;


sys(1) = sumv;