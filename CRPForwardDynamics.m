% CRP14RH10机械臂逆动力学数值仿真模型
% 输入：6关节角度、角速度、角加速度
% 输出：关节驱动力矩

function [sys,x0,str,ts] = CRPForwardDynamics(t, x, u, flag)
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
sizes.NumOutputs     = 18;  %输出变量个数
sizes.NumInputs      = 6;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出子函数中出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

global thetalist
thetalist = [0; 0; 0; 0; 0; 0];
global dthetalist
dthetalist = [0; 0; 0; 0; 0; 0];
global ddthetalist
ddthetalist = [0; 0; 0; 0; 0; 0];

function sys=mdlOutputs(t,x,u)   %计算输出子函数
global thetalist
global dthetalist
global ddthetalist
% 仿真步长
dt = 0.001;
% 驱动力矩
taulist = u( 1: 6 );

Ftip = [0; 0; 0; 0; 0; 0];

g = [0; 0; -9.80665];

M01 = [0.8722    0.1604   -0.4622   0.0467;
           -0.0660    0.9747    0.2136    -0.0104;
            0.4848   -0.1558    0.8607  0.1100;
            0   0   0   1];
        
M12 = [0.8634   -0.4835   -0.1441   0.3482;
    0.2543    0.1703    0.9520  -0.1597;
   -0.4358   -0.8586    0.2700  0.3146;
   0    0   0   1];

M23 = [0.6896    0.7216    0.0606   0.1586;
   -0.7223    0.6795    0.1283  -0.4589;
    0.0514   -0.1322    0.9899  0.1020;
    0   0   0   1];

M34 = [0.7880   -0.1684    0.5922   0.2721;
   -0.5969    0.0268    0.8019  0.1454;
   -0.1509   -0.9854   -0.0794  0.0036;
   0    0   0   1];

M45 = [0.1251   -0.7912    0.5986   -0.0506;
   -0.0749   -0.6091   -0.7895  -0.0222;
    0.9893    0.0539   -0.1354  0.2245;
    0   0   0   1];

M56 = [0.9996   -0.0152   -0.0222   -0.0030;
    0.0076   -0.6318    0.7751  0.0851;
   -0.0258   -0.7750   -0.6314  -0.0711;
   0    0   0   1];

M6e = [1         0         0   0;
         0    1         0    0;
         0         0    1  0;
         0  0   0   1];
     
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M6e);

G1 = [0.2526         0         0         0         0         0;
         0    0.4529         0         0         0         0;
         0         0    0.5259         0         0         0;
         0         0         0    17.9513         0         0;
         0         0         0         0    17.9513         0;
         0         0         0         0         0    17.9513];
     
G2 = [0.2044         0         0         0         0         0;
         0    0.0110         0         0         0         0;
         0         0    0.2110         0         0         0;
         0         0         0    3.5484         0         0;
         0         0         0         0    3.5484         0;
         0         0         0         0         0    3.5484];
     
G3 = [0.0357         0         0         0         0         0;
         0    0.1436         0         0         0         0;
         0         0    0.1365         0         0         0;
         0         0         0    7.3201         0         0;
         0         0         0         0    7.3201         0;
         0         0         0         0         0    7.3201];
     
G4 = [0.0709         0   -0         0         0         0;
         0    0.0640         0         0         0         0;
   -0         0    0.0168         0         0         0;
         0         0         0    3.8682         0   -0;
         0         0         0         0    3.8682         0;
         0         0         0   -0         0    3.8682];

G5 = [0.0022    0   -0         0         0         0;
    0    0.0018    0         0         0         0;
   -0    0    0.0017         0         0         0;
         0         0         0    0.7287    0   -0;
         0         0         0    0    0.7287    0;
         0         0         0   -0    0    0.7287];
     
G6 = [0.0017         0   -0         0         0         0;
         0    0.0017         0         0         0         0;
   0         0    0.0017         0         0         0;
         0         0         0    1         0   0;
         0         0         0         0    1         0;
         0         0         0   0         0    1];
     
Glist = cat(3, G1, G2, G3, G4, G5, G6);
     
S1 = [0
     0
     1
     0
     0
     0];

 S2 = [0
    1
         0
   -0.2850
         0
    0.1949];

S3 = [0
    1
         0
   -0.8987
         0
    0.1950];

S4 = [1
         0
         0
         0
    1.0987
   -0.0184];

S5 = [0
    -1
         0
   1.0995
         0
    -0.8363];

S6 = [0
         0
   -1
   -0.0239
    0.8358
         0];
     
Slist = [S1, S2, S3, S4, S5, S6];

intRes = 20;
for j = 1: intRes
   ddthetalist = ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist);     
   [thetalist, dthetalist] = EulerStep(thetalist, dthetalist,  ddthetalist, dt / intRes);
end

sys(1) = thetalist(1);
sys(2) = thetalist(2);
sys(3) = thetalist(3);
sys(4) = thetalist(4);
sys(5) = thetalist(5);
sys(6) = thetalist(6);
sys(7) = dthetalist(1);
sys(8) = dthetalist(2);
sys(9) = dthetalist(3);
sys(10) = dthetalist(4);
sys(11) = dthetalist(5);
sys(12) = dthetalist(6);
sys(13) = ddthetalist(1);
sys(14) = ddthetalist(2);
sys(15) = ddthetalist(3);
sys(16) = ddthetalist(4);
sys(17) = ddthetalist(5);
sys(18) = ddthetalist(6);