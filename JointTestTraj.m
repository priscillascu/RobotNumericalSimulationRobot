% CRP14RH10��е���涯��ѧ��ֵ����ģ��
% ���룺6�ؽڽǶȡ����ٶȡ��Ǽ��ٶ�
% ������ؽ���������

function [sys,x0,str,ts] = JointTestTraj(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % ���ó�ʼ���Ӻ���
  case 1,
    sys=[];
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %��������Ӻ���
  case 4,
    sys=[];   %������һ����ʱ���Ӻ���
  case 9,
    sys=[];    %��ֹ�����Ӻ���
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %��ʼ���Ӻ���

sizes = simsizes;

sizes.NumContStates  = 0;  %����״̬��������
sizes.NumDiscStates  = 0;  %��ɢ״̬��������
sizes.NumOutputs     = 1;  %�����������
sizes.NumInputs      = 0;   %�����������
sizes.DirFeedthrough = 0;   %�����ź��Ƿ�������Ӻ����г���
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %��ʼֵ
str = [];   
ts  = [];   %[0 0]��������ϵͳ��[-1 0]��ʾ�̳���ǰ�Ĳ���ʱ������
simStateCompliance = 'UnknownSimState';
global sumv
sumv = 0;

function sys=mdlOutputs(t,x,u)   %��������Ӻ���
global sumv
v0 = 0;
v1 = sqrt(0.2^2+0.2^2+0.2^2)/(16*10/7);  %���õ�һ�����ٶȹյ��ٶ�v1

dt = 10/7;
t1 = dt;
t2 = 2*dt;
t3 = 3*dt;
t4 = 4*dt;
t5 = 5*dt;
t6 = 6*dt;
t7 = 7*dt;
J = 2*v1/(t1^2);    %�Ӽ��ٶ�
amax = J*t1;   %���ٶ����ֵ
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