close all
% Create a trajectory to follow using functions from Chapter 9
thetastart = [0; 0; 0; 0; 0; 0];
Tf = 1.5;
N= 1501;  % N = 插值数*Tf + 1

dt =0 : Tf / (N - 1) : Tf;
% thetaArr = [dt', -exp(-dt') + 1/2*dt'.^2 - dt' + 1];
% thetamat = (-exp(-dt) + 1/2*dt.^2 - dt + 1).*[1; 1; 1; 1; 1; 1];
% dthetamat = (exp(-dt) + dt - 1).*[1; 1; 1; 1; 1; 1];
% ddthetamat = (-exp(-dt) + 1).*[1; 1; 1; 1; 1; 1];
thetamat = dt.*[0; 0; 0; 0; 0; 0];
dthetamat = dt.*[0; 0; 0; 0; 0; 0];
ddthetamat = dt.*[0; 0; 0; 0; 0; 0];
Ftipmat = zeros(N, 6);
%Initialise robot descripstion (Example with 3 links)
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

taumat = InverseDynamicsTrajectory(thetamat', dthetamat', ddthetamat', ...
                                 g, Ftipmat, Mlist, Glist, Slist);
 
tauArr1 = [dt', taumat(:, 1)];
tauArr2 = [dt', taumat(:, 2)];
tauArr3 = [dt', taumat(:, 3)];
tauArr4 = [dt', taumat(:, 4)];
tauArr5 = [dt', taumat(:, 5)];
tauArr6 = [dt', taumat(:, 6)];
% Output using matplotlib to plot the joint forces/torques
time=dt;

% 绘制力矩曲线
figure
subplot(3, 2, 1);
plot(taumat(:, 1), 'b', 'LineWidth', 1.5);
xlabel('Time')
ylabel('Torque')
legend('$\tau_{1}$', 'interpreter','latex', 'FontSize', 12)

subplot(3, 2, 2);
plot(taumat(:, 2), 'b', 'LineWidth', 1.5);
xlabel('Time')
ylabel('Torque')
legend('$\tau_{2}$', 'interpreter','latex', 'FontSize', 12)

subplot(3, 2, 3);
plot(taumat(:, 3), 'b', 'LineWidth', 1.5);
xlabel('Time')
ylabel('Torque')
legend('$\tau_{3}$', 'interpreter','latex', 'FontSize', 12)

subplot(3, 2, 4);
plot(taumat(:, 4), 'b', 'LineWidth', 1.5);
xlabel('Time')
ylabel('Torque')
legend('$\tau_{4}$', 'interpreter','latex', 'FontSize', 12)

subplot(3, 2, 5);
plot(taumat(:, 5), 'b', 'LineWidth', 1.5);
xlabel('Time')
ylabel('Torque')
legend('$\tau_{5}$', 'interpreter','latex', 'FontSize', 12)

subplot(3, 2, 6);
plot(taumat(:, 6), 'b', 'LineWidth', 1.5);
xlabel('Time')
ylabel('Torque')
legend('$\tau_{6}$', 'interpreter','latex', 'FontSize', 12)
