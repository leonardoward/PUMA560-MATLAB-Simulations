close all;
clear all;


% Th_1 = 0*pi/180;
% Th_2 = 0*pi/180;
% Th_3 = 0*pi/180;
% 
% L_1 = 20;
% L_2 = 50;
% L_3 = 40;
% 
% L(1) = Link([0 L_1 0 pi/2]);
% L(2) = Link([0 0 L_2 0]);
% L(3) = Link([0 0 L_3 0]);
% 
% Robot = SerialLink(L);
% 
% Robot.name = 'RRR';
% 
% q = [Th_1 Th_2 Th_3];
% 
% Robot.plot(q);
% 
% % Transformation Matrix from the Forward Kinematics
% T = Robot.fkine(q);

% PX = 50;
% PY = 50;
% PZ = 10;
% 
% L_1 = 20;
% L_2 = 50;
% L_3 = 40;
% 
% L(1) = Link([0 L_1 0 pi/2]);
% L(2) = Link([0 0 L_2 0]);
% L(3) = Link([0 0 L_3 0]);
% 
% Robot = SerialLink(L);
% 
% Robot.name = 'RRR';
% 
% T = [1 0 0 PX;
%      0 1 0 PY;
%      0 0 1 PZ;
%      0 0 0 1];
%  
% J = Robot.ikine(T,  'q0', [0 0 0], 'mask', [1 1 1 0 0 0])*180/pi;
%J = Robot.ikine(T);
%%
mdl_puma560
q = [0 -pi/4 -pi/4 0 pi/8 0];
%plot_box('centre', [-0.5, -0.5], 'size', [1/8, 1/8])
plot_sphere([0.25, 0.25, 0.25],.1,'b')
p560.plot(q)
