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
q = [0 -pi/4 -pi/4 0 pi/8 0]
p560.plot(q)
% T = p560.fkine(q)
% qi = p560.ikine(T);
% 
% qi
% q
% 
% qi = p560.ikine6s(T)
% p560.fkine(qi)
% 
% p560.ikine6s(T, 'rdf')
% 
% T1 = transl(0.6, -0.5, 0.0) % define the start point
% T2 = transl(0.4, 0.5, 0.2)	% and destination
% T = ctraj(T1, T2, 50); 	% compute a Cartesian path
% 
% q = p560.ikine6s(T); 
% about q
% 
% subplot(3,1,1); plot(q(:,1)); xlabel('Time (s)'); ylabel('Joint 1 (rad)');
% subplot(3,1,2); plot(q(:,2)); xlabel('Time (s)'); ylabel('Joint 2 (rad)');
% subplot(3,1,3); plot(q(:,3)); xlabel('Time (s)'); ylabel('Joint 3 (rad)');
% clf; p560.plot(q)
%%
% % Copyright (C) 1993-2017, by Peter I. Corke
% %
% % This file is part of The Robotics Toolbox for MATLAB (RTB).
% % 
% % RTB is free software: you can redistribute it and/or modify
% % it under the terms of the GNU Lesser General Public License as published by
% % the Free Software Foundation, either version 3 of the License, or
% % (at your option) any later version.
% % 
% % RTB is distributed in the hope that it will be useful,
% % but WITHOUT ANY WARRANTY; without even the implied warranty of
% % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% % GNU Lesser General Public License for more details.
% % 
% % You should have received a copy of the GNU Leser General Public License
% % along with RTB.  If not, see <http://www.gnu.org/licenses/>.
% %
% % http://www.petercorke.com
% 
% %%begin
% 
% % A serial link manipulator comprises a series of links.  Each link is described
% % by four Denavit-Hartenberg parameters.
% %
% % Let's define a simple 2 link manipulator.  The first link is
% 
% L1 = Link('d', 0, 'a', 1, 'alpha', pi/2)
% 
% % The Link object we created has a number of properties
% L1.a
% L1.d
% 
% % and we determine that it is a revolute joint
% L1.isrevolute
% 
% % For a given joint angle, say q=0.2 rad, we can determine the link transform
% % matrix
% L1.A(0.2)
% 
% % The second link is
% L2 = Link('d', 0, 'a', 1, 'alpha', 0)
% 
% % Now we need to join these into a serial-link robot manipulator
% 
% bot = SerialLink([L1 L2], 'name', 'my robot')
% % The displayed robot object shows a lot of details.  It also has a number of
% % properties such as the number of joints
% bot.n
% 
% % Given the joint angles q1 = 0.1 and q2 = 0.2 we can determine the pose of the
% % robot's end-effector
% 
% bot.fkine([0.1 0.2])
% % which is referred to as the forward kinematics of the robot.  This, and the
% % inverse kinematics are covered in separate demos.
% 
% % Finally we can draw a stick figure of our robot
% 
% bot.plot([0.1 0.2])
