%% Close all Clear all
close all
clear all
clc

%% Robot prism 3m
pr1 = [4.7936, 0.7133, -1.1588];
pr2 = [4.9039, 0.5273, -1.1567];
pr3 = [4.7644, 0.5646, -1.1601];

pr = [pr1', pr2', pr3'];



%% Gate prism 3m
p1 = [4.834, 0.9204, -1.1524];
p2 = [4.945, 0.7433, -1.1495];
p3 = [4.8056, 0.7702, -1.1546];

p = [p1', p2', p3'];



% Plot initial positions
figure
hold on
grid on
scatter3(p(1,:), p(2,:), p(3,:))
scatter3(pr(1,:), pr(2,:), pr(3,:))
view(3)
title('Positions from Leica')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
legend('Left', 'Right')
%% Base
Vgp1 = [-0.0713284, 0.0768284, 0.0209];
Vgp2 = [0.0713284, -0.0768284, 0.0209];
Vgp3 = [-0.0713284, -0.0768284, 0.0209];

b = [Vgp1', Vgp2', Vgp3'];

tf_gate_leica = tf_calc(b,p)
tf_robot_leica = tf_calc(b,pr)

tf_robot_gate = tf_robot_leica * inv(tf_gate_leica)
final_rotation = rotm2eul(tf_robot_gate(1:3,1:3), 'XYZ')*180/pi
final_translation = tf_robot_gate(1:3,4)
