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


%% Base
Vgp1 = [-0.0713284, 0.0768284, 0.0209];
Vgp2 = [0.0713284, -0.0768284, 0.0209];
Vgp3 = [-0.0713284, -0.0768284, 0.0209];

b = [Vgp1', Vgp2', Vgp3'];
%% math
disp('Section 5m Distance Plate Prisms')
disp('Distance of individual points')
dist = sqrt((pr1(1)-p1(1))^2+(pr1(2)-p1(2))^2+(pr1(3)-p1(3))^2)
dist = sqrt((pr2(1)-p2(1))^2+(pr2(2)-p2(2))^2+(pr2(3)-p2(3))^2)
dist = sqrt((pr3(1)-p3(1))^2+(pr3(2)-p3(2))^2+(pr3(3)-p3(3))^2)

disp('Direct Offset and Rotation')
tf_mat = tf_calc(p,pr);

%% Robot prism 1m
pr1 = [1.2127, 0.7072, -1.1661];
pr2 = [1.4105, 0.6411, -1.1642];
pr3 = [1.2854, 0.5736, -1.1632];
pr = [pr1', pr2', pr3'];

%% Gate prism 1m
p1 = [1.1122, 0.8919, -1.1648];
p2 = [1.3102, 0.8254, -1.1636];
p3 = [1.185, 0.7579, -1.1628];
p = [p1', p2', p3'];

disp('Section 1m Distance Plate Prisms')
disp('Distance of individual points')
dist = sqrt((pr1(1)-p1(1))^2+(pr1(2)-p1(2))^2+(pr1(3)-p1(3))^2)
dist = sqrt((pr2(1)-p2(1))^2+(pr2(2)-p2(2))^2+(pr2(3)-p2(3))^2)
dist = sqrt((pr3(1)-p3(1))^2+(pr3(2)-p3(2))^2+(pr3(3)-p3(3))^2)

disp('Direct Offset and Rotation')
tf_mat = tf_calc(p,pr);
