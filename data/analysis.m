close all

mega = readtable('mega1595460993.csv');
mega = table2array(mega);
mega = mega(2:101,:);
micro = readtable('micro1595460993.csv');
micro = table2array(micro);
micro = micro(2:101,:);
mini = readtable('mini1595460993.csv');
mini = table2array(mini);
mini = mini(2:101,:);

mega_x_avg = mean(mega(:,1));
micro_x_avg = mean(micro(:,1));
mini_x_avg = mean(mini(:,1));

mega_x_dist = mega(:,1)-mega_x_avg;
micro_x_dist = micro(:,1)-micro_x_avg;
mini_x_dist = mini(:,1)-mini_x_avg;

figure
hold on
title('Big 360 Prism')
xlabel('Offest x (m)')
ylabel('Count')
histfit(mega_x_dist)
figure
hold on
title('Micro 360 Prism')
xlabel('Offest x (m)')
ylabel('Count')
histfit(micro_x_dist)
figure
hold on
title('Mini 360 Prism')
xlabel('Offest x (m)')
ylabel('Count')
histfit(mini_x_dist)

mega_y_avg = mean(mega(:,2));
micro_y_avg = mean(micro(:,2));
mini_y_avg = mean(mini(:,2));

mega_y_dist = mega(:,2)-mega_y_avg;
micro_y_dist = micro(:,2)-micro_y_avg;
mini_y_dist = mini(:,2)-mini_y_avg;

figure
hold on
title('Big 360 Prism')
xlabel('Offest y (m)')
ylabel('Count')
histfit(mega_y_dist)
figure
hold on
title('Micro 360 Prism')
xlabel('Offest y (m)')
ylabel('Count')
histfit(micro_y_dist)
figure
hold on
title('Mini 360 Prism')
xlabel('Offest y (m)')
ylabel('Count')
histfit(mini_y_dist)

mega_z_avg = mean(mega(:,3));
micro_z_avg = mean(micro(:,3));
mini_z_avg = mean(mini(:,3));

mega_z_dist = mega(:,3)-mega_z_avg;
micro_z_dist = micro(:,3)-micro_z_avg;
mini_z_dist = mini(:,3)-mini_z_avg;


figure
hold on
title('Big 360 Prism')
xlabel('Offest z (m)')
ylabel('Count')
histfit(mega_z_dist)
figure
hold on
title('Micro 360 Prism')
xlabel('Offest z (m)')
ylabel('Count')
histfit(micro_z_dist)
figure
hold on
title('Mini 360 Prism')
xlabel('Offest z (m)')
ylabel('Count')
histfit(mini_z_dist)

mega_x_std = std(mega(:,1));
micro_x_std = std(micro(:,1));
mini_x_std = std(mini(:,1));


mega_y_std = std(mega(:,2));
micro_y_std = std(micro(:,2));
mini_y_std = std(mini(:,2));


mega_z_std = std(mega(:,3));
micro_z_std = std(micro(:,3));
mini_z_std = std(mini(:,3));

mega_std = [mega_x_std, mega_y_std, mega_z_std]
mini_std = [mini_x_std, mini_y_std, mini_z_std]
micro_std = [micro_x_std, micro_y_std, micro_z_std]