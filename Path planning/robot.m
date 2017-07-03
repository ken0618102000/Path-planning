clc;clear all;close;
    
A = imread('L1.png');

% [rho_sim,alpha_sim,beta_sim,car_x_sim,car_y_sim,car_zdir_sim, rho_sim2,alpha_sim2,beta_sim2,x_here,y_here,zdir_here]=...
% textread('子機器人輸出.txt','%f%f%f%f%f%f%f%f%f%f%f%f','headerlines',0);

[Car1_x,Car1_y,Car1_dir,Car2_x,Car2_y,Car2_dir,Car3_x,Car3_y,Car3_dir,Car4_x,Car4_y,Car4_dir]=...
textread('control_pos_m_sim.txt','%f%f%f%f%f%f%f%f%f%f%f%f','headerlines',0);
    
% figure(1);  
% plot(car_x_sim,'r','LineWidth',2);hold on;
% plot(x_here,'g','LineWidth',2);hold on;
% plot(car_y_sim,'LineWidth',2);hold on;
% plot(y_here,'LineWidth',2);hold on;
% plot(car_zdir_sim,'LineWidth',2);hold on;
% plot(zdir_here,'LineWidth',2);hold on;
% legend('car x sim','x here','car_y_sim','y_here','car_zdir_sim','zdir_here');
% grid on;

figure(2);
show_img = image([0 880],[0 880],A);hold on;
plot(Car1_x,Car1_y,'r','LineWidth',2);hold on;
plot(Car2_x,Car2_y,'g','LineWidth',2);hold on;
plot(Car3_x,Car3_y,'blue','LineWidth',2);hold on;
plot(Car4_x,Car4_y,'c','LineWidth',2);hold on;


   