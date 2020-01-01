clearvars; close all; clc;
plot_settings

addpath('../python/OCP_pendulum_2dof_rr/results');
filename = 'rr_robot_max_velocity_mutli_stage_final_2019-12-20T11:21:08.mat';
load(filename)

if ismac
%   path for macbook
    save_path = '/Users/Paul/Dropbox/Apps/Overleaf/IIT - Thesis/Images/Chapter3/';
    addpath('/Users/Paul/Documents/GitHub/tameshiwari/matlab')
elseif isunix
%   path for Linux machine
    save_path = '/home/user/Dropbox/Apps/Overleaf/IIT - Thesis/Images/Chapter3/';
    addpath('/home/user/catkin_ws/src/tameshiwari/matlab')
end

save = true(1);

ccycle = colorcycle();
pt = 426.7913;
inch = pt*0.01384;

nj = 2;

var = {'q','\dot{q}','\tau'};
for i = 1:nj
    for j = 1:3
        leg_cell{j,i} = strcat('$',var{j},'_',num2str(i),'$');
    end
end

location = 'north';

figure(1)
hold on
p = plot(time,joint_position.q);
for i = 1:nj
    p(i).Color = ccycle(i,:);
%     plot(time,joint_position.q(i,:),'Color',ccycle(i,:))
end
for i = 1:nj
    plot(time,joint_position.q_lb(i,:),'--','Color',ccycle(i,:))
    plot(time,joint_position.q_ub(i,:),'--','Color',ccycle(i,:))
end
legend(p,leg_cell{1,:},'Location',location)
xlabel('$t$ [s]')
ylabel('$q_i$ [rad]')
grid on
box on
xlim([0 time(end)])
if save
    cleanfigure;
    fig_name = '2dof_q'
    matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth', ...
                'showInfo', false, ...
                'extraaxisoptions',['title style={font=\scriptsize},'...
                       'xlabel style={font=\scriptsize},'...
                       'ylabel style={font=\scriptsize},',...
                       'legend style={font=\scriptsize},',...
                       'ticklabel style={font=\scriptsize}']);
end

figure(2)
hold on
p = plot(time,joint_velocity.qdot);
for i = 1:nj
    p(i).Color = ccycle(i,:);
end
for i = 1:nj
    plot(time,joint_velocity.qdot_lb(i,:),'--','Color',ccycle(i,:))
    plot(time,joint_velocity.qdot_ub(i,:),'--','Color',ccycle(i,:))
end
legend(p,leg_cell{2,:},'Location',location)
xlabel('$t$ [s]')
ylabel('$\dot{q}_i$ [rad/s]')
grid on
box on
xlim([0 time(end)])
if save
    cleanfigure;
    fig_name = '2dof_qdot'
    matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth', ...
                'showInfo', false, ...
                'extraaxisoptions',['title style={font=\scriptsize},'...
                       'xlabel style={font=\scriptsize},'...
                       'ylabel style={font=\scriptsize},',...
                       'legend style={font=\scriptsize},',...
                       'ticklabel style={font=\scriptsize}']);
end

linewidth = 1;
figure(3)
hold on
p = stairs(time,joint_effort.tau');
for i = 1:nj
    p(i).Color = ccycle(i,:);
    p(i).LineWidth = linewidth;
end
for i = 1:nj
    stairs(time,joint_effort.tau_lb(i,:),'--','Color',ccycle(i,:),'LineWidth',linewidth)
    stairs(time,joint_effort.tau_ub(i,:),'--','Color',ccycle(i,:),'LineWidth',linewidth)
end
legend(p,leg_cell{3,:},'Location',location)
xlabel('$t$ [s]')
ylabel('$\tau_i$ [Nm]')
grid on
box on
xlim([0 time(end)])
if save
    cleanfigure;
    fig_name = '2dof_tau'
    matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth', ...
                'showInfo', false, ...
                'extraaxisoptions',['title style={font=\scriptsize},'...
                       'xlabel style={font=\scriptsize},'...
                       'ylabel style={font=\scriptsize},',...
                       'legend style={font=\scriptsize},',...
                       'ticklabel style={font=\scriptsize}']);
end

figure(4)
plot(time,pdot_e(3,:))
hold on
grid on
box on
plot([time(1) time(end)],[-6.19 -6.19])
legend({'$\dot{p}_{e,\iota}$','$\dot{p}_{e,max}$'},'Location',location)
xlabel('$t$ [s]')
ylabel('$\dot{p}_i$ [m/s]')
xlim([0 time(end)])
if save
    cleanfigure;
    fig_name = '2dof_pdot'
    matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth', ...
                'showInfo', false, ...
                'extraaxisoptions',['title style={font=\scriptsize},'...
                       'xlabel style={font=\scriptsize},'...
                       'ylabel style={font=\scriptsize},',...
                       'legend style={font=\scriptsize},',...
                       'ticklabel style={font=\scriptsize}']);
end
