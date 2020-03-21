clearvars; close all; clc;
plot_settings

path = matlab.desktop.editor.getActiveFilename;
[path, t1, t2] = fileparts(path);
cd(fullfile(path))

addpath('../python/OCP_pendulum_2dof_rr/results');
filename = 'rr_robot_max_velocity_mutli_stage_final_2019-12-20T11:21:08.mat';
load(filename)

if ismac
%   path for macbook
    save_path = '/Users/Paul/Dropbox/Apps/Overleaf/IIT - Thesis V2/Images/Chapter3/';
    addpath('/Users/Paul/Documents/GitHub/tameshiwari/matlab')
elseif isunix
%   path for Linux machine
    save_path = '/home/user/Dropbox/Apps/Overleaf/IIT - Thesis V2/Images/Chapter3/';
    addpath('/home/user/catkin_ws/src/tameshiwari/matlab')
end
cd(fullfile(save_path))

save = true(1);
% save = false(1);
tikz = false(1);

ccycle = colorcycle();
pt = 426.7913;
inch = pt*0.01384;

nj = 2;

fx = 300;
fy = 187;
ratio = 1.3369;
fy = round(fx/ratio);
sx = 100;
sy = 100;
% figure raster
size_3 = [sx sy fx fy];
size_1 = [sx sy+1.5*fy fx fy];
size_2 = [sx+1*fx sy+1.5*fy fx fy];
size_4 = [sx+1*fx sy fx fy];


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
vline(2,'k:','$t_\iota$')
legend(p,leg_cell{1,:},'Location',location)
xlabel('$t$ [s]')
ylabel('$q_i$ [rad]')
grid on
box on
xlim([0 time(end)])
set(gcf, 'Position',  size_1)
if save
    fig_name = '2dof_q';
%     fig_file = strcat(fig_name,'.eps')
    if tikz
        cleanfigure;
        matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth', ...
                    'showInfo', false, ...
                    'extraaxisoptions',['title style={font=\scriptsize},'...
                           'xlabel style={font=\scriptsize},'...
                           'ylabel style={font=\scriptsize},',...
                           'legend style={font=\scriptsize},',...
                           'ticklabel style={font=\scriptsize}']);
    else
        hgexport(gcf,fig_name);
    end
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
vline(2,'k:','$t_\iota$')
legend(p,leg_cell{2,:},'Location',location)
xlabel('$t$ [s]')
ylabel('$\dot{q}_i$ [rad/s]')
grid on
box on
xlim([0 time(end)])
set(gcf, 'Position',  size_2)
if save
    fig_name = '2dof_qdot';
    if tikz
        cleanfigure;
        matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth', ...
                    'showInfo', false, ...
                    'extraaxisoptions',['title style={font=\scriptsize},'...
                           'xlabel style={font=\scriptsize},'...
                           'ylabel style={font=\scriptsize},',...
                           'legend style={font=\scriptsize},',...
                           'ticklabel style={font=\scriptsize}']);
    else
        hgexport(gcf,fig_name);
    end
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
vline(2,'k:','$t_\iota$')
legend(p,leg_cell{3,:},'Location',location)
xlabel('$t$ [s]')
ylabel('$\tau_i$ [Nm]')
grid on
box on
xlim([0 time(end)])
set(gcf, 'Position',  size_3)
if save
    fig_name = '2dof_tau';
    if tikz
        cleanfigure;
        matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth', ...
                    'showInfo', false, ...
                    'extraaxisoptions',['title style={font=\scriptsize},'...
                           'xlabel style={font=\scriptsize},'...
                           'ylabel style={font=\scriptsize},',...
                           'legend style={font=\scriptsize},',...
                           'ticklabel style={font=\scriptsize}']);
    else
        hgexport(gcf,fig_name);
%         saveas(gcf,fig_name);
    end
end

figure(4)
plot(time,pdot_e(3,:))
hold on
grid on
box on
plot([time(1) time(end)],[-6.19 -6.19])
vline(2,'k:','$t_\iota$')
legend({'$\dot{p}_{e,\boldmath{z}}$','$\dot{p}_{e,\boldmath{z}_{max}}$'},'Location',location,'FontSize',12)
xlabel('$t$ [s]')
ylabel('$\dot{p}_{\boldmath{z}}$ [m/s]')
xlim([0 time(end)])
set(gcf, 'Position',  size_4)
if save
    fig_name = '2dof_pdot';
    if tikz
        cleanfigure;
        matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth', ...
                    'showInfo', false, ...
                    'extraaxisoptions',['title style={font=\scriptsize},'...
                           'xlabel style={font=\scriptsize},'...
                           'ylabel style={font=\scriptsize},',...
                           'legend style={font=\scriptsize},',...
                           'ticklabel style={font=\scriptsize}']);
    else
        hgexport(gcf,fig_name);
    end
end

function hhh=vline(x,in1,in2)
% function h=vline(x, linetype, label)
% 
% Draws a vertical line on the current axes at the location specified by 'x'.  Optional arguments are
% 'linetype' (default is 'r:') and 'label', which applies a text label to the graph near the line.  The
% label appears in the same color as the line.
%
% The line is held on the current axes, and after plotting the line, the function returns the axes to
% its prior hold state.
%
% The HandleVisibility property of the line object is set to "off", so not only does it not appear on
% legends, but it is not findable by using findobj.  Specifying an output argument causes the function to
% return a handle to the line, so it can be manipulated or deleted.  Also, the HandleVisibility can be 
% overridden by setting the root's ShowHiddenHandles property to on.
%
% h = vline(42,'g','The Answer')
%
% returns a handle to a green vertical line on the current axes at x=42, and creates a text object on
% the current axes, close to the line, which reads "The Answer".
%
% vline also supports vector inputs to draw multiple lines at once.  For example,
%
% vline([4 8 12],{'g','r','b'},{'l1','lab2','LABELC'})
%
% draws three lines with the appropriate labels and colors.
% 
% By Brandon Kuczenski for Kensington Labs.
% brandon_kuczenski@kensingtonlabs.com
% 8 November 2001
if length(x)>1  % vector input
    for I=1:length(x)
        switch nargin
        case 1
            linetype='r:';
            label='';
        case 2
            if ~iscell(in1)
                in1={in1};
            end
            if I>length(in1)
                linetype=in1{end};
            else
                linetype=in1{I};
            end
            label='';
        case 3
            if ~iscell(in1)
                in1={in1};
            end
            if ~iscell(in2)
                in2={in2};
            end
            if I>length(in1)
                linetype=in1{end};
            else
                linetype=in1{I};
            end
            if I>length(in2)
                label=in2{end};
            else
                label=in2{I};
            end
        end
        h(I)=vline(x(I),linetype,label);
    end
else
    switch nargin
    case 1
        linetype='r:';
        label='';
    case 2
        linetype=in1;
        label='';
    case 3
        linetype=in1;
        label=in2;
    end
    
    
    
    g=ishold(gca);
    hold on
    y=get(gca,'ylim');
    h=plot([x x],y,linetype);
    if length(label)
        xx=get(gca,'xlim');
        xrange=xx(2)-xx(1);
        xunit=(x-xx(1))/xrange;
        if xunit<0.8
            text(x+0.01*xrange,y(1)+0.1*(y(2)-y(1)),label,'color',get(h,'color'))
        else
            text(x-.05*xrange,y(1)+0.1*(y(2)-y(1)),label,'color',get(h,'color'))
        end
    end     
    if g==0
    hold off
    end
    set(h,'tag','vline','handlevisibility','off')
end % else
if nargout
    hhh=h;
end
end