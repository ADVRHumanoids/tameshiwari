clearvars; close all; clc;
plot_settings
box on
save_path = '/Users/Paul/Dropbox/Apps/Overleaf/IIT - Thesis/Images/test_images/';

str = ['#1f77b4';
       '#ff7f0e';
       '#2ca02c';
       '#d62728';
       '#9467bd';
       '#8c564b';
       '#e377c2';
       '#7f7f7f';
       '#bcdb22';
       '#17becf'];
color = zeros(size(str,1),3);
for i = 1:size(str,1)
    color(i,:) = sscanf(str(i,2:end),'%2x%2x%2x',[1 3])/255;
end
color;

col1 = transpose(1:size(str,1));
col2 = col1+1;
x = linspace(0,1,11);
A = col1 + x.*(col2 - col1);

figure(1)
hold on
for i = 1:size(str,1)
    plot(0:10,A(i,:),'Color',color(i,:))
end
cleanfigure;
fig_name = 'figure1';
matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth' , 'height', '\fheight','showInfo', false);

figure(2)
hold on
for i = 1:size(str,1)
    plot(0:10,A(i,:))
end
cleanfigure;
fig_name = 'figure2';
matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth' , 'height', '\fheight','showInfo', false);

% close all

legendCell = strcat('N=',string(num2cell(1:size(str,1))));

figure(3)
hold on
for i = 1:size(str,1)
    plot(0:10,A(i,:),'Color',color(i,:))
end
legend(legendCell,'Location','eastoutside')
grid on
box on
cleanfigure;
fig_name = 'figure3';
matlab2tikz(strcat(save_path,fig_name,'.tex'),'width', '\fwidth' , 'height', '\fheight','showInfo', false);

