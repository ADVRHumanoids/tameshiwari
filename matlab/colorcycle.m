function arrayRGB = colorcycle()
% This is the colorcycle of matplotlib in python
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
arrayRGB = color;
end

