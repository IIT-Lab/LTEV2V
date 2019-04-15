function []=PlotDisPDF(distance)
figure;
[p,x]  = hist(distance)
d=plot(x,p/sum(p));
saveas(d, 'OutputGraph\dis.png','png');




