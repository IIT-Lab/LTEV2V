function [] = PlotPDF(HistogramMatrix)
figure;
hold on
a=transpose(HistogramMatrix);

hold on;
[p,x] = hist(a(:, 1));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 2));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 3));
c=plot(x,p/sum(p));

[p,x] = hist(a(:, 4));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 5));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 6));
c=plot(x,p/sum(p));

[p,x] = hist(a(:, 7));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 8));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 9));
c=plot(x,p/sum(p));

[p,x] = hist(a(:, 10));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 11));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 12));
c=plot(x,p/sum(p));


[p,x] = hist(a(:, 13));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 14));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 15));
c=plot(x,p/sum(p));

[p,x] = hist(a(:, 16));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 17));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 18));
c=plot(x,p/sum(p));

[p,x] = hist(a(:, 19));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 20));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 21));
c=plot(x,p/sum(p));

[p,x] = hist(a(:, 22));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 23));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 24));
c=plot(x,p/sum(p));

[p,x] = hist(a(:, 25));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 26));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 27));
c=plot(x,p/sum(p));


[p,x] = hist(a(:, 28));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 29));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 30));
c=plot(x,p/sum(p));


[p,x] = hist(a(:, 31));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 32));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 33));
c=plot(x,p/sum(p));


[p,x] = hist(a(:, 34));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 35));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 36));
c=plot(x,p/sum(p));

[p,x] = hist(a(:, 37));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 38));
c=plot(x,p/sum(p));
[p,x] = hist(a(:, 39));
c=plot(x,p/sum(p));

legend('0-10m','10-20m','20-30m','30-40m','40-50m','50-60m','60-70m','70-80m','80-90m','90-100m','100-110m','110-120m','120-130m','130-140m','140-150m','150-160m','160-170m','170-180m','180-190m','190-200m','200-210m','210-220m','220-230m','230-240m','240-250m','250-260m','260-270m','270-280m','280-290m','290-300m','300-310m','310-320m','320-330m','330-340m','340-350m','350-360m','360-370m','370-380m','380-390m');
hold off
saveas(c, 'OutputGraph\pdf.png','png');




