function [] = PlotCDF(HistogramMatrix)
figure;
hold on
a=transpose(HistogramMatrix);

b=cdfplot(a(:, 1));
b=cdfplot(a(:, 2));
b=cdfplot(a(:, 3));

b=cdfplot(a(:, 4));
b=cdfplot(a(:, 5));
b=cdfplot(a(:, 6));

b=cdfplot(a(:, 7));
b=cdfplot(a(:, 8));
b=cdfplot(a(:, 9));

b=cdfplot(a(:, 10));
b=cdfplot(a(:, 11));
b=cdfplot(a(:, 12));


b=cdfplot(a(:, 13));
b=cdfplot(a(:, 14));
b=cdfplot(a(:, 15));

b=cdfplot(a(:, 16));
b=cdfplot(a(:, 17));
b=cdfplot(a(:, 18));

b=cdfplot(a(:, 19));
b=cdfplot(a(:, 20));
b=cdfplot(a(:, 21));

b=cdfplot(a(:, 22));
b=cdfplot(a(:, 23));
b=cdfplot(a(:, 24));

b=cdfplot(a(:, 25));
b=cdfplot(a(:, 26));
b=cdfplot(a(:, 27));


b=cdfplot(a(:, 28));
b=cdfplot(a(:, 29));
b=cdfplot(a(:, 30));


b=cdfplot(a(:, 31));
b=cdfplot(a(:, 32));
b=cdfplot(a(:, 33));


b=cdfplot(a(:, 34));
b=cdfplot(a(:, 35));
b=cdfplot(a(:, 36));

b=cdfplot(a(:, 37));
b=cdfplot(a(:, 38));
b=cdfplot(a(:, 39));

legend('0-10m','10-20m','20-30m','30-40m','40-50m','50-60m','60-70m','70-80m','80-90m','90-100m','100-110m','110-120m','120-130m','130-140m','140-150m','150-160m','160-170m','170-180m','180-190m','190-200m','200-210m','210-220m','220-230m','230-240m','240-250m','250-260m','260-270m','270-280m','280-290m','290-300m','300-310m','310-320m','320-330m','330-340m','340-350m','350-360m','360-370m','370-380m','380-390m');
hold off
saveas(b, 'OutputGraph\cdf.png','png');

