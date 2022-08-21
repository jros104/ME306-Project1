close
clc
clear

filename = 'TuningDiagV2.txt';

FID = fopen(filename, 'r');

fgets(FID);

data = fscanf(FID, '%f', [4, inf]);
T = data(1,:);
CL = data(2,:);
CR = data(3,:);
CD = data(4,:);


figure(1);
plot(T,CL,'r', T, CR, 'b',T,CD,'k');
legend('CL', 'CR','CD');
ylabel('Distance (counts)');
xlabel('Time (s)');
grid on
grid minor