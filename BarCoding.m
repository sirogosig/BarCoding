
%% Starting engine

clc         % clear the command line
clear       % empty all the  variables
close all   %closing figures

%% Data

syms x

exp_low= exp(-x)*heaviside(x)+heaviside(-x);
exp_high= exp(-3*x)*heaviside(x)+heaviside(-x);


%% Graphs
xx = -2 : 0.01 : 10; %real values

exp_low_PLOT = subs(exp_low,x,xx);
exp_high_PLOT = subs(exp_high,x,xx);

exp_low_PLOT = double(exp_low_PLOT);
exp_high_PLOT = double(exp_high_PLOT);

plot(xx,exp_low_PLOT,'r','linewidth',2);
hold on
plot(xx,exp_high_PLOT,'g','linewidth',2);

title('Capacitor discharging for different surfaces');
grid off;
xlabel('Time');
ylabel('Output voltage');
set(gca,'xtick',[]);
set(gca,'ytick',[]);
yline(0,'--');
legend('Dark surface','Bright surface','');


axis([-2 7 -0.2 1.2])
grid on;