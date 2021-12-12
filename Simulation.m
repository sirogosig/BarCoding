
%% Starting engine

clc         % clear the command line
clear       % empty all the  variables
close all   %closing figures

%% Data

syms od n
n=[0:1:70]; %real values
W=16;
alpha=3.13/100; %spatial error
%hover_speed=;
beta=-1.666667/100; % velocity error,takes different values based on speed !
x = W/2 +n*W*(alpha/(1-alpha)+beta*(1-alpha));
%eqn= W/2+n*W*(alpha/(1-alpha)+beta*(1-alpha))==0;
%S=solve(eqn,n);



%% Graphs
stem(n, x, linestyle='none');

title('Capacitor discharging for different surfaces');
grid on;
xlabel('Bit number');
ylabel('Sampling position');

axis([0 50 0 W])