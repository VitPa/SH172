clc
clear
close all

%% VETTORE DI STATO
DATI = load('DATA.txt');

figure(1)
tiledlayout(3,4);

nexttile;
plot(DATI(:,1), DATI(:,3), LineWidth = 1.5)
hold on
title('VELOCITA v')
xlabel("t[s]")
ylabel("v[m/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,5), LineWidth = 1.5)
hold on
title('VELOCITA ANGOLARE p')
xlabel("t[s]")
ylabel("p[°/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,8), LineWidth = 1.5)
hold on
title('ROLLIO \phi')
xlabel("t[s]")
ylabel("\phi[°]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,13), LineWidth = 1.5)
hold on
title('SPOSTAMENTO Y')
xlabel("t[s]")
ylabel("Y NED[m]")
grid on
hold off


nexttile;
plot(DATI(:,1), DATI(:,2), LineWidth = 1.5)
hold on
title('VELOCITA u')
xlabel("t[s]")
ylabel("u[m/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,6), LineWidth = 1.5)
hold on
title('VELOCITA ANGOLARE q')
xlabel("t[s]")
ylabel("q[°/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,9), LineWidth = 1.5)
hold on
title('BECCHEGGIO \theta')
xlabel("t[s]")
ylabel("\theta[°]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,12), LineWidth = 1.5)
hold on
title('SPOSTAMENTO X')
xlabel("t[s]")
ylabel("X NED[m]")
grid on
hold off


nexttile;
plot(DATI(:,1), DATI(:,4), LineWidth = 1.5)
hold on
title('VELOCITA w')
xlabel("t[s]")
ylabel("w[m/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,7), LineWidth = 1.5)
hold on
title('VELOCITA ANGOLARE r')
xlabel("t[s]")
ylabel("r[°/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,10), LineWidth = 1.5)
hold on
title('IMBARDATA \psi')
xlabel("t[s]")
ylabel("\psi[°]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,11), LineWidth = 1.5)
hold on
title('QUOTA h')
xlabel("t[s]")
ylabel("Z NED[m]")
grid on
hold off


figure(2);
plot3(DATI(:,12),DATI(:,10),DATI(:,11))
title('TRAIETTORIA NED')
grid on
xlabel('X NED[m]')
ylabel('Y NED[m]')
zlabel('Z NED[m]')

%% COMANDI
COM = load('COMMAND.txt');

figure(3)
tiledlayout(4,1);

nexttile;
plot(COM(:,1), COM(:,2), LineWidth = 2)
hold on
title('da')
xlabel("t[s]")
ylabel("da [deg]")
hold off

nexttile;
plot(COM(:,1), COM(:,3), LineWidth = 2)
hold on
title('de')
xlabel("t[s]")
ylabel("de [deg]")
hold off

nexttile;
plot(COM(:,1), COM(:,4), LineWidth = 2)
hold on
title('dr')
xlabel("t[s]")
ylabel("dr [deg]")
hold off

nexttile;
plot(COM(:,1), COM(:,5), LineWidth = 2)
hold on
title('manetta')
xlabel("t[s]")
ylabel("manetta [RPM]")
hold off

%% VALORI AGGIUNTIVI
AGG = load('EXTRA.txt');

figure(4)
plot(AGG(:,1), AGG(:,2), LineWidth = 2)
hold on
title('aggiuntivi')
xlabel("t[s]")
ylabel("aggiuntivi []")
hold off

% figure(5)
% tiledlayout(4,1);
% 
% nexttile;
% plot(AGG(:,1), AGG(:,2), LineWidth = 2)
% hold on
% title('da')
% xlabel("t[s]")
% ylabel("da [deg]")
% hold off
% 
% nexttile;
% plot(AGG(:,1), AGG(:,3), LineWidth = 2)
% hold on
% title('de')
% xlabel("t[s]")
% ylabel("de [deg]")
% hold off
% 
% nexttile;
% plot(AGG(:,1), AGG(:,4), LineWidth = 2)
% hold on
% title('dr')
% xlabel("t[s]")
% ylabel("dr [deg]")
% hold off
% 
% nexttile;
% plot(AGG(:,1), AGG(:,5), LineWidth = 2)
% hold on
% title('manetta')
% xlabel("t[s]")
% ylabel("manetta [RPM]")
% hold off