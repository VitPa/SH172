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
title('VELOCITÀ v', 'FontSize', 14)
xlabel("t [s]")
ylabel("v [m/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,5), LineWidth = 1.5)
hold on
title('VELOCITÀ ANGOLARE p', 'FontSize', 14)
xlabel("t [s]")
ylabel("p [°/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,8), LineWidth = 1.5)
hold on
title('ROLLIO \phi', 'FontSize', 14)
xlabel("t [s]")
ylabel("\phi [°]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,13), LineWidth = 1.5)
hold on
title('SPOSTAMENTO Y', 'FontSize', 14)
xlabel("t [s]")
ylabel("Y NED[m]")
grid on
hold off


nexttile;
plot(DATI(:,1), DATI(:,2), LineWidth = 1.5)
hold on
title('VELOCITÀ u', 'FontSize', 14)
xlabel("t [s]")
ylabel("u [m/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,6), LineWidth = 1.5)
hold on
title('VELOCITÀ ANGOLARE q', 'FontSize', 14)
xlabel("t [s]")
ylabel("q [°/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,9), LineWidth = 1.5)
hold on
title('BECCHEGGIO \theta', 'FontSize', 14)
xlabel("t [s]")
ylabel("\theta [°]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,12), LineWidth = 1.5)
hold on
title('SPOSTAMENTO X', 'FontSize', 14)
xlabel("t [s]")
ylabel("X NED [m]")
grid on
hold off


nexttile;
plot(DATI(:,1), DATI(:,4), LineWidth = 1.5)
hold on
title('VELOCITÀ w', 'FontSize', 14)
xlabel("t [s]")
ylabel("w [m/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,7), LineWidth = 1.5)
hold on
title('VELOCITÀ ANGOLARE r', 'FontSize', 14)
xlabel("t [s]")
ylabel("r [°/s]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,10), LineWidth = 1.5)
hold on
title('IMBARDATA \psi', 'FontSize', 14)
xlabel("t [s]")
ylabel("\psi [°]")
grid on
hold off

nexttile;
plot(DATI(:,1), DATI(:,11), LineWidth = 1.5)
hold on
title('QUOTA h', 'FontSize', 14)
xlabel("t [s]")
ylabel("h [m]")
grid on
hold off

%%
figure(2);
plot3(DATI(:,12),DATI(:,10),DATI(:,11), LineWidth = 3)
hold on
plot3(DATI(1,12), DATI(1,10), DATI(1,11), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g')
plot3(DATI(end,12), DATI(end,10), DATI(end,11), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
hold off
title('TRAIETTORIA', 'FontSize', 14)
grid on
xlabel('X NED [m]')
ylabel('Y NED [m]')
zlabel('h [m]')

%% COMANDI
COM = load('COMMAND.txt');

figure(3)
tiledlayout(4,1);

nexttile;
plot(COM(:,1), COM(:,2), LineWidth = 2)
hold on
grid on
title('COMANDO ALETTONE', 'FontSize', 14)
xlabel("t [s]")
ylabel("da [°]")
hold off

nexttile;
plot(COM(:,1), COM(:,3), LineWidth = 2)
hold on
grid on
title('COMANDO EQUILIBRATORE', 'FontSize', 14)
xlabel("t [s]")
ylabel("de [°]")
hold off

nexttile;
plot(COM(:,1), COM(:,4), LineWidth = 2)
hold on
grid on
title('COMANDO TIMONE', 'FontSize', 14)
xlabel("t[s]")
ylabel("dr [°]")
hold off

nexttile;
plot(COM(:,1), COM(:,5)*100, LineWidth = 2)
hold on
grid on
title('COMANDO MANETTA', 'FontSize', 14)
xlabel("t [s]")
ylabel("manetta [%]")
hold off

%% VALORI AGGIUNTIVI
AGG = load('EXTRA.txt');
SVAL = load('SVALUE.txt');

figure(4)
tiledlayout(2,3);

nexttile;
plot(AGG(:,1), AGG(:,2), LineWidth = 2)
hold on
grid on
title('\alpha', 'FontSize', 14)
xlabel("t [s]")
ylabel("\alpha [°]")
hold off

nexttile;
plot(AGG(:,1), AGG(:,3), LineWidth = 2)
hold on
grid on
title('\beta', 'FontSize', 14)
xlabel("t[s]")
ylabel("\beta [°]")
hold off

nexttile;
plot(AGG(:,1), AGG(:,4), LineWidth = 2)
hold on
grid on
title('\gamma', 'FontSize', 14)
xlabel("t [s]")
ylabel("\gamma [°]")
hold off

nexttile;
plot(AGG(:,1), AGG(:,5), LineWidth = 2)
hold on
grid on
title('Velocity (TAS)', 'FontSize', 14)
xlabel("t [s]")
ylabel("V [m/s]")
hold off

nexttile;
plot(AGG(:,1), AGG(:,6), LineWidth = 2)
hold on
yline(SVAL(2), 'r', 'LineWidth', 1);
grid on
title('Thrust', 'FontSize', 14)
xlabel("t [s]")
ylabel("T [N]")
hold off

nexttile;
plot(AGG(:,1), AGG(:,7), 'b', LineWidth = 2)
hold on
plot(AGG(:,1), AGG(:,8).*9.80665.*cos(AGG(:,2).*(pi/180)), 'r', LineWidth = 1)
grid on
title('Portanza', 'FontSize', 14)
xlabel("t [s]")
ylabel("-Z [N]")
hold off



% figure(5)
% tiledlayout(4,1);
% 
% nexttile;
% plot(AGG(:,1), AGG(:,2), LineWidth = 2)
% hold on
% grid on
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