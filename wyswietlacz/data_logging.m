clc; clear all; close all;

s = serialport('COM5', 115200); % Zmień 'COM5' na właściwy numer portu

% Inicjalizacja pustych tablic na dane
measuredValue = [];
W_rpm_setpoint = [];
controlValue = [];

% Tworzenie nowego okna do rysowania wykresu
figure;
hold on;
    data = readline(s); % Odczytaj linię danych
    values = str2double(split(data, ',')); % Rozdziel dane i przekształć je na liczby

while true
    data = readline(s); % Odczytaj linię danych
    values = str2double(split(data, ',')); % Rozdziel dane i przekształć je na liczby
    
    % Dodawanie nowych wartości do tablic
    measuredValue(end+1) = values(1);
    W_rpm_setpoint(end+1) = values(2);
    controlValue(end+1) = values(3);

    % Rysowanie wykresów
    subplot(2,1,1);
    plot(measuredValue, 'b'); % Niebieski dla measuredValue
    hold on;
    plot(W_rpm_setpoint, 'r'); % Czerwony dla W_rpm_setpoint
    legend('Measured Value', 'RPM Setpoint');
    xlabel('Czas [próbki]');
    ylabel('Wartości');

    subplot(2,1,2);
    plot(controlValue, 'g'); % Zielony dla controlValue
    legend('Control signal');
    xlabel('Czas [próbki]');
    ylabel('Wartości');


    % Pauza krótka, aby GUI Matlab mógł się odświeżyć
    pause(0.05);

    % Opcjonalnie, możesz dodać warunek, aby przerwać pętlę po spełnieniu pewnego kryterium
end

hold off;