% Program for executing Ziegler-Nichols method for tuning PID controller in Octave
% Input: your plant transfer function; output: Kp, Ki, Kd + step response 
% Note: this is the second method, based on Ogata "Modern Control Eng" Chapter 8
% By: Radon Dhelika

clear;
clc;

pkg load control;

% determine coefficients of your transfer function in this format:
%       E    
% --------------
% As^3+Bs^2+Cs+D

A = 1;
B = 6;
C = 5;
D = 0;
E = 1;

K = 1;  % Initial value of K

% Step 1: Set Ki and Kd to zero 

% Step 2: increase Kp and find a critical value Kcr which outputs sustained oscillation

    while true
        % Get the degree of the characteristic equation
        n = 3;  % Change this to the appropriate degree of your characteristic equation

        % Initialize the Routh table with zeros
        routh = zeros(n + 1, ceil((n + 1) / 2));

        % Fill in the first row of the Routh table with the even-indexed coefficients
        routh(1, :) = [A, C];

        % Fill in the second row of the Routh table with the odd-indexed coefficients
        routh(2, :) = [B, D + (E*K)];

        % Generate the rest of the Routh table
        for i = 3:n+1
            for j = 1:ceil((n + 1) / 2) - 1
                routh(i, j) = -det([routh(i - 1, 1) routh(i - 1, j + 1); routh(i - 2, 1) routh(i - 2, j + 1)]) / routh(i - 1, 1);
            end
        end

        % Check if the first column contains a zero
        containsZero = any(routh(:, 1) == 0);
        if containsZero == 1
            break;  % Exit the loop if a zero is found
        else
            K = K + 1;  % Increment K and try again
        end
    end

    % Display the Routh table and the value of K
    disp('Routh Table:');
    disp(routh);
    disp('Value of Kcr:');
    disp(K);

% Step 3: Find Pcr from the step response
num = [E];
den = [A B C D];
G = tf(num,den);
G1 = feedback(K*G,1);
t = 0:0.01:10;
step_response = step(G1,t);

% Period is calculated by finding zero crossings
max_value = max(step_response);
shifted_response = step_response - 0.5 * max_value;
crossings = find(shifted_response(1:end-1).*shifted_response(2:end) < 0);

Pcr = (crossings(3) - crossings (1))/100 %because t is in the increment of 0.01

% Step 4: calculating Kp, Ki, Kd
Kp = 0.6 * K
Ti = 0.5 * Pcr;
Ki = Kp / Ti
Td = 0.125 * Pcr;
Kd = Kp * Td

% Step 5: plotting response
cont = pid(Kp,Ki,Kd);
G2 = feedback(cont*G,1)

step(G2,t);
xlabel ("time (sec)");
ylabel ("Amplitude");
title ("Step response with parameters of PID controller tuned by Ziegler-Nichols");

% Step 6: fine tuning
% feel free to fine-tune the parameters further by yourself

