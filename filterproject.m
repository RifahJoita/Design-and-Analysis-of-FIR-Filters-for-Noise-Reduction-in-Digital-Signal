fs = 20000;
n = 50;
passband_ripple = 1;
stopband_attenuation = 50;
passband_freq = 4500;
stopband_freq = 5000;

fc=(stopband_freq+passband_freq)/2;

M = floor((n - 1) / 2);
Fs = fs;
wl = 2*pi*fc/fs;

h = zeros(1, n);
h(M + 1) = wl / pi;

for k = 1:M
    pos = k + M + 1;
    h(pos) = (sin(k * wl) / (k * pi));
    new_pos = pos - (2 * k);
    h(new_pos) = h(pos);
end

            if stopband_attenuation <= 21 && passband_ripple > 0.7416
                window_choice = 'rectangular';
            elseif stopband_attenuation <= 44 && passband_ripple > 0.0546
                window_choice = 'hanning';
            elseif stopband_attenuation <= 53 && passband_ripple > 0.0196
                window_choice = 'hamming';
            elseif stopband_attenuation <= 74 && passband_ripple > 0.0017
                window_choice = 'blackman';
            else
                error('Invalid window choice.');
            end
            fprintf('Chosen window: %s\n', window_choice);
           
          


            if strcmpi(window_choice, 'rectangular')
                w_rect = ones(1, n);
                num = h.*w_rect;
            elseif strcmpi(window_choice, 'hanning')
                w = zeros(1, n);
                for k = 0:M
                    pos = k + M + 1;
                    w(pos) = 0.5 + 0.5 * cos(k * pi / M);
                    new_pos = pos - (2 * k);
                    w(new_pos) = w(pos);
                end
                num = h.*w;
            elseif strcmpi(window_choice, 'hamming')
                w_hamm = zeros(1, n);
                for k = 0:1:M
                    pos = k + M + 1;
                    w_hamm(pos) = 0.54 + 0.46 * cos(k * pi / M);
                    new_pos = pos - (2 * k);
                    w_hamm(new_pos) = w_hamm(pos);
                end
                num =  h.*w_hamm;
            elseif strcmpi(window_choice, 'blackman')
                w = zeros(1, n);
                for k = 0:M
                    pos = k + M + 1;
                    w(pos) = 0.42 + 0.5 * cos(k * pi / M) + 0.08 * cos(2 * k * pi / M);
                    new_pos = pos - (2 * k);
                    w(new_pos) = w(pos);
                end
                num = h.*w;
            else
                error('Invalid window choice.');
            end



% Calculate the frequency response of the filter
[hz, w] = freqz(num, 1, 512, fs);

% Create the FIR filter using the selected window function


% Check the selected window function and create the FIR filter
        if strcmpi(window_choice, 'rectangular')
        b = fir1(n, (passband_freq + stopband_freq) / (2 * Fs), 'low', rectwin(n+1));
        elseif strcmpi(window_choice, 'hanning')
        b = fir1(n, (passband_freq + stopband_freq) / (2 * Fs), 'low', hann(n+1));
        elseif strcmpi(window_choice, 'hamming')
        b = fir1(n, (passband_freq + stopband_freq) / (2 * Fs), 'low', hamming(n+1));
    	 strcmpi(window_choice, 'blackman')
        b = fir1(n, (passband_freq + stopband_freq) / (2 * Fs), 'low', blackman(n+1));
        else
            error('Invalid window choice.');
        end


% Check the stability

figure;
zplane(b, 1);
title('Pole-Zero Plot (FIR Filter)');
grid on;

% Calculate the step response of the filter
[step_response, time_step] = stepz(b, 1, 1024, Fs);

% Calculate the impulse response of the filter
[impulse_response, time_impulse] = impz(b, 1, 1024, Fs);


% Create the sample index for the x-axis
sample_index = 0:1023;

% Plot the step response
figure;

plot(sample_index, step_response);
title('Step Response of the FIR Filter');
xlabel('Sample Index');
ylabel('Amplitude');
grid on;

% Plot the impulse response
figure;
plot(sample_index, impulse_response);
title('Impulse Response of the FIR Filter');
xlabel('Sample Index');
ylabel('Amplitude');
grid on;

figure(1)
subplot(2, 1, 1)
plot(w, 20*log10(abs(hz)));
xlabel('Frequency (Hz)');
ylabel('Magnitude Response (dB)');
subplot(2, 1, 2)
plot(w, 180*unwrap(angle(hz))/pi)
xlabel('Frequency (Hz)')
ylabel('Phase (degrees)')


% Generate the input signal
t = 0:1/Fs:0.1;  % Time vector from 0 to 0.1 seconds
input_signal = sin(6000 * pi * t) + sin(1000 * pi * t) + sin(200 * pi * t) + sin(100 * pi * t);

% Add noise to the input signal
SNR_dB = 1;  % Signal-to-noise ratio
noise = randn(size(input_signal));  % Generate white Gaussian noise
noise_power = 10^(-SNR_dB/10);  % Convert SNR to linear scale
scaled_noise = sqrt(noise_power) * noise;
noisy_signal = input_signal + scaled_noise;

% Filter the noisy signal using the FIR filter
filtered_signal = filter(b, 1, noisy_signal);

% Plot the original signal, noisy signal, and filtered signal
figure;

subplot(3, 1, 1);
plot(t, input_signal);
title('Original Signal');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(3, 1, 2);
plot(t, noisy_signal);
title('Noisy Signal');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(3, 1, 3);
plot(t, filtered_signal);
title('Filtered Signal');
xlabel('Time (s)');
ylabel('Amplitude');




N_audio = 50; % Filter order for the audio

[input_audio, Fs_audio] = audioread('C:/Users/San/Downloads/tv-glitch-6245.mp3'); % Replace 'noisy_audio.wav' with your noisy audio file

% Apply the FIR filter to the noisy audio signal
filtered_audio = filter(b, 1, input_audio);

% Play the original noisy audio
disp('Playing Noisy Audio...');
sound(input_audio, Fs_audio);
pause(length(input_audio)/Fs_audio + 1); % Pause to allow the sound to finish playing

% Play the filtered audio
disp('Playing Filtered Audio...');
sound(filtered_audio, Fs_audio);
pause(length(filtered_audio)/Fs_audio + 1); % Pause to allow the sound to finish playing

% Plot the signals for the audio
t_audio = (0:length(input_audio)-1) / Fs_audio;

figure;
subplot(2, 1, 1);
plot(t_audio, input_audio);
title('Noisy Audio Signal');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(2, 1, 2);
plot(t_audio, filtered_audio);
title('Filtered Audio Signal');
xlabel('Time (s)');
ylabel('Amplitude');

% Calculate the noise removed signal for the audio
noise_removed_audio = input_audio - filtered_audio;


% Frequency response plot of the FIR filter for the audio
figure;
freqz(b, 1, 1024, Fs_audio);
title(['Frequency Response of FIR Filter with' window_choice 'Window']);