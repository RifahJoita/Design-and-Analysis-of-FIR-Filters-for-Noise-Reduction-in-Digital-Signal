# Design-and-Analysis-of-FIR-Filters-for-Noise-Reduction-in-Digital-Signal
Designing FIR filters and analyzing the frequency responses of noisy and enhanced samples using Matlab
This project demonstrates the design, analysis, and application of Finite Impulse Response (FIR) filters for noise suppression in both synthesized signals and audio files.
It involves:
1. Designing an FIR low-pass filter using different window functions based on user specifications.
2. Analyzing frequency responses, step and impulse responses.
3. Filtering noisy signals and comparing original vs. enhanced signals.
4. Filtering a real-world noisy audio file and evaluating performance.

Design specifications:
Sampling Frequency (fs): 20,000 Hz
Filter Order (n): 50
Passband Ripple: 1 dB
Stopband Attenuation: 50 dB
Passband Frequency: 4500 Hz
Stopband Frequency: 5000 Hz
The filter coefficients are calculated based on the ideal sinc function and modified using an appropriate window function,
Window Choices: Rectangular, Hanning, Hamming, Blackman
