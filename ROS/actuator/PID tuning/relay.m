% suppress lines not terminated with semi colon
%#ok<*NOPTS>
% criticalPeriod = 49.445 - 48.639 % left
% criticalPeriod = 49.702 - 48.942 % back
criticalPeriod = 49.96 - 49.196 % right
% amplitudePeakToPeak = 3.46e+02 - -3.46e+02 % leftl
% amplitudePeakToPeak = 3.189e+02 - -3.189e+02 % back
amplitudePeakToPeak = 3.253e+02 - -3.253e+02 % right
hysteresis = 1 - (-1)
limitUpper = 0.02;pi*10
limitLower = -0.02;
criticalGain = (4 * (limitUpper - limitLower)) / (pi * sqrt((amplitudePeakToPeak^2) - (hysteresis^2)))
[(0.5 * criticalGain) 0 0; (0.4 * criticalGain) (0.8 * criticalPeriod) 0; (0.6 * criticalGain) (0.5 * criticalPeriod) (0.125 * criticalPeriod)]
