function [fitresult, gof] = createFit(theta1, ANGLE)
%CREATEFIT(THETA1,AS_R)
%  Create a fit.
%
%  Data for 'untitled fit 1' fit:
%      X Input: theta1
%      Y Output: AS_R
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 22-Oct-2023 16:27:10


%% Fit: 'untitled fit 1'.
[xData, yData] = prepareCurveData( theta1, ANGLE );

% Set up fittype and options.
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = 0.999993438143731;

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, xData, yData );
legend( h, 'AS_R vs. theta1', 'untitled fit 1', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'theta1', 'Interpreter', 'none' );
ylabel( 'AS_R', 'Interpreter', 'none' );
grid on

