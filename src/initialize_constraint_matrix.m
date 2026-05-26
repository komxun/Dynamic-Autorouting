function [weatherMat, weatherMatMod, WMCell, dwdxCell, dwdyCell] = ...
    initialize_constraint_matrix(matFile, B_L, B_U)
% INITIALIZE_CONSTRAINT_MATRIX  Load weather data and build interpolants.
%
%   [weatherMat, weatherMatMod, WMCell, dwdxCell, dwdyCell] = ...
%       initialize_constraint_matrix(matFile, B_L, B_U)
%
%   Inputs:
%     matFile  - path to a .mat file containing variable 'weatherMat'
%     B_L      - lower bound for constraint matrix filtering
%     B_U      - upper bound for constraint matrix filtering
%
%   Outputs:
%     weatherMat    - raw NxNxT weather matrix
%     weatherMatMod - filtered (clamped) weather matrix
%     WMCell        - 1xT cell of griddedInterpolant for each time step
%     dwdxCell      - 1xT cell of griddedInterpolant (x-gradient)
%     dwdyCell      - 1xT cell of griddedInterpolant (y-gradient)

    S = load(matFile, 'weatherMat');
    weatherMat = S.weatherMat;

    % Clamp to [B_L, 1]
    weatherMatMod = weatherMat;
    weatherMatMod(weatherMatMod < B_L) = B_L;
    weatherMatMod(weatherMatMod > B_U) = 1;

    % Build gridded interpolants and gradient interpolants
    xspace = 1:200;
    yspace = 1:200;
    [xgrid, ygrid] = ndgrid(xspace, yspace);

    nT = size(weatherMat, 3);
    WMCell   = cell(1, nT);
    dwdxCell = cell(1, nT);
    dwdyCell = cell(1, nT);

    for j = 1:nT
        WMCell{j} = griddedInterpolant(weatherMat(:,:,j)');
        z_values = WMCell{j}(xgrid, ygrid);
        [grad_y, grad_x] = gradient(z_values, yspace, xspace);
        dwdxCell{j} = griddedInterpolant(grad_x);
        dwdyCell{j} = griddedInterpolant(grad_y);
    end
end