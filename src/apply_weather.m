function Object = apply_weather(Object, numObj, k, B_L, B_U, xx, yy, weatherMat, dwdx, dwdy)
% APPLY_WEATHER  Modify Object Gamma and gradients using weather constraints.
%
%   Applies the constraint-matrix coupling to each object's boundary
%   function, augmenting the implicit surface with weather-based penalties.
%
%   Reference: Technical Report Section on Constraint Matrix Integration.

    omega    = weatherMat(xx+1, yy+101);
    dwdx_now = dwdx(xx+1, yy+101);
    dwdy_now = dwdy(xx+1, yy+101);

    for j = 1:numObj
        Gm   = Object(j).Gamma;
        dGdx = Object(j).n(1);
        dGdy = Object(j).n(2);
        dGdz = Object(j).n(3);

        expTerm = exp( (B_L - omega)/(B_L - B_U) * log((Gm-1)/k + 1) );

        dGx_p = dGdx + k * expTerm * ...
            ( log((Gm-1)/k + 1)/(B_L - B_U) * dwdx_now ...
            - ((B_L - omega)/((Gm-1+k)*(B_L - B_U))) * dGdx );

        dGy_p = dGdy + k * expTerm * ...
            ( log((Gm-1)/k + 1)/(B_L - B_U) * dwdy_now ...
            - ((B_L - omega)/((Gm-1+k)*(B_L - B_U))) * dGdy );

        dGz_p = dGdz + k * expTerm * ...
            ( -((B_L - omega)/((Gm-1+k)*(B_L - B_U))) * dGdz );

        Object(j).Gamma = Object(j).Gamma - k * (expTerm - 1);
        Object(j).n = [dGx_p; dGy_p; dGz_p];
        Object(j).t = [dGy_p; -dGx_p; 0];
    end
end
