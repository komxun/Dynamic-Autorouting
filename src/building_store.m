function B = building_store(newB)
% BUILDING_STORE  Persistent cache for the GeoJSON-derived building boxes.
%
%   building_store(B)  stores the building struct array B (called once by
%                      setup_simulation for the GeoJSON scene).
%   B = building_store()  retrieves the cached building struct array (used
%                         inside create_scene, which has a fixed signature).
%
%   This avoids passing the building geometry through the IFDS call chain
%   or polluting the Object struct with extra fields.

    persistent store
    if nargin >= 1
        store = newB;
    end
    B = store;
end
