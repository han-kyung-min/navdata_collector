
function mapped = remap_costmap(final_costmap)
    % Create an empty uint8 array of same size
    mapped = zeros(size(final_costmap), 'uint8');

    % Rule 1: Unknown (-1) --> 128
    mapped(final_costmap == -1) = 128;

    % Rule 2: Free (0) --> 0
    mapped(final_costmap == 0) = 0;

    % Rule 3: 1 ~ 98 (linear scaling)
    for i = 1:98
        original_value = i;
        mapped_value = uint8( ((i - 1) * 251 - 1) / 97 + 1 );
        mapped(final_costmap == original_value) = mapped_value;
    end

    % Rule 4: 99 --> 253
    mapped(final_costmap == 99) = 253;

    % Rule 5: 100 --> 254
    mapped(final_costmap == 100) = 254;
end