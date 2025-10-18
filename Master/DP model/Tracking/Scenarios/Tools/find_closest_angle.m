function shortest_array_angles = find_closest_angle(angle_array)
    
    array_length = size(angle_array,2);
    shortest_array_angles = zeros(1, array_length);

    shortest_array_angles(1) = angle_array(1);

    for i=1:(array_length-1)
        
        current_angle = mod(shortest_array_angles(i), 2*pi);
        next_angle = mod(angle_array(i+1), 2*pi);
        
        % Find shortest delta angle
        shortest_delta_angle = mod(next_angle - current_angle + 2*pi, 2*pi);
        
        if (shortest_delta_angle > pi)
            shortest_delta_angle = 2*pi - shortest_delta_angle;
            new_angle_change = -shortest_delta_angle;
        else
            new_angle_change = shortest_delta_angle;
        end

        shortest_array_angles(i+1) = shortest_array_angles(i) + new_angle_change;
    end
end