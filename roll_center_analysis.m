% Non Parallel SLA roll center analysis tool by Jonathan Choi, 2/20/2024
% This program is being used for a 1972 Datsun 240z suspension redesign
% Given the mounting points of the lower control arm (lca), upper control arm (uca), lengths of the lca and uca, 
% and the spindle length, distance between left lca and right lca mounts,
% we can determine the roll center of a vehicle
clear; clf;

% Load in the Params
param_file_name = "sla_params_config.yaml";
sla_params = yaml.loadFile(param_file_name);


subframe_width = sla_params.subframe_width;
spindle_length = sla_params.uprights.spindle_length;
hub_ride_height = sla_params.uprights.hub_center_from_lca;

% Finding the wheel specs for plotting the Roll Center 
wheel_radius = sla_params.wheel.radius;
wheel_width = sla_params.wheel.width;
wheel_backspace = sla_params.wheel.back_space;

tire_width = sla_params.tire.width;
tire_ratio = sla_params.tire.ratio;
tire_radius = wheel_radius + tire_width*(tire_ratio/100);

% Control Arm Params loaded in
lca_length = sla_params.lca.length;
uca_length = sla_params.uca.length;

driver_lca_mount = [sla_params.lca.mount{1} + subframe_width/2, sla_params.lca.mount{2}];
driver_uca_mount = [sla_params.uca.mount{1} + subframe_width/2, sla_params.uca.mount{2}];
passenger_lca_mount = [-subframe_width/2 + sla_params.lca.mount{1}, sla_params.lca.mount{2}];
passenger_uca_mount = [-subframe_width/2 - sla_params.uca.mount{1}, sla_params.uca.mount{2}];


% Plotting of the LCA, UCA, and Splindle
[driver_lca_end, driver_uca_end, passenger_lca_end, passenger_uca_end, wheel_mounts, upright_phi] = plot_suspension(sla_params, ...
    driver_lca_mount, driver_uca_mount, passenger_lca_mount, passenger_uca_mount, lca_length, uca_length, hub_ride_height);

% Plotting of the virtual arms of the LCA, UCA. Calculate the IC of the given side 
driver_ic = inst_roll_center(driver_lca_mount, driver_lca_end, driver_uca_mount, driver_uca_end, "driver"); 
passenger_ic = inst_roll_center(passenger_lca_mount, passenger_lca_end, passenger_uca_mount, passenger_uca_end, "passenger");


tire_points = create_wheel_tire_array(wheel_width, wheel_backspace, tire_width, tire_radius, wheel_mounts, upright_phi);

% Plot the roll center and the virtual lever arm on which it acts
roll_center_plot(driver_ic, passenger_ic, tire_points, param_file_name, sla_params);

function instant_roll_center = inst_roll_center(lca_mount, lca_end, uca_mount, uca_end, side)
    
    % Changes the range depenging on the driver vs passenger side
    % configuration
    if side == "driver"
        x_lca = -60:0.01:lca_end(1);
        x_uca = -60:0.01:uca_end(1);
    else
        x_lca = lca_end(1):0.01:60;
        x_uca = uca_end(1):0.01:60;
    end

    % Plotting of the LCA virtual lines
    linewidth = 1.5;
    lca_x_in = lca_mount(1);
    lca_y_in = lca_mount(2);
    lca_x_out = lca_end(1);
    lca_y_out = lca_end(2);
    lca_coef = polyfit([lca_x_in, lca_x_out], [lca_y_in, lca_y_out], 1);
    y_lca = polyval(lca_coef, x_lca);
    plot(x_lca, y_lca, ':k', LineWidth = linewidth);


    uca_x_in = uca_mount(1);
    uca_y_in = uca_mount(2);
    uca_x_out = uca_end(1);
    uca_y_out = uca_end(2);    
    uca_coef = polyfit([uca_x_in, uca_x_out], [uca_y_in, uca_y_out], 1);
    y_uca = polyval(uca_coef, x_uca);
    plot(x_uca, y_uca, ':k', LineWidth = linewidth);
    
    % Plotting the center line for finding the roll center
    xline(0);
    title("Ride Height Instantaneous Roll Centers")
    hold on;

    % Finding the point for the instant roll center accounting for slight
    % numerical errors due to lack of resolution
    instant_roll_center = [0, 0];
    for i = 1:length(y_uca)
        if abs(y_uca(i)) < 0.001
            instant_roll_center(1) = x_uca(i);
            instant_roll_center(2) = y_uca(i);
        end
    end


end



function [driver_lca_end, driver_uca_end, passenger_lca_end, passenger_uca_end, wheel_mounts, upright_phi] = plot_suspension(sla_params, ...
    driver_lca_mount, driver_uca_mount, passenger_lca_mount, passenger_uca_mount, lca_length, uca_length, hub_ride_height)

    % Finding the end point of the driver LCA
    driver_lca_rh_phi = deg2rad(sla_params.ride_height.lca_phi);
    driver_lca_end_x = driver_lca_mount(1) + lca_length*cos(driver_lca_rh_phi);
    driver_lca_end_y = driver_lca_mount(2) + lca_length*sin(driver_lca_rh_phi);
    driver_lca_end  = [driver_lca_end_x, driver_lca_end_y];
    
    % Finding the end point of the driver UCA
    driver_uca_rh_phi = deg2rad(sla_params.ride_height.uca_phi);
    driver_uca_end_x = driver_uca_mount(1) + uca_length*cos(driver_uca_rh_phi);
    driver_uca_end_y = driver_uca_mount(2) + uca_length*sin(driver_uca_rh_phi);
    driver_uca_end = [driver_uca_end_x, driver_uca_end_y];
    
    % Finding the end point of the passenger LCA
    passenger_lca_rh_phi = deg2rad(sla_params.ride_height.lca_phi);
    passenger_lca_end_x = passenger_lca_mount(1) - lca_length*cos(passenger_lca_rh_phi);
    passenger_lca_end_y = passenger_lca_mount(2) - lca_length*sin(passenger_lca_rh_phi);
    passenger_lca_end  = [passenger_lca_end_x, passenger_lca_end_y];
    
    % Finding the end point of the passenger UCA    
    passenger_uca_rh_phi = deg2rad(sla_params.ride_height.uca_phi);
    passenger_uca_end_x = passenger_uca_mount(1) + uca_length*cos(pi - passenger_uca_rh_phi);
    passenger_uca_end_y = passenger_uca_mount(2) + uca_length*sin(pi - passenger_uca_rh_phi);
    passenger_uca_end = [passenger_uca_end_x, passenger_uca_end_y];
    
    
    % Finding the position of the hub on the upright wrt the LCA mount
    upright_phi = atan((driver_uca_end_y-driver_lca_end_y)/(driver_lca_end_x-driver_uca_end_x));    
    hub_y = hub_ride_height * sin(upright_phi);
    hub_x = hub_ride_height * cos(upright_phi);

    wheel_mounts = [[driver_lca_end_x - hub_x, driver_lca_end_y + hub_y], [passenger_lca_end_x + hub_x, passenger_lca_end_y + hub_y]];

    % Plot LCA (driver)
    plot([driver_lca_mount(1), driver_lca_end_x], [driver_lca_mount(2), driver_lca_end_y], 'b', LineWidth = 2)

    % Plot UCA (driver)
    plot([driver_uca_mount(1), driver_uca_end_x], [driver_uca_mount(2), driver_uca_end_y], 'r', LineWidth = 2)

    % Plot Upright (driver)
    plot([driver_lca_end_x, driver_uca_end_x], [driver_lca_end_y, driver_uca_end_y], 'k', LineWidth = 2)

    % Plot wheel center (driver)
    plot(wheel_mounts(1), wheel_mounts(2), "*k", LineWidth = 3)

    % Plot LCA (passenger)
    plot([passenger_lca_mount(1), passenger_lca_end_x], [passenger_lca_mount(2), passenger_lca_end_y], 'b', LineWidth = 2)

    % Plot UCA (passenger)
    plot([passenger_uca_mount(1), passenger_uca_end_x], [passenger_uca_mount(2), passenger_uca_end_y], 'r', LineWidth = 2)

    % Plot Upright (passenger)
    plot([passenger_lca_end_x, passenger_uca_end_x], [passenger_lca_end_y, passenger_uca_end_y], 'k', LineWidth = 2)

    % Plot wheel center (passenger)
    plot(wheel_mounts(3), wheel_mounts(4), "*k", LineWidth = 3)

    xlim([passenger_lca_end_x - 10, driver_lca_end_x + 10]);
    ylim([passenger_lca_end_x - 10, driver_lca_end_x + 10]);
    hold on;


end 

function tire_center_points = create_wheel_tire_array(wheel_width, wheel_backspace, tire_width, tire_radius, wheel_mounts, upright_phi)
    
    % Find the components of change of the wheel due to camber
    d_y_wheel = tire_radius * sin(upright_phi);    
    d_x_wheel = tire_radius * cos(upright_phi);
    
    % Find the components of the change in tire due to camber
    d_y_tire = (tire_width/2) * cos(pi-upright_phi);
    d_x_tire = (tire_width/2) * sin(pi-upright_phi);

    % Calcualte the offset
    offset = (wheel_width/2) - wheel_backspace;

    % Calculate the top and bottom points of the driver and passenger wheels
    driver_wheel_top = [wheel_mounts(1) - d_x_wheel, wheel_mounts(2) + d_y_wheel];
    driver_wheel_bottom = [wheel_mounts(1) + d_x_wheel, wheel_mounts(2) - d_y_wheel];
    passenger_wheel_top = [wheel_mounts(3) + d_x_wheel, wheel_mounts(4) + d_y_wheel];
    passenger_wheel_bottom = [wheel_mounts(3) - d_x_wheel, wheel_mounts(4) - d_y_wheel];

    % Calculate the outer and inner points of the tire contact patch for
    % the driver and passenger wheels taking into account the ride height camber
    driver_tire_in = [driver_wheel_bottom(1) + d_x_tire + offset, driver_wheel_bottom(2) - d_y_tire];
    driver_tire_out = [driver_wheel_bottom(1) - d_x_tire + offset, driver_wheel_bottom(2) + d_y_tire];
    passenger_tire_in = [passenger_wheel_bottom(1) + d_x_tire - offset, passenger_wheel_bottom(2) + d_y_tire];
    passenger_tire_out = [passenger_wheel_bottom(1) - d_x_tire - offset, passenger_wheel_bottom(2) - d_y_tire];
    
    driver_center_x = (driver_tire_in(1) + driver_tire_out(1))/2;
    driver_center_y = (driver_tire_in(2) + driver_tire_out(2))/2;
    passenger_center_x = (passenger_tire_in(1) + passenger_tire_out(1))/2;
    passenger_center_y = (passenger_tire_in(2) + passenger_tire_out(2))/2;
    tire_center_points = [driver_center_x, driver_center_y, passenger_center_x, passenger_center_y];


    linewidth = 1.75;
    plot([driver_wheel_top(1), driver_wheel_bottom(1)], [driver_wheel_top(2), driver_wheel_bottom(2)], 'k', LineWidth = linewidth);
    plot([driver_tire_in(1), driver_tire_out(1)], [driver_tire_in(2), driver_tire_out(2)], 'r', LineWidth = linewidth);


    plot([passenger_wheel_top(1), passenger_wheel_bottom(1)], [driver_wheel_top(2), passenger_wheel_bottom(2)], 'k', LineWidth = linewidth);
    plot([passenger_tire_in(1), passenger_tire_out(1)], [passenger_tire_in(2), passenger_tire_out(2)], 'r', LineWidth = linewidth);
    hold on;

end

function roll_center_plot(driver_ic, passenger_ic, tire_points, param_file_name, sla_params)
    linewidth = 5;
    plot(driver_ic(1), driver_ic(2), '*k', LineWidth = linewidth);
    plot(passenger_ic(1), passenger_ic(2), '*k', LineWidth = linewidth);
    
    % Markers for the center of the tires
    plot(tire_points(1), tire_points(2), '*k', LineWidth = linewidth)
    plot(tire_points(3), tire_points(4), '*k', LineWidth = linewidth)
    
    % Plot moment from the tire contact patch to the instant roll centers
    plot([tire_points(1), driver_ic(1)], [tire_points(2), driver_ic(2)], ':k', LineWidth = 1.25)
    plot([tire_points(3), passenger_ic(1)], [tire_points(4), passenger_ic(2)], ':k', LineWidth = 1.25);


    driver_moment_coeff = polyfit([tire_points(1), driver_ic(1)], [tire_points(2), driver_ic(2)], 1);
    driver_moment_x = driver_ic(1):0.1:tire_points(1);
    driver_moment_y = polyval(driver_moment_coeff, driver_moment_x);

    passenger_moment_coeff = polyfit([tire_points(3), passenger_ic(1)], [tire_points(4), passenger_ic(2)], 1);
    passenger_moment_x = tire_points(3):0.1:passenger_ic(1);
    passenger_moment_y = polyval(passenger_moment_coeff, passenger_moment_x);
    [x_rc, y_rc] = polyxpoly(driver_moment_x, driver_moment_y, passenger_moment_x, passenger_moment_y);

    plot(x_rc, y_rc, '*r', LineWidth = linewidth)
    hold on;


    sla_params.roll_center.front = [x_rc, y_rc];
    yaml.dumpFile(param_file_name, sla_params);


end
