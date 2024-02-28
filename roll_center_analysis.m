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

tire_width = sla_params.tire.width;
tire_ratio = sla_params.tire.ratio;

lca_length = sla_params.lca.length;
uca_length = sla_params.uca.length;

driver_lca_mount = [sla_params.lca.mount{1} + subframe_width/2, sla_params.lca.mount{2}];
driver_uca_mount = [sla_params.uca.mount{1} + subframe_width/2, sla_params.uca.mount{2}];
passenger_lca_mount = [-subframe_width/2 + sla_params.lca.mount{1}, sla_params.lca.mount{2}];
passenger_uca_mount = [-subframe_width/2 - sla_params.uca.mount{1}, sla_params.uca.mount{2}];


[driver_lca_end, driver_uca_end, passenger_lca_end, passenger_uca_end] = plot_suspension(sla_params, ...
    driver_lca_mount, driver_uca_mount, passenger_lca_mount, passenger_uca_mount, lca_length, uca_length);

inst_roll_center(driver_lca_mount, driver_lca_end, driver_uca_mount, driver_uca_end);

inst_roll_center(passenger_lca_mount, passenger_lca_end, passenger_uca_mount, passenger_uca_end);


function instant_roll_center = inst_roll_center(lca_mount, lca_end, uca_mount, uca_end)
    x = -60:0.01:60;
    linewidth = 1.5;
    lca_x_in = lca_mount(1);
    lca_y_in = lca_mount(2);
    lca_x_out = lca_end(1);
    lca_y_out = lca_end(2);
    lca_coef = polyfit([lca_x_in, lca_x_out], [lca_y_in, lca_y_out], 1);
    y_lca = polyval(lca_coef, x);
    plot(x, y_lca, ':k', LineWidth = linewidth);
    hold on

    uca_x_in = uca_mount(1);
    uca_y_in = uca_mount(2);
    uca_x_out = uca_end(1);
    uca_y_out = uca_end(2);    
    uca_coef = polyfit([uca_x_in, uca_x_out], [uca_y_in, uca_y_out], 1);
    y_uca = polyval(uca_coef, x);
    plot(x, y_uca, ':k', LineWidth = linewidth);
    hold on

    xline(0);
    hold on

    title("Ride Height Instantaneous Roll Centers")

    instant_roll_center = [0, 0];
end



function [driver_lca_end, driver_uca_end, passenger_lca_end, passenger_uca_end] = plot_suspension(sla_params, ...
    driver_lca_mount, driver_uca_mount, passenger_lca_mount, passenger_uca_mount, lca_length, uca_length)

    
    driver_lca_rh_phi = deg2rad(sla_params.ride_height.lca_phi);
    driver_lca_end_x = driver_lca_mount(1) + lca_length*cos(driver_lca_rh_phi);
    driver_lca_end_y = driver_lca_mount(2) + lca_length*sin(driver_lca_rh_phi);
    driver_lca_end  = [driver_lca_end_x, driver_lca_end_y];
    

    driver_uca_rh_phi = deg2rad(sla_params.ride_height.uca_phi);
    driver_uca_end_x = driver_uca_mount(1) + uca_length*cos(driver_uca_rh_phi);
    driver_uca_end_y = driver_uca_mount(2) + uca_length*sin(driver_uca_rh_phi);
    driver_uca_end = [driver_uca_end_x, driver_uca_end_y];
    
    
    passenger_lca_rh_phi = deg2rad(sla_params.ride_height.lca_phi);
    passenger_lca_end_x = passenger_lca_mount(1) - lca_length*cos(passenger_lca_rh_phi);
    passenger_lca_end_y = passenger_lca_mount(2) - lca_length*sin(passenger_lca_rh_phi);
    passenger_lca_end  = [passenger_lca_end_x, passenger_lca_end_y];
    
    
    passenger_uca_rh_phi = deg2rad(sla_params.ride_height.uca_phi);
    passenger_uca_end_x = passenger_uca_mount(1) + uca_length*cos(pi - passenger_uca_rh_phi);
    passenger_uca_end_y = passenger_uca_mount(2) + uca_length*sin(pi - passenger_uca_rh_phi);
    passenger_uca_end = [passenger_uca_end_x, passenger_uca_end_y];
    
    
    plot([driver_lca_mount(1), driver_lca_end_x], [driver_lca_mount(2), driver_lca_end_y], 'b', LineWidth = 2)
    hold on
    plot([driver_uca_mount(1), driver_uca_end_x], [driver_uca_mount(2), driver_uca_end_y], 'r', LineWidth = 2)
    hold on
    plot([driver_lca_end_x, driver_uca_end_x], [driver_lca_end_y, driver_uca_end_y], 'k', LineWidth = 2)
    hold on
    plot([passenger_lca_mount(1), passenger_lca_end_x], [passenger_lca_mount(2), passenger_lca_end_y], 'b', LineWidth = 2)
    hold on
    plot([passenger_uca_mount(1), passenger_uca_end_x], [passenger_uca_mount(2), passenger_uca_end_y], 'r', LineWidth = 2)
    hold on
    plot([passenger_lca_end_x, passenger_uca_end_x], [passenger_lca_end_y, passenger_uca_end_y], 'k', LineWidth = 2)
    hold on
    xlim([passenger_lca_end_x - 10, driver_lca_end_x + 10]);
    ylim([passenger_lca_end_x - 10, driver_lca_end_x + 10]);


end 
