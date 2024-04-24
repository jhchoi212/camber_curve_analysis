% Non Parallel SLA Camber curve analysis tool by Jonathan Choi, 2/18/2024
% This program is being used for a 1972 Datsun 240z suspension redesign
% Given the mounting points of the lower control arm (lca), upper control arm (uca), lengths of the lca and uca, 
% and the spindle length, we can determine the camber curves for Short Long Arm Suspension

clear; clf; echo off; close all;

% Load in the Params
param_file_name = "sla_params_config.yaml";
sla_params = yaml.loadFile(param_file_name);

% Spingle measurement

spindle_length = sla_params.uprights.spindle_length;
                                                                                        
% LCA mount, length and angle of operation
% LCA mount is the origin and ride heig
% ht is equal to a horizontal LCA
lca_mount = [sla_params.lca.mount{1}, sla_params.lca.mount{2}];
lca_length = sla_params.lca.length;
lca_phi = deg2rad((sla_params.lca.phi{1}:sla_params.lca.phi{2}:sla_params.lca.phi{3}));
rh_lca_phi = deg2rad(sla_params.ride_height.lca_phi);

% UCA mount, length, and angle of operation as well as the full radius
uca_mount = [sla_params.uca.mount{1}, sla_params.uca.mount{2}];
uca_length = sla_params.uca.length;
uca_phi = deg2rad((sla_params.uca.phi{1}:sla_params.uca.phi{2}:sla_params.uca.phi{3}));



camber_curve(lca_mount, lca_length, lca_phi, rh_lca_phi, uca_mount, uca_length, uca_phi, spindle_length, sla_params, param_file_name);


function matched_points = camber_curve(lca_mount, lca_length, lca_phi, rh_lca_phi, uca_mount, uca_length, uca_phi, spindle_length, sla_params, param_file_name)
    

    [lca_mount_polar_phi, lca_mount_polar_rho] = cart2pol(lca_mount(1), lca_mount(2));
    lca_full = deg2rad((0:0.1:360));
    [uca_mount_polar_phi, uca_mount_polar_rho] = cart2pol(uca_mount(1), uca_mount(2));
    uca_full = deg2rad((0:0.1:360));


    % Translating the UCA sweep to be plotted along with the LCA sweep
    [uca_r_trans, uca_phi_trans] = transform_origin(uca_length, uca_phi, uca_mount);
    [uca_r_trans_full, uca_phi_trans_full] = transform_origin(uca_length, uca_full, uca_mount);

    
    % Find the Matching points in the LCA sweep and UCA sweep such that the
    % distance between them is <= 0.001 inches
    matched_points = ca_distance(lca_phi, lca_length, uca_phi_trans, uca_r_trans, spindle_length);
    sla_params.camber.front.matched_points = matched_points;
    
    % Using the matched points, calculate the camber change and ride height
    % deviation
    camber = calculate_camber(matched_points);
    sla_params.camber.front.camber_data = camber;

    % Find UCA Ride height angle for later calculations
    rh_uca_phi = find_rh_uca_phi(rh_lca_phi, lca_length, uca_phi_trans, uca_r_trans, spindle_length, uca_mount);
    sla_params.ride_height.uca_phi = rh_uca_phi;

    

    pos1 = [0.1 0.3 0.4 0.46];
    subplot('Position',pos1);
    % Plot the resulting camber curves, ride height deviation [in] as a function of
    % camber change [deg]
    plot(camber(:, 1), camber(:, 2), LineWidth=2);
    xlabel("Camber [deg]");
    ylabel("Ride Height Deviation [in]");
    yline(0);
    title("Non Equal SLA Camber Curve");
    
    pos2 = [0.5 0.3 0.46 0.46];
    subplot('Position',pos2);
    % Polar plots for LCA and UCA sweep
    polarplot(lca_phi, lca_length*ones(1, length(lca_phi)), LineWidth=5, Color="r");
    hold on
    polarplot(lca_full, lca_length*ones(1, length(lca_full)), ':', LineWidth=1.5, Color=[255/256 165/256 0]);
    hold on
    polarplot(uca_phi_trans, uca_r_trans, LineWidth=5, Color="b");
    hold on
    polarplot(uca_phi_trans_full, uca_r_trans_full, ':', LineWidth=1.5, Color=[92/256 191/256 240/256]);
    title("Sweep of LCA and UCA")
    polarplot(lca_mount_polar_phi, lca_mount_polar_rho, '*', LineWidth = 5, Color = 'r');
    hold on
    polarplot(uca_mount_polar_phi, uca_mount_polar_rho, '*', LineWidth = 5, Color = 'b');
    hold on
    
    formatSpec = 'LCA [Length, Mount]:[%d, (%d, %d)]. UCA [Length, Mount]:[%d, (%d, %d)]. Spindle: %d';
    lca_mount(1);
    title_subplot = sprintf(formatSpec, lca_length, lca_mount(1), lca_mount(2), uca_length, uca_mount(1), uca_mount(2), spindle_length);
    sgtitle(title_subplot);

    % Save matched points and camber data to yaml file for further analysis
    yaml.dumpFile(param_file_name, sla_params);

end



function [r_transform, phi_transform, x, y] = transform_origin(length, phi, mount_points)
    % Perform the translation of the UCA fromt he origin to the UCA mount
    % point
    x = length*cos(phi) + mount_points(1);
    y = length*sin(phi) + mount_points(2);
    r_transform = hypot(y, x);
    phi_transform = atan2(y, x);
end



function matched_points = ca_distance(lca_phi, lca_length, uca_phi_trans, uca_r_trans, spindle_length)
    matched_points = [];
    % Iterate over all LCA sweep angles
    for i = 1:length(lca_phi)
        % For each LCA angle check every UCA sweep angle
        for j = 1:length(uca_phi_trans)
            % Utilizing the polar distance equation
            r_1 = lca_length;
            r_2 = uca_r_trans(j);
            d_theta = lca_phi(i) - uca_phi_trans(j);
            alpha = 2*r_1*r_2;
            dist = (r_1^2 + r_2^2 -alpha*cos(d_theta))^0.5;
            
            % Conditional to check if the UCA sweep point and LCA sweep
            % point are within a tolerance of the spindle length
            if abs(dist - spindle_length) <= 0.001
                % Convert to cartesian for later calculation
                [lca_x, lca_y] = pol2cart(lca_phi(i), lca_length);
                [uca_x, uca_y] = pol2cart(uca_phi_trans(j), uca_r_trans(j));
                matched_points = [
                    matched_points; 
                    [[lca_x, lca_y], [uca_x, uca_y]]
                    ]; %#ok<AGROW> 
            end
        end
    end
end


function camber = calculate_camber(matched_points)
    % Calculate camber with matched points on the LCA and UCA sweep paths
    camber = [];
    
    for i = 1:length(matched_points)
        d_x = (matched_points(i, 1) - matched_points(i, 3));
        d_y = abs(matched_points(i, 2) - matched_points(i, 4));
        angle = -rad2deg(atan(d_x/d_y));        % Camber is measured from 90 degrees where ccw motion is negative and cw is positive
        [lca_deviation_phi, lca_length] = cart2pol(matched_points(i, 1), matched_points(i, 2));
        ride_height_deviation = lca_length*asin(lca_deviation_phi);     % Calculate ride height deviation in inches
        camber = [camber; [angle, ride_height_deviation]]; %#ok<AGROW> 
    end
end

function rh_uca_phi = find_rh_uca_phi(rh_lca_phi, lca_length, uca_phi_trans, uca_r_trans, spindle_length, uca_mount)
    rh_uca_phi = NaN;
    for i = 1:length(uca_phi_trans)
        r_1 = lca_length;
        r_2 = uca_r_trans(i);
        d_theta = rh_lca_phi - uca_phi_trans(i);
        alpha = 2*r_1*r_2;
        dist = (r_1^2 + r_2^2 -alpha*cos(d_theta))^0.5;
        
        % Conditional to check if the UCA sweep point and LCA sweep
        % point are within a tolerance of the spindle length
        if abs(dist - spindle_length) <= 0.01
            % Convert to cartesian for later calculation
            [uca_x, uca_y] = pol2cart(uca_phi_trans(i), uca_r_trans(i));
            rh_uca_phi = rad2deg(atan2(abs(uca_mount(2) - uca_y), abs(uca_mount(1) - uca_x)));
        end
    end
end
