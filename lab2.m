
clc; clear; close all;
%% Exercise 1: Right-Hand Rule Visualization
    % % origin
    % origin = [0 0 0];
    % 
    % x2_dir = [1 0 0];  
    % y2_dir = [0 0 -1];  
    % 
    % % Calculate Z using the Right-Hand Rule
    % z2_dir = cross(x2_dir, y2_dir);
    % 
    % % Plot Frame A1
    % figure('Name', 'Exercise 1 - Frame A1');
    % quiver3(origin(1), origin(2), origin(3), x2_dir(1), x2_dir(2), x2_dir(3), 'r', 'LineWidth', 2, 'DisplayName', 'X2'); hold on;
    % quiver3(origin(1), origin(2), origin(3), y2_dir(1), y2_dir(2), y2_dir(3), 'g', 'LineWidth', 2, 'DisplayName', 'Y2');
    % quiver3(origin(1), origin(2), origin(3), z2_dir(1), z2_dir(2), z2_dir(3), 'b', 'LineWidth', 2, 'DisplayName', 'Z2 (Computed)');
    % axis equal; grid on; legend;
    % title('Frame A1: Z = cross(X, Y)');
    % xlabel('X'); ylabel('Y'); zlabel('Z');
    % 
    % fprintf('Exercise 1: For the given X and Y, the Z axis is: [%d %d %d]\n', z2_dir);
%% Exercise 3: Frame E Relative to Frame A
    % Defining Cuboid Dimensions and Vertices
    % L = 7; % Along X-axis
    % W = 4; % Along Y-axis
    % H = 1.5; % Along Z-axis
    % 
    % % Defining the 8 vertices & faces of the cuboid
    % vertices = [
    %     0 0 0;  % 1: a (Origin of Frame A)
    %     L 0 0;  % 2
    %     L W 0;  % 3
    %     0 W 0;  % 4
    %     0 0 H;  % 5
    %     L 0 H;  % 6
    %     L W H;  % 7: e (Origin of Frame E)
    %     0 W H   % 8
    % ];
    % 
    % faces = [
    %     1 2 6 5; % Front
    %     2 3 7 6; % Right
    %     3 4 8 7; % Back
    %     4 1 5 8; % Left
    %     1 2 3 4; % Bottom
    %     5 6 7 8  % Top
    % ];
    % 
    % % 2. Sketch the Cuboid
    % figure('Name', 'Exercise 3: Cuboid and Frame E', 'Color', 'w');
    % patch('Vertices', vertices, 'Faces', faces, ...
    %       'FaceColor', 'cyan', 'FaceAlpha', 0.1, 'EdgeColor', 'k', 'LineWidth', 1.5);
    % hold on;
    % axis equal; grid on;
    % xlabel('X_A'); ylabel('Y_A'); zlabel('Z_A');
    % view(3); % Set 3D view
    % 
    % % Label Points 'a' and 'e'
    % text(0, 0, 0, '  a (Frame A)', 'FontSize', 12, 'FontWeight', 'bold');
    % text(L, W, H, '  e (Frame E)', 'FontSize', 12, 'FontWeight', 'bold');
    % 
    % % Plot Frame A Axes (World Frame) at (0,0,0)
    % quiver3(0,0,0, 1.5,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % X_A
    % quiver3(0,0,0, 0,1.5,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Y_A
    % quiver3(0,0,0, 0,0,1.5, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Z_A
    % 
    % %3. Assign and Plot Frame E Axes (Right-Hand Rule)
    % origin_E = [L, W, H];
    % 
    % e_x_dir = [0, -1, 0]; 
    % e_y_dir = [-1, 0, 0];
    % e_z_dir = cross(e_x_dir, e_y_dir); % Calculates Z to ensure Right-Hand Rule
    % 
    % % Scale factor for arrows
    % scale = 1.5;
    % 
    % % Ploting E Axes
    % quiver3(origin_E(1), origin_E(2), origin_E(3), ...
    %         e_x_dir(1)*scale, e_x_dir(2)*scale, e_x_dir(3)*scale, ...
    %         'r', 'LineWidth', 3, 'MaxHeadSize', 0.5); % e_x (Red)
    % 
    % quiver3(origin_E(1), origin_E(2), origin_E(3), ...
    %         e_y_dir(1)*scale, e_y_dir(2)*scale, e_y_dir(3)*scale, ...
    %         'g', 'LineWidth', 3, 'MaxHeadSize', 0.5); % e_y (Green)
    % 
    % quiver3(origin_E(1), origin_E(2), origin_E(3), ...
    %         e_z_dir(1)*scale, e_z_dir(2)*scale, e_z_dir(3)*scale, ...
    %         'b', 'LineWidth', 3, 'MaxHeadSize', 0.5); % e_z (Blue)
    % 
    % % Adding Labels for E Axes
    % text(origin_E(1)+e_x_dir(1)*scale, origin_E(2)+e_x_dir(2)*scale, origin_E(3)+e_x_dir(3)*scale, ' e_x');
    % text(origin_E(1)+e_y_dir(1)*scale, origin_E(2)+e_y_dir(2)*scale, origin_E(3)+e_y_dir(3)*scale, ' e_y');
    % text(origin_E(1)+e_z_dir(1)*scale, origin_E(2)+e_z_dir(2)*scale, origin_E(3)+e_z_dir(3)*scale, ' e_z');
    % 
    % title('Exercise 3: Frame A vs Frame E');
    % hold off;
    % % Constructing and Printing the Matrix
    % R_AE = [e_x_dir(:), e_y_dir(:), e_z_dir(:)]; 
    % 
    % % Position Vector (Transposed to make it a column)
    % P_AE = origin_E(:);
    % 
    % % Formula: T = [ R, P ; 0 0 0 1 ]
    % T_AE = [R_AE, P_AE; 0 0 0 1];
    % 
    % disp('The Homogeneous Transformation Matrix (Frame A to Frame E) is:');
    % disp(T_AE);
