function test_trajectory(space, varargin)
% This function test the joint space if space = "j" and task space if
% space = "t". It accepts additional boolean argument for visualization

% Check varargin
switch length(varargin)
    case 0
        visualize = false;
    case 1
        visualize = varargin{1};
    otherwise
        error("Wrong number of additional arguments provided: %d\n", length(varargin))
end

%% Trajectory test
% Serial link object
robot = actual;

% Home position
q_home = [0, 0, 0, 0, 0, 0];

%% #####################################################
%             JOINT SPACE TRAJECTORY TEST
if strcmp(space, "j")
    %% Pick
    % Approach pick
    T_home = direct_kine(dh, q_home);

    % Calculating T_peack
    % T_pick_approach desired
    % Remove the translation of T_home and substitute it with desired one
    T_home(1:3, 4) = zeros(3, 1);
    % T_pick_app = translate([8 -26 7])*T_home*RPY2T([0, -pi/2, 0]);
    T_pick_app = translate([19, 15, 7])*T_home*RPY2T([0, -pi/2, 0]);

    % Get q for approaching pick
    q_pick_app = inv_kine(T_pick_app, dh, q_home);

    % Trajectory
    t = get_time(q_home, q_pick_app);
    % q_home joint angles for home position; q_pick_app is joint angles for
    % pick_approach position (q1 ... q6) and t is a time vector
    Q_pick_app = joint_traj(@LSPB, q_home, q_pick_app, t);

    %{
    % Joint space
    % T_pick desired
    T_pick_app(1:3, 4) = zeros(3, 1);
    T_pick = translate([19, 15, 2])*T_pick_app*RPY2T([0, 0, 0]);

    % Calculate q for pick
    q_pick = inv_kine(T_pick, dh, q_home);

    % Trajectory
    % get time
    t = get_time(q_pick_app, q_pick);
    Q_pick = joint_traj(@LSPB, q_pick_app, q_pick, t);
    %}

    % Task space
    T_temp = T_pick_app;
    T_temp(1:3, 4) = zeros(3, 1);
    T_pick = translate([19, 15, 3])*T_temp*RPY2T([0, 0, 0]);
    Q_pick = task_traj(@LSPB, T_pick_app, T_pick);

    %% Retract back to the approach pick position
    %{
    % Joint space
    t = get_time(q_pick, q_pick_app);
    Q_retract_back_pick = joint_traj(@LSPB, q_pick, q_pick_app, t);
    %}
    % Task space
    Q_retract_back_pick = task_traj(@LSPB, T_pick, T_pick_app);

    %% Place
    % Approach place
    % T_place_approach desired
    T_pick_app(1:3, 4) = zeros(3, 1);
    T_place_app = translate([19.5 0 7])*T_pick_app*RPY2T([0, 0, 0]);

    % Compute q_place_approach desired
    q_place_app = inv_kine(T_place_app, dh, q_home);

    % Trajectory
    t = get_time(q_pick_app, q_place_app);
    Q_place_app = joint_traj(@LSPB, q_pick_app, q_place_app, t);
    %{
    % Joint space
    % T_place_desired
    T_place_app(1:3, 4) = zeros(3, 1);
    T_place = translate([19.5 0 2])*T_place_app*RPY2T([0, 0, 0]);

    % Calculate q for place
    q_place = inv_kine(T_place, dh, q_home);

    % Trajectory
    t = get_time(q_place_app, q_place);
    Q_place = joint_traj(@LSPB, q_place_app, q_place, t);
    %}
    % Task space
    T_temp = T_place_app;
    T_temp(1:3, 4) = zeros(3, 1);
    T_place = translate([19.5 0 2])*T_temp*RPY2T([0, 0, 0]);
    Q_place = task_traj(@LSPB, T_place_app, T_place);

    %% Retract to approach place
    %{
    % Joint sace
    t = get_time(q_place, q_place_app);
    Q_retract_back_place = joint_traj(@LSPB, q_place, q_place_app, t);
    %}
    Q_retract_back_place = task_traj(@LSPB, T_place, T_place_app);
    %% Go back to home position
    % Get_time
    t = get_time(q_place_app, q_home);
    Q_home = joint_traj(@LSPB, q_place_app, q_home, t);

    % Combine all the trajectory points
    Q_trajectory = [Q_pick_app; Q_pick; Q_retract_back_pick;
        Q_place_app; Q_place; Q_retract_back_place;
        Q_home];
    %% Visualize
    if visualize
        % Initialize the end-effector positions
        endEffectorPositions = zeros(length(Q_trajectory), 3);

        % Get the end-effector positions for each time step
        for i = 1:numrows(Q_trajectory)
            % Get end-effector position at current time step
            T = robot.fkine(Q_trajectory(i, :));
            T = double(T);
            endEffectorPosition = T(1:3, 4);

            % Store the end-effector position
            endEffectorPositions(i, :) = endEffectorPosition;
        end

        % Visualize robot configurations and plot end-effector trajectory as a red line
        plot3(endEffectorPositions(:, 1), endEffectorPositions(:, 2), endEffectorPositions(:, 3), 'r-', 'LineWidth', 2);
        hold on;
        robot.plot(Q_trajectory);
        hold off;
    end

elseif strcmp(space, "t")
    %% ####################################################
    %                 TASK SPACE TRAJECTORY TEST
    % We want to create a right angle triangle 
    T_home = direct_kine(dh, q_home);

    % Corner 1
    T_home(1:3, 4) = zeros(3, 1);
    T_corner_1 = translate([10, 0, 0])*T_home*RPY2T([0, -pi/2, 0]);
    q1 = inv_kine(T_corner_1, dh, q_home);

    % Corner 2
    T_temp = T_corner_1;
    T_temp(1:3, 4) = zeros(3, 1);
    T_corner_2 = translate([10, 20, 0])*T_temp*RPY2T([0, 0, 0]);
    q2 = inv_kine(T_corner_2, dh, q_home);

    % Corner 3
    T_temp = T_corner_2;
    T_temp(1:3, 4) = zeros(3, 1);
    T_corner_3 = translate([20, 0, 0])*T_temp*RPY2T([0, 0, 0]); 
    q3 = inv_kine(T_corner_3, dh, q_home);

    % Task space trajectory from corner 1 to corner 2
    Q_12 = task_traj(@LSPB, T_corner_1, T_corner_2);

    % Task space trajectory from corner 2 to corner 3
    Q_23 = task_traj(@LSPB, T_corner_2, T_corner_3);

    % Task space trajectory from corner 3 to corner 1
    Q_31 = task_traj(@LSPB, T_corner_3, T_corner_1);

    % Combine all of them to one trajectory
    Q_task = [Q_12; Q_23; Q_31];

    % The same trajectory in joint space
    Q_joint = [joint_traj(@LSPB, q1, q2, get_time(q1, q2));
               joint_traj(@LSPB, q2, q3, get_time(q2, q3));
               joint_traj(@LSPB, q3, q1, get_time(q3, q1))];

    if visualize
        % Initialize the end-effector positions
        endEffectorPositions_j = zeros(length(Q_task), 3);

        % Get the end-effector positions for each time step
        for i = 1:numrows(Q_task)
            % Get end-effector position at current time step
            T = robot.fkine(Q_task(i, :));
            T = double(T);
            endEffectorPosition = T(1:3, 4);

            % Store the end-effector position
            endEffectorPositions_j(i, :) = endEffectorPosition;
        end

        % Initialize the end-effector positions
        endEffectorPositions_t = zeros(length(Q_joint), 3);
        for i = 1:numrows(Q_joint)
            % Get end-effector position at current time step
            T = robot.fkine(Q_joint(i, :));
            T = double(T);
            endEffectorPosition = T(1:3, 4);

            % Store the end-effector position
            endEffectorPositions_t(i, :) = endEffectorPosition;
        end

        % Visualize robot configurations and plot end-effector trajectory
        plot3(endEffectorPositions_t(:, 1), endEffectorPositions_t(:, 2), endEffectorPositions_t(:, 3), 'b-', 'LineWidth', 2);
        hold on;
        plot3(endEffectorPositions_j(:, 1), endEffectorPositions_j(:, 2), endEffectorPositions_j(:, 3), 'r-', 'LineWidth', 2);
        robot.plot(Q_task);
        hold off;
    end

end
