%% Trajectory test
% visualization
visualize = 0;

% Serial link object
robot = actual;

% Home position
q_home = [0, 0, 0, 0, 0, 0];

%% #####################################################

%% Pick
% Approach pick
T_home = direct_kine(dh, q_home);

% Calculating T_peack
% T_pick_approach desired
% Remove the translation of T_home and substitute it with desired one
T_home(1:3, 4) = zeros(3, 1);
T_pick_app = translate([19.5 0 7])*T_home*RPY2T([0, -pi/2, 0]);
%T_pick_app = translate([19, 15, 7])*T_home*RPY2T([0, -pi/2, 0]); 
%T_pick_app = translate([13, 14, 7])*T_home*RPY2T([0, -pi/2, 0]); 

% Get q for approaching pick
q_pick_app = inv_kine(T_pick_app, dh, q_home);

% Trajectory
t = get_time(q_home, q_pick_app);
% q_home joint angles for home position; q_pick_app is joint angles for
% pick_approach position (q1 ... q6) and t is a time vector
[Q_pick_app, dQ_pick_app] = joint_traj(@LSPB, q_home, q_pick_app, t);

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
T_pick = translate([19.5 0 1.5])*T_temp*RPY2T([0, 0, 0]);
%T_pick = translate([13, 14, 1.5])*T_temp*RPY2T([0, 0, 0]);
[Q_pick, dQ_pick] = task_traj(@LSPB, T_pick_app, T_pick);

%% Retract back to the approach pick position
%{
    % Joint space
    t = get_time(q_pick, q_pick_app);
    Q_retract_back_pick = joint_traj(@LSPB, q_pick, q_pick_app, t);
%}
% Task space
[Q_retract_back_pick, dQ_retract_back_pick] = task_traj(@LSPB, T_pick, T_pick_app);

%% Place
% Approach place
% T_place_approach desired
T_pick_app(1:3, 4) = zeros(3, 1);
%T_place_app = translate([19 0 7])*T_pick_app*RPY2T([0, 0, 0]);
T_place_app = translate([23 13.5 7])*T_pick_app*RPY2T([0, 0, 0]);

% Compute q_place_approach desired
q_place_app = inv_kine(T_place_app, dh, q_home);

% Trajectory
t = get_time(q_pick_app, q_place_app);
[Q_place_app, dQ_place_app] = joint_traj(@LSPB, q_pick_app, q_place_app, t);
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
%T_place = translate([19 0 1.5])*T_temp*RPY2T([0, 0, 0]);
T_place = translate([23 13.5 1.5])*T_temp*RPY2T([0, 0, 0]);
[Q_place, dQ_place] = task_traj(@LSPB, T_place_app, T_place);

%% Retract to approach place
%{
    % Joint sace
    t = get_time(q_place, q_place_app);
    Q_retract_back_place = joint_traj(@LSPB, q_place, q_place_app, t);
%}
[Q_retract_back_place, dQ_retract_back_place] = task_traj(@LSPB, T_place, T_place_app);
%% Go back to home position
% Get_time
t = get_time(q_place_app, q_home);
[Q_home, dQ_home] = joint_traj(@LSPB, q_place_app, q_home, t);

% Combine all the trajectory points
Q_trajectory = [Q_pick_app; Q_pick; Q_retract_back_pick;
    Q_place_app; Q_place; Q_retract_back_place;
    Q_home];
% Combine all velocity points
dQ_trajectory = [dQ_pick_app; dQ_pick; dQ_retract_back_pick;
    dQ_place_app; dQ_place; dQ_retract_back_place;
    dQ_home];
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

%% Hardware
% Transform the joint angles and velocities to the manipulator unit
Qm_trajectory = traj_pos_joint2motor(Q_trajectory);
dQm_trajectory = traj_vel_joint2motor(dQ_trajectory);

% delay
d = 1;

%% Open port
port_num = open_port;

%% Enable torque
for DXL_ID = 1:7
    enable_torque(DXL_ID, port_num);
end

%% Send to hardware
% Home pose
qm_home = [2033;
           2053;
           1010;
           500 ;
           510 ;
           507
            ];

% Velocity to go to home
vel_home = 60*ones(1, 6);
write_data(qm_home', vel_home, port_num);
%write_pos_and_vel(qm_home', vel_home, port_num);
pause(1)

% Approach pick
Qm_pick_app = traj_pos_joint2motor(Q_pick_app);
dQm_pick_app = traj_vel_joint2motor(dQ_pick_app);
%dQm_pick_app = 60*ones(size(dQ_pick_app));
% Send velocity and trajectory to the manipulator
sync_send(Qm_pick_app, dQm_pick_app, port_num);
pause(d);

% Open gripper
gripper('open', port_num);
pause(d);

% Go to Pick
Qm_pick = traj_pos_joint2motor(Q_pick);
dQm_pick = traj_vel_joint2motor(dQ_pick);
%dQm_pick = 20*ones(size(dQ_pick));
% Send velocity and trajectory to the manipulator
sync_send(Qm_pick, dQm_pick, port_num);
pause(d);

% Close gripper
gripper('close', port_num);
pause(d);

% Retract back to pre-pick
Qm_retract_back_pick = traj_pos_joint2motor(Q_retract_back_pick);
dQm_retract_back_pick = traj_vel_joint2motor(dQ_retract_back_pick);
%dQm_retract_back_pick = 20*ones(size(dQ_retract_back_pick));
sync_send(Qm_retract_back_pick, dQm_retract_back_pick, port_num);
pause(d);

% Place approach
Qm_place_app = traj_pos_joint2motor(Q_place_app);
dQm_place_app = traj_vel_joint2motor(dQ_place_app);
%dQm_place_app = 60*ones(size(dQ_place_app));
% Send velocity and trajectory to the manipulator
sync_send(Qm_place_app, dQm_place_app, port_num);
pause(d);

% Go to place
Qm_place = traj_pos_joint2motor(Q_place);
dQm_place = traj_vel_joint2motor(dQ_place);
%dQm_place = 20*ones(size(dQ_place));
sync_send(Qm_place, dQm_place, port_num);
pause(d);

% Open gripper
gripper('open', port_num);
pause(d);

% Retract back to approach place
Qm_retract_back_place = traj_pos_joint2motor(Q_retract_back_place);
dQm_retract_back_place = traj_vel_joint2motor(dQ_retract_back_place);
%dQm_retract_back_place = 20*ones(size(dQ_retract_back_place));
sync_send(Qm_retract_back_place, dQm_retract_back_place, port_num);
pause(d);

% Close gripper
gripper('close', port_num);
pause(d);

% Go to home pose
Qm_home = traj_pos_joint2motor(Q_home);
dQm_home = traj_vel_joint2motor(dQ_home);
%dQm_home = 60*ones(size(dQ_home));
sync_send(Qm_home, dQm_home, port_num);
pause(d);


% Option to disaple torque
if strcmp(input('Disable torque? (y/n): ', 's'), 'y')
    for DXL_ID = 1:7
        disable_torque(DXL_ID, port_num);
    end
end

% close port
close_port(port_num);
