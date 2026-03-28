function y_est = myMeasurementFcn(x_k, u)
    % x_k: 4x1 vector of mu_2 estimates [FL; FR; RL; RR]
    % u: 24x1 multiplexed vector of inputs. 
    
    % --- 1. UNPACK INPUTS (4x1 vectors for each wheel) ---
    Bx    = u(1:4);   % Longitudinal tire stiffness
    By    = u(5:8);   % Lateral tire stiffness
    kappa = u(9:12);  % Longitudinal slip ratio
    alpha = u(13:16); % Lateral slip angle
    Fz    = u(17:20); % Normal load on each tire
    delta = u(21:24); % Steering angle for each wheel
    
    % --- 2. DEFINE CONSTANTS ---
    % Physical parameters (Replace with your actual 14DOF parameters)
    Iz = 2500;   % Yaw moment of inertia (kg*m^2)
    a  = 1.2;    % Distance from CG to front axle (m)
    b  = 1.5;    % Distance from CG to rear axle (m)
    w  = 1.6;    % Track width (m)
    g  = 9.81;   % Gravity (m/s^2)
    
    % Pacejka Constants (Shape & Curvature)
    Cx = 1.65; 
    Cy = 1.30; 
    Ex = 0.01; 
    Ey = 0.01; 
    
    % --- 3. CALCULATE PEAK FACTOR (D) ---
    % D = mu_2 * Fz (calculated element-wise for all 4 wheels)
    Dx = x_k .* Fz; 
    Dy = x_k .* Fz;
    
    % --- 4. PACEJKA MAGIC FORMULA (Vectorized for 4x1) ---
    Fx = Dx .* sin(Cx .* atan(Bx .* kappa - Ex .* (Bx .* kappa - atan(Bx .* kappa))));
    Fy = Dy .* sin(Cy .* atan(By .* alpha - Ey .* (By .* alpha - atan(By .* alpha))));
    
    % --- 5. RESOLVE FORCES TO VEHICLE BODY ---
    Fx_body = Fx .* cos(delta) - Fy .* sin(delta);
    Fy_body = Fx .* sin(delta) + Fy .* cos(delta);
    
    % --- 6. CALCULATE ESTIMATED KINEMATICS (12x1 Output) ---
    % Calculate dynamic corner mass based on normal load
    m_corner = Fz ./ g;
    
    % Localized accelerations (4x1 each)
    ax_est = Fx_body ./ m_corner;
    ay_est = Fy_body ./ m_corner;
    
    % Global yaw moment and acceleration
    Mz = (Fx_body(2) - Fx_body(1))*(w/2) + (Fx_body(4) - Fx_body(3))*(w/2) ...
       + (Fy_body(1) + Fy_body(2))*a - (Fy_body(3) + Fy_body(4))*b;
       
    r_global = Mz / Iz;
    
    % Broadcast global yaw to a 4x1 vector to match the required format
    r_est = [r_global; r_global; r_global; r_global];
    
    % Final 12x1 Measurement Vector: [ax(4x1); ay(4x1); r(4x1)]
    y_est = [ax_est; ay_est; r_est];
end