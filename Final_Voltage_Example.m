 
%Name: Alec Knutsen
%Date:12/08/15
%Description:This Program runs the Discrete Kalman Filter Algorithm for the
%voltage example described in the paper.
%
%
%State Space Model:
    %   Equation 1: x_k+1 = (F_k)(x_k)+ (G_k)(u_k) + w_k
    %   Equation 2: z_k = (H_k)(x_k) + v_k
    %
    % Parameters:
        % x_k - State vector at time k with deminsions nx1
        % u_k - Control input at time k with dimensions mx1
        % z_k - Observation at time k with dimensions px1
        % F_k - State transition matrix at time k with dimensions nxn
        % G_k - Input transition matrix at time k with dimensions nxm
        % H_k - Observation matrix at time k with dimensions pxn
        % w_k - Process noise at time k with dimensions nx1
        % v_k - Additive noise measurement at time k with dimension px1
        % Q_k - Covariance matrix (Q= E[(w_k)(w_k)^T]) at time k with
        % dimensions nxn
        %R_k - Covariance matrix (R_k= E[(v_k)(v_k)^T]) at time k with
        % dimensions pxp
        %P_k - Covariance error matrix(P_k = E[(xhat_k - x_k)(xhat_k-x_k)^T])

%We have the following Kalman Filter Equations:
    %Initial Conditions:
        %xhat_0 = E[x_0]
        %P_0|0 = E[(xhat_0 - x_0)(xhat_0-x_0)^T]
    %Prediction Equations:
        %Equation 3: xhat_k+1|k = (F_k)(xhat_k|k) + (G_k)(u_k)
        %Equation 4: P_k+1|k = (F_k)(P_k|k)(F_k)^T + Q_k
    %Update Equations:
        %Equation 5: xhat_k+1|k+1 = xhat_k+1|k + K_k+1(z_k+1 - (H_k+1))(xhat_k+1|k)
        %Equation 6: P_k+1|k+1 = (I - (K_k+1)(H_k+1))(P_k+1|k)(I - (K_k+1)(H_k+1))^T + (K_k+1)(R_k+1)(K_k+1)^T

% Extra Equations:
    % Kalman Gain: K_k+1 = (P_k+1|k)(H_k+1)^T[(H_k)(P_k+1|k)(H_k)^T +
    %                       R_k+1]^-1



%Beginning of Program:
%Note: Everything in this example is one dimensional. When, comments refer
%to vectors or matrices, everything is a scalar.
   
%User input: n - Size of state vectors
n=1;

%User input: m - Size of input vectors
m=1;

%User input: p - The size of observation vectors
p=1;

%time - Discrete time variable
time=1;

%User input: num_est - Total number of states you want to estimate. In this
%voltage example, it estimates 100 states
num_est=100;

%xhat - Cell array that will store each estimated state vector (xhat_k|k)
% Each  estimated state vector is size n x 1
xhat = cell(num_est,1);
%Initialize each estimated state vector to be zeros. These will be replaced with
%estimates
for  i =1:num_est
    xhat{i,1} = zeros(n,1);
    
end

%u - Cell array that will store each input vector (u_k)
%Each input vector is size mx1
u = cell(num_est,1);
%User input: See the paper for proper initialization
for  i =1:num_est
    u{i,1} = [0];
    
end
    

%F - Cell array that stores each F (state transistion) matrix 
%Each F matrix is size nxn
%User input: See the paper for proper initialization
F = cell(num_est,1);
for  i =1:num_est
    F{i,1} = [1];
    
end


%G - Cell array that stores each G (input transition) matrix
%Each G matrix is size nxm
G = cell(num_est,1);
%User input: See the paper for proper initialization
for  i =1:num_est
    G{i,1} = [0];
    
end

%H - Cell array that stores each H matrix 
%Each H matrix is size pxn
H = cell(num_est,1);
%User input: See the paper for proper initialization
for  i =1:num_est
    H{i,1} = [1];
    
end

%Q - Cell array that stores each covariance matrix for the noise w_k
%The size of each Q is nxn
Q = cell(num_est,1);
%User input: See the paper for proper initialization
for  i =1:num_est
    Q{i,1} = [0.0001];
           
    
end

%R - Cell array that stores each covariance matrix for the noise v_k
%The size of each R is pxp
R = cell(num_est,1);
%User input: See the paper for proper initialization
for  i =1:num_est
    R{i,1} = [1];
    
end

%W - Matrix that will store each noise vector 
%The noise vectors are of size nx1
W = cell(num_est,1);

%Note the noise is automatically generated based on the covariance matrix
%using mvrnd
for  i =1:num_est
    W{i,1} = (mvnrnd(zeros(1,n),Q{1,1}))';
    
end

%V - Matrix that will store each noise vector 
%The v vectors are size px1
V = cell(num_est,1);
%Note the noise is automatically generated based on the covariance matrix
%using mvrnd
for  i =1:num_est
    V{i,1} = (mvnrnd(zeros(1,p),R{1,1}))';
    
end

%P - Covariance matrix of errors (difference between estimated state and actual state) 
%Each p matrix is size nxn
P = cell(num_est,1);
%We initially fill all covariance matrices to be a matrix of zeros
for  i =1:num_est
    P{i,1} = zeros(n,1);
    
end


%z_predicted stores each predicted observation (zhat_k+1|k)
%The size of each element of z_predicted is px1
z_predicted = cell(num_est,1);
for  i =1:num_est
    %We initially fill all z predicted vectors to be vectors of zeros
    z_predicted{i,1} = zeros(p,1);
end


%x - Cell array that will store each state vector (x_k). We will generate state
%vectors based on an initial value of data and using equation 1
% Each state vector is size n x 1
x = cell(num_est,1);

%z - Matrix that will store each observation vector (z_k) in its columns
%Each observation vector is px1
z = cell(num_est,1);

%User Input: P_0 - Initial Covariance Matrix.
%See the paper for proper initialization.
P_0 = [1];

%Store initial value in P_cell
p_update = P_0;

%We make an initial observation of 0.5 DC Voltage. 
x_1 = [0.5]; 

%Store initial value for Kalman Filter
x{1,1} = x_1;
x_update = x_1;

%We generate the first observation vector
z{1,1} = H{1,1}*x{1,1} + V{1,1};

%We generate state vectors based on the initial observation and equation 1
%We generate observation vectors based on equation 2
for  i = 2:num_est
   x{i,1} = F{i-1,1}*x{i-1,1} + G{i-1,1}*u{i-1,1} + W{i-1,1};
   z{i,1} = H{i,1}*x{i,1} + V{i,1};
      
end


while(time<=num_est);
    
    
    %Store updated estimates for state(xhat_k+1|k+1) and error covariance (Phat_k+1|k+1)
    xhat{time,1} = x_update;
    P{time,1} = p_update;
    
    % Implements prediction equations 3 and 4
    [x_pred, p_pred] = predict(x_update,p_update, F{time,1}, Q{time,1},G{time,1},u{time,1});
    
     
    % Implements update equations 5 and 6
    [x_update,p_update, K,z_predictions] = update(x_pred, p_pred, z{time,1},H{time,1}, R{time,1});
    
    %Store z_predicted (zhat_k+1|k)
    z_predicted{time,1}= z_predictions;
    
    time=time+1;
    
end;

%Store time for plotting purposes
time_vec = zeros(1,num_est);


%This vector will hold a constant DC Voltage of 0.5V
ideal_voltage = zeros(1,num_est);

%This vector will hold the actual voltage measurements (z_k)
measurements_with_noise = zeros(1,num_est);

%This vector will hold the predicted observations based on The Kalman Filter (zhat_k+1)
predicted_observations = zeros(1,num_est);

%This stores the elements of the cell array elements into the the above
%vectors
for i=0:num_est-1 
    
    %Stores time
    time_vec(1,i+1)=i;
    
    %Stores ideal voltage
    ideal_voltage(1,i+1)=0.5;
   
    %Stores noisy voltage measurements
    d = z{i+1,1};
    measurements_with_noise(1,i+1)=d(1,1);
    
    %Stores Kalman Filter predicted measurements
    g=z_predicted{i+1,1};
    predicted_observations(1,i+1)=g(1,1);
    
  
end


%Plot of the constant 0.5 V DC Voltage, Kalman Filter predicted voltage, and
%the noisy voltage measurements
figure()
plot(time_vec,ideal_voltage,'r'); hold on;
plot(time_vec,measurements_with_noise,'.-b'); hold on;
plot(time_vec,predicted_observations,'--g');
legend('Constant 0.5 DC Voltage','Measured Values of Voltage','Kalman Filter for Voltage')
title('Voltage vs. Time')
xlabel('Time')
ylabel('Voltage');
