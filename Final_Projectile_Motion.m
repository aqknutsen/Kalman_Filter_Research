
%Name: Alec Knutsen
%Date:12/08/15
%Description:This Program runs the Discrete Kalman Filter Algorithm for the
%projectile motion example described in the paper.
%
%
%State Space Model:
    %   Equation 1: x_k+1 = (F_k)(x_k)(G_k)(u_k) + w_k
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
        %Equation 6: P_k+1|k+1 = (I - (K_k+1)(H_k+1))(P_k+1|k)(I - (K_k+1)(H_k+1))^T +
        %               + (K_k+1)(R_k+1)(K_k+1)^T

% Extra Equations:
    % Kalman Gain: K_k+1 = (P_k+1|k)(H_k+1)^T[(H_k)(P_k+1|k)(H_k)^T +
    %                       R_k+1]^-1


%Beginning of Program:
   
%User input: n - Size of state vectors
n=4;

%User input: m - Size of input vectors
m=4;

%User input: p - Size of observation vectors
p=2;

%time - Discrete time variable
time=1;

%User input: num_est - Total number of states you want to estimate
%Projectile Example: This estimates from 0-120 seconds at 0.1 second time
%intervals
num_est=1200;

%xhat - Cell array that will store each updated state vector (xhat_k|k)
% Each updated state vector is size n x 1
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
    u{i,1} = [0;  0;  0; -0.98];
    
end
    

%F - Cell array that stores each F (state transistion) matrix 
%Each F matrix is size nxn
%User input: See the paper for proper initialization
F = cell(num_est,1);
for  i =1:num_est
    F{i,1} = [1 0 0.1 0; 0 1 0 0.1;0 0 1-(0.0001) 0;0 0 0 1-(0.0001)];
    
end


%G - Cell array that stores each G (input transition) matrix
%Each G matrix is size nxm
G = cell(num_est,1);
%User input: See the paper for proper initialization
for  i =1:num_est
    G{i,1} = [1 0 0 0; 0 1 0 0; 0 0 1 0 ; 0 0 0 1];
    
end

%H - Cell array that stores each H matrix 
%Each H matrix is size pxn
H = cell(num_est,1);
%User input: See the paper for proper initialization
for  i =1:num_est
    H{i,1} = [1 0 0 0;0 1 0 0;];
    
end

%Q - Cell array that stores each covariance matrix for the noise w_k
%The size of each Q is nxn
Q = cell(num_est,1);
%User input: See the paper for proper initialization.
for  i =1:num_est
    Q{i,1} = [0.1 0 0 0;0 0.1 0 0;0 0 0.1 0;0 0 0 0.1];
           
    
end

%R - Cell array that stores each covariance matrix for the noise v_k
%The size of each R is pxp
R = cell(num_est,1);
%User input: See the paper for proper initialization
for  i =1:num_est
    R{i,1} = [500 0;0 500];
    
end

%W - Cell array that will store each noise vector 
%The noise vectors are of size nx1
W = cell(num_est,1);

%Note the noise is automatically generated based on the covariance matrix
%using mvrnd
for  i =1:num_est
    W{i,1} = (mvnrnd(zeros(1,n),Q{1,1}))';
    
end

%V - Cell array that will store each noise vector 
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

%z_predicted - Stores each predicted observation (zhat_k+1|k)
%The size of each element predicted obervation is px1
z_predicted = cell(num_est,1);
for  i =1:num_est
    %We initially fill all z predicted vectors to be vectors of zeros
    z_predicted{i,1} = zeros(p,1);
end

%x - Cell array that will store a generated sequence of state vectors (x_k) given
%an initial state vector.
% Each state vector is size n x 1
x = cell(num_est,1);

%z - Matrix that will store each observation vector (z_k) in its columns
%Each observation vector is px1
z = cell(num_est,1);

%ideal_observations - Matrix that will store each z_k with noise w_k,v_k
%removed
%Each ideal observation vector is px1
ideal_observations = cell(num_est,1);

%User Input: P_0 - Initial Covariance Matrix.  In this example, use the
%identity matrix.
P_0 = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];

%Store initial value in P_cell
p_update = P_0;

%Initiialize initial state vector. Assume we know the initial conditions
%for projectile motion.
x_1 = [0;0;300;600];
x{1,1} = x_1;
x_update = x{1,1};

%Generate the first observation vector
z{1,1} = H{1,1}*x{1,1} + V{1,1};

%Generate first ideal motion value (no noise)
ideal_observations{1,1} = H{1,1}*x{1,1};

%Generate a sequence of state vectors (x_k) based on the initial conditions.
%Also, generate a sequence of observation vectors (z_k)
%Finally, generate a sequence of observations with no noise w_k,v_k
for  i = 2:num_est
   x{i,1} = F{i-1,1}*x{i-1,1} + G{i-1,1}*u{i-1,1} + W{i-1,1};
   z{i,1} = H{i,1}*x{i,1} + V{i,1};
   
   x_no_noise = F{i-1,1}*x{i-1,1} + G{i-1,1}*u{i-1,1};
   ideal_observations{i,1} = H{i,1}*x_no_noise;

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

%END OF PROGRAM ANALYSIS AFTER. 

%Store time for plotting purposes
time_vec = zeros(1,num_est);

%This matrix will hold the ideal projectile motion observations (z_k with no noise w_k, v_k)
ideal_motion_mesurements = zeros(2,num_est);

%This matrix will hold the actual measurements (z_k)
measurements_with_noise = zeros(2,num_est);

%This matrix will hold the predicted observations (zhat_k+1)
predicted_observations = zeros(2,num_est);


%This stores the elements of the cell array elements into the the above
%vectors
for i=0:num_est-1 
    
    %Stores time
    time_vec(1,i+1)=i*0.1;
    
    %Store ideal x,y position (z_k with no noise w_k, v_k)
    b = ideal_observations{i+1,1};
    ideal_motion_mesurements(1,i+1)=b(1,1);
    ideal_motion_mesurements(2,i+1)=b(2,1);
    
    %Store x,y position of observations with noise(z_k)
    d = z{i+1,1};
    measurements_with_noise(1,i+1)=d(1,1);
    measurements_with_noise(2,i+1)=d(2,1);
    
    %Store x,y position of predicted observations from Kalman Filter (zhat_k+1|k)
    g=z_predicted{i+1,1};
    predicted_observations(1,i+1)=g(1,1);
    predicted_observations(2,i+1)=g(2,1);
    
  
end


%Plot of the ideal projectile motion, the noisy measurements motion, and
%the Kalman Filter predicted motion for the time between 4 and 6 seconds
%and 0.1 second time intervals
figure()
plot(ideal_motion_mesurements(1,400:600),ideal_motion_mesurements(2,400:600),'r'); hold on;
plot(measurements_with_noise(1,400:600),measurements_with_noise(2,400:600),'.'); hold on;
plot(predicted_observations(1,400:600),predicted_observations(2,400:600),'--g');
legend('True Projectile Motion for Measurements','Observed Measurement Values','Kalman Filter')
title('Y Position vs. X Position')
xlabel('X Position')
ylabel('Y Position')


