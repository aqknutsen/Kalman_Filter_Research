 
%Name: Alec Knutsen
%Date:12/08/15
%Description: Function that implements Kalman Filter Update Equations, Kalman Gain, and Predicton of Observations:
        %Kalman Gain: K_k+1 = (P_k+1|k)(H_k+1)^T[(H_k+1)(P_k+1|k)(H_k+1|k)^T
        %                      +R_k+1]^-1
        %Predicted Observation: zhat_k+1|k = (H_k+1)(xhat_k+1|k)
        %Update State: xhat_k+1|k+1 = xhat_k+1|k + K_k+1(z_k+1 -
        %(H_k+1)(xhat_k+1|k))
        %Update Error Covariance: P_k+1|k+1 = (I - (K_k+1)(H_k+1))(P_k+1|k)(I - (K_k+1)(H_k+1))^T + (K_k+1)(R_k+1)(K_k+1)^T
        
function [x_update,p_update,K ,z_pred] = update(xhat, P,z_k, H_k, R_k)

    K = P*H_k'* inv(H_k*P*H_k' + R_k);
    
    z_pred = H_k*xhat;
    
    x_update = xhat + K*(z_k - H_k*xhat);
    
    d = size(K*H_k);
    needed_size = d(1);
    
    p_update = (eye(needed_size) - K*H_k)*P*(eye(needed_size) - K*H_k)' + K*R_k*K';
    
    
    
end