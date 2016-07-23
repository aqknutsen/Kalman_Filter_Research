 
%Name: Alec Knutsen
%Date:12/08/15
%Description: Function that implements Kalman Filter Prediction Equations:
    %Predict State: xhat_k+1|k = (F_k)(xhat_k|k) + (G_k)(u_k)
    %Predict Error Covariance: P_k+1|k = (F_k)(P_k|k)(F_k)^T + Q_k
    
function [x_pred, p_pred] = predict(xhat,P,F_k,Q_k,G_k,u_k) 


    x_pred = F_k*xhat + G_k*u_k;
    p_pred = F_k*P*F_k' + Q_k;
    
end

