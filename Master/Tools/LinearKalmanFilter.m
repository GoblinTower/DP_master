classdef LinearKalmanFilter < handle
% Simple Kalman filter  class that allows for time varying state transition
% matrix, input matrix and output matrix.
%
    properties
        x_apriori {mustBeNumeric}         % Apriori state vector
        x_aposteriori {mustBeNumeric}     % Aposteriori state vector
        p_apriori {mustBeNumeric}         % Apriori covariance matrix
        p_aposteriori {mustBeNumeric}     % Aposteriori covariance matrix

        y_apriori {mustBeNumeric}         % Apriori output vector
    
        kalman_gain {mustBeNumeric}       % Kalman gain
    end
    
    methods
        
        function obj = LinearKalmanFilter(x_init, p_init)
            % Constructor. Need initial state vector estimate and covariance matrix
            % estimate to start Kalman filter.
            %
            % INPUT:
            % x_init            : Initial x_aposteriori (state vector)
            % p_init            : Initial p_aposteriori (covariance matrix)
            %
            obj.x_aposteriori = x_init;
            obj.p_aposteriori = p_init;
        end
      
        function [x_est, P, K] = UpdateFilter(obj, u, y, A, B, C, W, V)
            % This functions update the Kalman filter.
            %
            % INPUT:
            % u                 : Input vector
            % y                 : Output vector
            % A                 : State transition matrix
            % B                 : Input matrix
            % C                 : Output matrix
            % W                 : Process noise covariance matrix
            % V                 : Measurement noise covariance matrix
            %
            % OUTPUT:
            % x                 : State estimate (aposteriori)
            % P                 : Covariance matrix of state error (aposteriori)
            % K                 : Kalman gain
            %
           
            % Update covariance matrix for time step (k+1, apriori)
            obj.p_apriori = A*obj.p_aposteriori*A' + W;

            % Update Kalman gain
            obj.kalman_gain = obj.p_apriori*C'*inv(C*obj.p_apriori*C' + V);

            % Compute predicted state (k+1, apriori)
            obj.x_apriori = A*obj.x_aposteriori + B*u;

            % Compute output (k, apriori)
            obj.y_apriori = C*obj.x_aposteriori;

            % Compute the corrected value 
            obj.x_aposteriori = obj.x_apriori + obj.kalman_gain*(y - obj.y_apriori);

            % Update covariance matrix
            dim_n = size(A,1);
            obj.p_aposteriori = (eye(dim_n) - obj.kalman_gain*C)*obj.p_apriori*...
                (eye(dim_n) - obj.kalman_gain*C)' + obj.kalman_gain*V*obj.kalman_gain';

            x_est = obj.x_aposteriori;
            P = obj.p_aposteriori;
            K = obj.kalman_gain;
        end
    end
end