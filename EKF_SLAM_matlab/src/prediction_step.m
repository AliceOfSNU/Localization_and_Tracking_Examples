function [mu_pred, sigma_pred] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
N = (size(sigma,1)-3)/2;

theta = mu(3);
mu_pred = mu;
mu_pred(1) = mu(1) + u.t*cos(theta+u.r1);
mu_pred(2) = mu(2) + u.t*sin(theta+u.r1);
mu_pred(3) = normalize_angle(theta+u.r1+u.r2);

g = [-u.t*sin(theta+u.r1) ; u.t*cos(theta+u.r1); 0.0];
G3 = eye(3) + [zeros(3, 2) g];
G = eye(size(sigma,1));
G(1:3,1:3) = G3;

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0;
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

sigma_pred = G*sigma*G' + R;

end
