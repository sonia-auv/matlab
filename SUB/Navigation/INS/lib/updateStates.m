function [ X_k_plus ] = updateStates( X_k, dX )

% position
X_k_plus(1:2) = X_k(1:2) + dX(1:2);

% velocity
X_k_plus(3:4) = X_k(3:4) + dX(3:4);

% Heading
X_k_plus(5) = X_k(5) + dX(5);

% Biases
X_k_plus(6:8) = X_k(6:8) + dX(6:8);

end

