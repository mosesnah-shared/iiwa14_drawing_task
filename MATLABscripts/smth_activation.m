function act = smth_activation( N, val1, val2, idx1, idx2 )

% Given an N array
act = zeros( 1, N );

assert( idx1 <= N && idx2 <= N )
assert( idx1 >= 1 && idx2 >= 1 )
assert( idx2 > idx1 )

% Fill in the values
act(    1:idx1 ) = val1;
act( idx2:end  ) = val2;

% In between
tmp = idx2-idx1+1;
% Using the sine function for smooth transition
% Mapping the phase from 0 to pi for smooth start at val1 and smooth end at val2
phase = linspace(0, pi, tmp);
smoothTransition = sin(phase - pi/2)/2 + 0.5; % Shift and scale sine wave
act( idx1:idx2 ) = val1 + ( val2 - val1 ) * smoothTransition;

end