function [CROSS] = CROSS_PARALLEL(V1_long, V2_long)

    N = length(V1_long)/3;
    V1 = zeros(3,1,N);
    V2 = V1;

    % Reshape
    V1 = reshape(V1_long,3,1,N);
    V2 = reshape(V2_long,3,1,N);

    % Extract vector entries
    a1 = V1(1, 1, :);
    a2 = V1(2, 1, :);
    a3 = V1(3, 1, :);

    % Initialize transformation matrix SSM
    V1_SSM = gpuArray.zeros(3, 3, N);
    
    % Populate the transformation matrices
    V1_SSM(1, 2, :) = -a3;
    V1_SSM(1, 3, :) = a2;
    V1_SSM(2, 1, :) = a3;
    V1_SSM(2, 3, :) = -a1;
    V1_SSM(3, 1, :) = -a2;
    V1_SSM(3, 2, :) = a1;

    % Compute ED using page-wise matrix multiplication
    CROSS = pagefun(@mtimes, V1_SSM, V2);
    
    % Gather the result if needed
    % CROSS = gather(CROSS);
end
