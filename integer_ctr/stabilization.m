clear all

%%% Stabilization by controllers having integer coefficients
%%% code by Joowon Lee, Donggil Lee
%%% example plant: linearized inverted pendulum

% plant model parameters
M = 0.5; m = 0.2; b = 0.1;
l = 0.2; I = 0.006; g = 9.8;
p = I*(M+m)+M*m*l^2;

% contiunous-time plant
A = [0, 1, 0, 0;
    0, -(I+m*l^2)*b/p, m^2*g*l^2/p, 0;
    0, 0, 0, 1;
    0, -m*l*b/p, m*g*l*(M+m)/p, 0];
B = [0; (I+m*l^2)/p; 0; m*l/p];
C = [1, 0, 0, 0];
D = 0;

% discrete-time plant
Ts = 0.05; % sampling time
plant_d = c2d(ss(A,B,C,D),Ts);
[Np,Dp] = ss2tf(plant_d.A,plant_d.B,plant_d.C,plant_d.D);
n = length(plant_d.B);

Q = P2M(Np,n); % Toeplitz matrix T_n(Np)
P = P2M(Dp,n); % Toeplitz matrix T_n(Dp)

iter_max = 2; % maximum number of iterations
prec = 1e-7; % precision for x=x^star
mu = 0.99;
trial = 1000; % maximum number of Algorithm 1 executions

for j = 1:trial
    rng("shuffle")

    % gamma^ini selection
    croots = round(n*rand); % number of complex roots
    % generation of complex roots inside unit circle
    theta = 2*pi*rand(1,croots);
    radi = rand(1,croots);
    croot_temp = radi.*(cos(theta)+1i.*sin(theta));
    % polynomial generation
    gamma_temp = poly([2*rand(1,2*(n-croots))-1, croot_temp, conj(croot_temp)]);
    gamma = gamma_temp/gamma_temp(1); % to make gamma monic

    % load('stabilization_gamma0.mat')  % This gamma is used to reproduce the results reported in the paper of Section 3.D.
    % gamma  = gamma0/gamma0(1);

    % x_0 computation
    init = [P,Q]\(gamma(2:end).'-[Dp(2:end).';zeros(n,1)]);
    a0 = init(1:n);
    b0 = init(n+1:end);
    x = a0;

    % initialization of alpha, beta, N
    alpha = [1, x.'];
    beta = b0;
    N = 0;

    % x^star selection
    xstar = round(x);
    % ===================================
    % check condition (18) for selected x^star
    % ===================================
    rNp = roots(Np);
    tol = 1e-10;

    % Real zeros of Np(z)
    real_roots = rNp(abs(imag(rNp)) < tol);

    % Complex zeros (one representative from each conjugate pair)
    complex_roots = rNp(imag(rNp) > tol);

    ok = true;

    % ----- Real zeros: single hyperplane (Eq. (16)) -----
    for k = 1:length(real_roots)
        lambda = real_roots(k);

        % Define phi and psi such that
        % phi^T x - psi = p_x(lambda)
        phi = lambda.^(n-1:-1:0).';
        psi = -lambda^n;

        % Sign condition (18)
        c = (phi.'*x - psi) * (phi.'*xstar - psi);

        if c <= 0
            ok = false;
            break;
        end
    end

    % ----- Complex zeros: two hyperplanes (Eq. (17)) -----
    if ok
        for k = 1:length(complex_roots)
            eta = complex_roots(k);   % imag(eta) > 0

            % Vandermonde rows for eta and its conjugate
            V = [eta.^(n:-1:0);
                 conj(eta).^(n:-1:0)];

            % Transformation from complex equations to
            % real (Re/Im) hyperplane representation
            T = 0.5 * [  1,  1;
                -1i, 1i ];

            M = T * V;   % 2-by-(n+1)

            % First hyperplane: Re(p_x(eta)) = 0
            phi1 = real(M(1,2:end)).';
            psi1 = -real(M(1,1));

            % Second hyperplane: Im(p_x(eta)) = 0
            phi2 = real(M(2,2:end)).';
            psi2 = -real(M(2,1));

            c1 = (phi1.'*x - psi1) * (phi1.'*xstar - psi1);
            c2 = (phi2.'*x - psi2) * (phi2.'*xstar - psi2);

            if (c1 <= 0) || (c2 <= 0)
                ok = false;
                break;
            end
        end
    end

    % If sign condition is violated, discard this trial
    if ~ok
        continue;
    end
    % ===================================

    Q2 = Q(n+1:end,:);

    % iterative update of alpha, beta, gamma

    for i = 1:iter_max
        % Compute Delta(x)
        delta = Delta(x,Np);

        % Compute update direction (Eq. (20))
        u = delta \ (xstar - x);

        % Enforce ||u||_1 < 1
        if norm(u,1) >= 1
            u = mu * u / norm(u,1);
        end

        % Update x, beta, and gamma
        R  = P2M([1;x].',n);
        R2 = R(n+1:end,:);

        x = x + delta*u;

        beta = conv([1, u.'],beta) ...
            - [conv(Dp, -inv(Q2)*R2*u), zeros(1,N)];

        N     = N + n;
        gamma = conv(gamma, [1, u.']);
        alpha = [1, x.', zeros(1,N)];

        % Convergence check
        if norm(x - xstar, inf) < prec
            break;
        end
    end

    % controller design and verification
    Dc = round(alpha);
    Nc = -beta;

    CL = conv(Dc,Dp) - conv([0,Nc],Np);

    clroots = max(abs(roots(CL)));

    if clroots < 1
        break;
    end
end

%%% function: polynomial to Toeplitz matrix
function M = P2M(a,m)
    % a: polynomial represented as a row vector
    len = length(a)-1;
    if len<m
        a = [zeros(1,m-len),a];
    elseif len>m
        a = a(len-m+1:end);
    end
    M = [];
    for i = 1:m
        temp = [zeros(i-1,1); a.'; zeros(m-i,1)];
        M = [M, temp];
    end
end

%%% function: construct Delta matrix
function X = Delta(x,q)
    n = length(x);
    Q = P2M(q,n);
    Q1 = Q(1:n,:);
    Q2 = Q(n+1:end,:);
    R = P2M([1;x].',n);
    R1 = R(1:n,:);
    R2 = R(n+1:end,:);
    X = R1 - Q1*inv(Q2)*R2;
end