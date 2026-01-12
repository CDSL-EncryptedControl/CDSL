clear all

%%% Stabilization by controllers having integer coefficients
%%% Section IV Conversion
%%% code by Joowon Lee, Donggil Lee
%%% example plant: linearized inverted pendulum

% plant model parameters
M = 0.5; m = 0.2; b = 0.1;
l = 0.2; I = 0.006; g = 9.8;
p = I*(M+m)+M*m*l^2;

% contiunous-time plant
Ac = [0, 1, 0, 0;
    0, -(I+m*l^2)*b/p, m^2*g*l^2/p, 0;
    0, 0, 0, 1;
    0, -m*l*b/p, m*g*l*(M+m)/p, 0];
Bc = [0; (I+m*l^2)/p; 0; m*l/p];
Cc = [1, 0, 0, 0];
Dc = 0;

% discrete-time plant
Ts = 0.05; % sampling time
plant_d = c2d(ss(Ac,Bc,Cc,Dc),Ts);
A = plant_d.A; B = plant_d.B; C = plant_d.C; D = plant_d.D;
[Np,Dp] = ss2tf(A,B,C,D);
n = length(plant_d.B);

% pre-designed controller
Q = eye(n);
R = 1;
L = dlqr(A.',C.',Q,R).';
K = -dlqr(A,B,10*Q,R);
KI = -0.02;
F = [A+B*K-L*C, B*KI;
    -C, 1];
G = [L; 0];
H = [K, KI];
P = [zeros(n,1); 1];
nc = length(G);
[Ncy,~] = ss2tf(F,G,H,0);
[Ncr,Dc] = ss2tf(F,P,H,0);

% Toeplitz matrix for Np
MNp = P2M(Np,n);
MNp2 = MNp(n+1:end,:);

iter_max = 5; % maximum number of iterations
prec = 1e-7; % precision for x=x^star
prec_final = 1e-10; % precision for Eq. (31)
mu = 0.99;
trial = 1000; % maximum number of Algorithm 2 executions
deg_max = 10; % maximum degree of alpha^ini

for j = 1:trial
    rng("shuffle")

    % alpha^ini selection
    deg_alpha = randi(deg_max);
    croots = floor(deg_alpha/2*rand);
    theta = 2*pi*rand(1,croots);
    radi = rand(1,croots);
    croot_temp = radi.*(cos(theta)+1i.*sin(theta));
    alpha_temp = poly([2*rand(1,deg_alpha-2*croots)-1, croot_temp, conj(croot_temp)]);
    alpha = alpha_temp/alpha_temp(1);

    %%% This alpha is used to reproduce the results reported in the paper of Section 4.A.
    % deg_alpha = 6;
    % load('conversion_alpha0.mat')
    % alpha = alpha0/alpha0(1);

    % initial N
    N = deg_alpha + nc - n;

    % ===================================
    % x0 computation
    % ===================================
    TNp = [];
    for i = 1:N
        temp = [zeros(i-1,1); Np.'; zeros(N-i,1)];
        TNp = [TNp, temp];
    end
    TNp1 = TNp(1:n,:);
    TNp2 = TNp(n+1:end,:);

    alphaDc = conv(alpha,Dc);
    beta = (-TNp2\alphaDc(n+2:end).').';
    x = alphaDc(2:n+1).' + TNp1*beta.';
    % ===================================

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

    % iterative update of alpha, beta, gamma
    for i = 1:iter_max

        delta = Delta(x,Np);

        u = delta\(xstar-x);

        if norm(u,1) >= 1
            u = mu*u/norm(u,1);
        end

        Mx = P2M([1;x].',n);
        Mx2 = Mx(n+1:end,:);

        x = x + delta*u;

        beta = conv(beta, [1, u.']) + [(-MNp2\(Mx2*u)).', zeros(1,N)];
        N = N + n;
        alpha = conv(alpha, [1, u.']);
        gamma = [1, x.', zeros(1,N)];

        if norm(x - xstar, inf) < prec
            break;
        end
    end

    % Problem 2 verification
    err = round(gamma) - conv(alpha,Dc) - [0,conv(beta,Np)];

    if norm(err, inf) < prec
        break;
    end
end

% controller design
Dc_ = round(gamma);
Ncy_ = [0,conv(beta,Dp)] + conv(alpha,Ncy);
Ncr_ = conv(alpha,Ncr);

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