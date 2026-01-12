close all

%%% Run conversion.m first

% simulation time steps
nsim = 400;

% new controller having integer coefficients
nc_ = length(Dc_) - 1; % order
F_0 = [eye(nc_-1); zeros(1,nc_-1)];
F_ = [-Dc_(2:end).', F_0];
G_ = Ncy_(2:end).';
P_ = Ncr_(2:end).';
H_ = [1, zeros(1,nc_-1)];

% plant initial condition
xp_ini = [0; 0; 0.01; -0.1];
% pre-designed controller initial condition
xc_ini = zeros(nc,1);
% set point reference
r_set = 2;

% plant + pre-designed controller
xp = xp_ini; % plant state
xc = xc_ini; % controller state
y = []; % plant output
u = []; % plant input
r = r_set; % reference signal

% plant + new controller
y_ = [];
u_ = [];
xc_ = zeros(nc_,1);
xp_ = xp_ini;

% simulation
for k = 1:nsim
    y = [y, C*xp(:,k)];
    y_ = [y_, C*xp_(:,k)];
    u = [u, H*xc(:,k)];
    u_ = [u_, H_*xc_(:,k)];
    xp = [xp, A*xp(:,k)+B*u(:,k)];
    xp_ = [xp_, A*xp_(:,k)+B*u_(:,k)];
    xc = [xc, F*xc(:,k)+G*y(:,k)+P*r];
    xc_ = [xc_, F_*xc_(:,k)+G_*y_(:,k)+P_*r];
end

% figures
figure(1)
plot(Ts*(0:nsim-1), u, 'Linewidth', 1.6,'Color', 'blue')
hold on
plot(Ts*(0:nsim-1), u_, 'Linewidth', 1.6, 'Color', 'red', 'LineStyle', '--')
legend('pre-designed', 'new')
xlabel('Time (s)', 'FontSize', 12)
ylabel('$u$', 'interpreter', 'latex', 'FontSize', 12)
set(gca,'FontSize',14, 'LineWidth', 1.2)

figure(2)
plot(Ts*(0:nsim-1), xp(3,1:nsim), 'Linewidth', 1.6,'Color', 'blue')
hold on
plot(Ts*(0:nsim-1), xp_(3,1:nsim), 'Linewidth', 1.6, 'Color', 'red', 'LineStyle', '--')
legend('pre-designed', 'new', 'Location', 'southeast')
xlabel('Time (s)', 'FontSize', 12)
ylabel('$\phi$', 'interpreter', 'latex', 'FontSize', 12)
set(gca,'FontSize',14, 'LineWidth', 1.2)

figure(3)
plot(Ts*(0:nsim-1), y, 'Linewidth', 1.6,'Color', 'blue')
hold on
plot(Ts*(0:nsim-1), y_, 'Linewidth', 1.6, 'Color', 'red', 'LineStyle', '--')
hold on
plot(Ts*(0:nsim-1), r_set*ones(1,nsim), 'Linewidth', 1.6, 'Color', 'black', 'LineStyle','-.')
legend('pre-designed', 'new', 'reference','Location', 'southeast')
xlabel('Time (s)', 'FontSize', 12)
ylabel('$x$', 'interpreter', 'latex', 'FontSize', 12)
set(gca,'FontSize',14, 'LineWidth', 1.2)