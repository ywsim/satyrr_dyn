function h = plot_ellip(ax, M, origin, scale)
if isrow(origin)
    origin = origin';
end
M_squared = M'*M;

N = 200;
q = linspace(0, 2*pi, N);
x_elip = cos(q);
y_elip = sin(q);

v_elip = [x_elip;y_elip];

% diagonalize  M = V*D*inv(V), then S = sqrt(M) = V*sqrt(D)*inv(V)
[V, D] = eig(M_squared);
S = V*diag(sqrt(diag(D)))*inv(V);

p_elip = scale*S*v_elip + [ones(1, N)*origin(1);ones(1,N)*origin(2)];

h = plot(ax, p_elip(1,:), p_elip(2, :));
end