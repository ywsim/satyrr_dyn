function h = plot_ellip2(ax, M, origin, scale)
if isrow(origin)
    origin = origin';
end

N = 200;
q = linspace(0, 2*pi, N);
x_elip = cos(q);
y_elip = sin(q);
unit_circ = [x_elip; y_elip];

transf_elip = M*unit_circ;
x_transf_elip = transf_elip(1,:)*scale + origin(1);
y_transf_elip = transf_elip(2,:)*scale + origin(2);

h = plot(ax, x_transf_elip, y_transf_elip);
end