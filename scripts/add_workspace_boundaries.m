function [] = add_workspace_boundaries(figHandle,varargin)
% ADD_WORKSPACE_BOUNDARIES draws the reaching model workspace boundaries on
% an existing figure.
% USAGE
%   add_workspace_boundaries(figHandle,optional:linestyle,optional:linewidth)

linestl = 'k--';
linewdth = 2;
if nargin>2
    for iarg = 1:nargin-1
        if isnumeric(varargin{iarg})
            linewdth = varargin{iarg};
        elseif isstring(varargin{iarg}) || ischar(varargin{iarg})
            linestl = varargin{iarg};
        end
    end
end

link_lengths = [0.44 0.32 0.25 0.20]';
Ltot = sum(link_lengths(2:end));

figure(figHandle)
hold on
xl = xlim;
yl = ylim;

% outer arc 1 (extended arm, shoulder flexion)
theta = (-20:5:120)'.*pi/180;
outer_arc1_x = Ltot*cos(theta);
outer_arc1_y = Ltot*sin(theta);

plot(outer_arc1_x, outer_arc1_y,linestl,'LineWidth',linewdth)

% inner arc 1 (flexed arm, shoulder flexion)
flex_pos_x = sum(link_lengths(2:end).*cos(cumsum([0 120 70]'*pi/180)));
flex_pos_y = sum(link_lengths(2:end).*sin(cumsum([0 120 70]'*pi/180)));
r = sqrt(flex_pos_x^2+flex_pos_y^2);
flex_pos_theta = atan(flex_pos_y/flex_pos_x);
theta = theta(1:end-9);
inner_arc1_x = r*cos(theta+flex_pos_theta+pi);
inner_arc1_y = r*sin(theta+flex_pos_theta+pi);

plot(inner_arc1_x, inner_arc1_y,linestl,'LineWidth',linewdth)

% outer arc 2 (flexed shoulder, elbow flexion)
theta = (0:5:120)'.*pi/180+120*pi/180;
outer_arc2_x = sum(link_lengths(3:end))*cos(theta)+link_lengths(2)*cos(120*pi/180);
outer_arc2_y = sum(link_lengths(3:end))*sin(theta)+link_lengths(2)*sin(120*pi/180);

plot(outer_arc2_x, outer_arc2_y,linestl,'LineWidth',linewdth)

% inner arc 2 (extended shoulder, elbow flexion)
flex_pos_x = cumsum(link_lengths(2:end).*cos(cumsum([0 120 70]'*pi/180)));
flex_pos_y = cumsum(link_lengths(2:end).*sin(cumsum([0 120 70]'*pi/180)));
flex_pos_x = flex_pos_x(end)-flex_pos_x(end-2);
flex_pos_y = flex_pos_y(end)-flex_pos_y(end-2);
r = sqrt(flex_pos_x^2+flex_pos_y^2);
theta = (0:5:120)'.*pi/180-15*pi/180;
flex_pos_theta = 0.45;
inner_arc2_x = r*cos(theta+flex_pos_theta)+link_lengths(2)*cos(-20*pi/180);
inner_arc2_y = r*sin(theta+flex_pos_theta)+link_lengths(2)*sin(-20*pi/180);

plot(inner_arc2_x, inner_arc2_y,linestl,'LineWidth',linewdth)

plot([-0.44 -.18],[0.045 0.045],linestl,'LineWidth',linewdth)

shoulder_x = -0.44 + 0.045*cos((0:5:90)'.*pi/180 + pi/2);
shoulder_y = 0.045*sin((0:5:90)'.*pi/180 + pi/2);
plot(shoulder_x,shoulder_y,linestl,'LineWidth',linewdth)

xlim(xl)
ylim(yl)

end
