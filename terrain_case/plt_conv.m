load('total_cost.mat');
load('cst_map.mat');

init_cost = sum(cst_map(:,400));
total_cost = [init_cost total_cost] .* [3 ones(1,10)];

plot(i-1,total_cost,'k-^', 'LineWidth',1.5);
xlabel('number of iteration')
ylabel('total cost') % plot on new axes
axis([0 10 12 17])
axes('position',[.275 .65 .25 .25])
box on;
plot(i-1,total_cost,'k-^', 'LineWidth',1.5) % plot on new axes
axis([0 0.1 282 283])
