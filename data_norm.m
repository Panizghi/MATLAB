% sample data

x = rand(1,500)/100; 
y = 2.*(rand(1,500)-0.5).*90; 
z = (x.*1e2).^2; 

% construct a grid of query points

X = linspace(min(x),max(x),25); 
Y = linspace(min(y),max(y),25); 
[xq, yq] = meshgrid(X,Y); 
zq = griddata(x,y,z,xq,yq); 

plot3(x,y,z,'mo')
hold on
mesh(xq,yq,zq)
xlabel('x')
ylabel('y')
hold off

% Normalize Sample Points
x = normalize(x);
y = normalize(y);

% Regenerate Grid 
X = linspace(min(x),max(x),25); 
Y = linspace(min(y),max(y),25); 
[xq, yq] = meshgrid(X,Y); 

% Interpolate and Plot
zq = griddata(x,y,z,xq,yq);
plot3(x,y,z,'mo')
hold on
mesh(xq,yq,zq)
 
  
