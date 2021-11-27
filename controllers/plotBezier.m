function [] = plotBezier(alpha)
n = length(alpha);
P(1,:) = alpha;
P(2,:) = zeros(1,n);
P(3,:) = zeros(1,n);
count = 1;

div = 50; %number of segments of the curve (Increase this value to obtain a
          %smoother curve

for u = 0:(1/div):1
    sum = [0 0 0]';
    for i = 1:n
        B = nchoosek(n,i-1)*(u^(i-1))*((1-u)^(n-i+1)); %B is the Bernstein polynomial value
        sum = sum + B*P(:,i);
    end
    B = nchoosek(n,n)*(u^(n));
    sum = sum + B*P(:,n);
    A(:,count) = sum; %the matrix containing the points of curve as column vectors. 
    count = count+1;  % count is the index of the points on the curve.
end

for j = 1:n %plots the points
    plot(P(1,j),P(2,j),'*');
    hold on;
end


p1 = P(:,1);

%draws the characteristic polygon describing the bezier curve
for l = 1:n-1
    p2 = P(:,l+1)';
    lineplot(p1,p2); %function the plots a line between two points.
    p1 = p2;
end

%plotting the curve
x = A(1,:);
y = A(2,:);
plot(x,y);
axis equal;

end

%function definitions
function [] = lineplot(A,B)

x = [A(1) B(1)]; 
y = [A(2) B(2)]; 
plot(x,y,'--r'); %a dashed red  line 

end 

