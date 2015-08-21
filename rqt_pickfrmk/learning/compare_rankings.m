Y = csvread('ranking_exp3_refined.txt')
ind2 = csvread('ranking_32_exp3.txt')

sum = 0;
d = corr(ind2, Y);
for i = 1:size(Y,1)
    %if Y1(i) > max(Y2)
    %    sum = sum + abs(max(Y2) - Y1(i));
    %    never_found = false;
    %end
    %if Y1(i) < min(Y2)
    %    sum = sum + abs(min(Y2) - Y1(i));
    %    never_found = false;
    %end
    pos = find(Y == ind2(i));
    if pos ~= i
        sum = sum + abs(pos - i);
    %    never_found = false;
    end

end
sum