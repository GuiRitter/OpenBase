a = 25;
b = 1000 / a;
c = idinput(a, 'prbs', [0 1], [-0.02, 0.02]);
for i = 0 : a - 1
    for j = 1 : b
        d((b * i) + j) = c(i + 1);
    end
end
fprintf(fopen('/home/guir/out.txt', 'wt+'), '%f\n', d);
