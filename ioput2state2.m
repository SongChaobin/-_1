%本函数用于求解线性控制中有输入输出描述导出状态空间描述的问题
%通过分子次数m进行分类:m=0和m~=0两种
%m=0时,有
%                         b0
%g(s) = -----------------------------------------
%         (p^n + an-1*p^(n-1) + ... + a1*p + a0)
%m~=0时,有
%        (bn*p^n + bn-1*p^(n-1) + ... + b1*p + b0)
%g(s) = -------------------------------------------
%         (p^n + an-1*p^(n-1) + ... + a1*p + a0)
%函数输入为g(s)分子多项式系数num和分母多项式系数den,均按降次排序
%函数输出为系统和矩阵A,输入矩阵B,输出矩阵C和转移矩阵D
%e.g.   [A,B,C,D] = ioput2state2(num,den)
function [A,B,C,D] = ioput2state2(num,den)

n = numel(den)-1;     %获取分母维数n
new_den = zeros(1,n);

for i=1:n
    new_den(i) = -den(n-i+2);  %获取系统矩阵A最后一行
end

%初始化系数矩阵
A = zeros(n,n);
B = zeros(n,1);
C = zeros(1,n);
D = zeros(1,1);

if numel(num) == 1  % m=0的情形
    %求系统矩阵A
    A(1:n-1,1) = 0;
    A(1:n-1,2:end) = eye(n-1,n-1);
    A(n,:) = new_den;
    %求输入矩阵B
    B(end) = num;
    %求输出矩阵C
    C(1) = 1;
   
else               % m~=0的情形
    n_1 = numel(den);
    %求系统矩阵A
    A(1:n-1,1) = 0;
    A(1:n-1,2:end) = eye(n-1,n-1);
    A(n,:) = new_den;
    %求输入矩阵B
    beta = beta_iterate(n_1,num,den);
    B = beta(2:end);
    %求输出矩阵C
    C(1) = 1;
    %求转移矩阵D
    D = num(1);
end
end

% beta迭代算法
function beta = beta_iterate(n,num,den)
beta = zeros(1,n);
beta(1) = num(1);   %beta0 = bn
for i = 1:n-1       %beta1 = bn-an-1 * beta0 ...
    count = i;      %每次迭代中beta次序递减
    beta(i+1) = num(i+1);
    j = 1;          %每次迭代中a次序递增
    while count > 0
        beta(i+1) = beta(i+1) - beta(count)*den(j+1);
        count = count - 1;
        j = j + 1;
    end
end
end

