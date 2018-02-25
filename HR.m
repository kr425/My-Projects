clear all; close all;

%num=xlsread('hr2signals.csv');
%num=xlsread('Heart_Rate.xlsx');
%num=xlsread('MYHRV_Peak_Det.csv');
num=xlsread('MYHRV.csv');
num1=load('DATA_01_TYPE01.mat');
num_samples=length(num);
% 
%  d = designfilt('lowpassfir', ...
%      'PassbandFrequency',0.01,'StopbandFrequency',0.08, ...
%      'PassbandRipple',1,'StopbandAttenuation',40, ...
%      'DesignMethod','equiripple');
%fvtool(d)
% h=[0.0233442033358828,-0.00690074311662770,7.48759818227425e-05,0.00441558364458574,-0.0156666799271075,0.0210102345082772,-0.0274040018573180,0.0224036038573684,-0.0134390766137762,-0.00926580861210226,0.0353030148676335,-0.0705277553075694,0.101605404211209,-0.132326479237670,0.149413305249862,0.841623317074389,0.149413305249862,-0.132326479237670,0.101605404211209,-0.0705277553075694,0.0353030148676335,-0.00926580861210226,-0.0134390766137762,0.0224036038573684,-0.0274040018573180,0.0210102345082772,-0.0156666799271075,0.00441558364458574,7.48759818227425e-05,-0.00690074311662770,0.0233442033358828];
% acc=zeros(8910,1);
% % for j=1:31
% %     acc(j,1)=h(j).*num(:,6);
% % end
% [b1,a1] = butter(5,.1/50,'high');
% freqz(b1,a1)
% figure
%  [b,a] = butter(5,.1/50,'low');
% 
% n    = length(a);
% z(n) = 0;  % Creates zeros if input z is omitted
% z1(n) = 0; 
% z2(n) = 0; 
% b = b / a(1);  % [Edited, Jan, 26-Oct-2014, normalize parameters]
% a = a / a(1);
% b1 = b1 / a1(1);  % [Edited, Jan, 26-Oct-2014, normalize parameters]
% a1 = a1 / a1(1);
% Y    = zeros(size(num(:,10)));
% % Y1    = zeros(size(num(:,6)));
% % Y2    = zeros(size(num(:,8)));
% tic
% for m = 1:length(Y)
%    Y(m) = b1(1) * num(m,2) + z(1);
% %    Y1(m) = b(1) * num(m,6) + z1(1);
% %    Y2(m) = b(1) * num(m,8) + z2(1);
%    for i = 2:n
%       z(i - 1) = b1(i) * num(m,2) + z(i) - a1(i) * Y(m);
% %       z1(i - 1) = b(i) * num(m,6) + z1(i) - a(i) * Y1(m);
% %       z2(i - 1) = b(i) * num(m,8) + z2(i) - a(i) * Y2(m);
%    end
% end
% for m = 1:length(Y)
%    Y(m) = b(1) * Y(m) + z(1);
% %    Y1(m) = b(1) * num(m,6) + z1(1);
% %    Y2(m) = b(1) * num(m,8) + z2(1);
%    for i = 2:n
%       z(i - 1) = b(i) * Y(m) + z(i) - a(i) * Y(m);
% %       z1(i - 1) = b(i) * num(m,6) + z1(i) - a(i) * Y1(m);
% %       z2(i - 1) = b(i) * num(m,8) + z2(i) - a(i) * Y2(m);
%    end
% end
% 
% toc
% z = z(1:n - 1);
% % z1 = z1(1:n - 1);
% % z2 = z2(1:n - 1);
% acc=filtfilt(d,Y);

%x=downsample(num(:,1)/10,4);
% x=num(:,1);
% num_samples=length(x);
% %n=downsample(sqrt(num(:,7).^2+num(:,6).^2),4);
% n=num(:,4);
% %n1=downsample(sqrt(num(:,7).^2+num(:,6).^2),4);
% n1=num(:,6);
% %n_=num(:,6);
% n2=num(:,8);
%num_samples=length(num1.sig);

%  n=filtfilt(d,num(:,1));
% n1=filtfilt(d,num(:,7));
%  n2=filtfilt(d,num(:,8));
% n3=sqrt(n.^2+n1.^2+n2.^2);
% 
%  t1 = linspace(0,length(x)/100,length(x));
% subplot(3,1,1);
% plot(t1,n)
% subplot(3,1,2);
% plot(t1,n1)
% subplot(3,1,3);
% plot(t1,n2)

%n=sqrt(num1.sig(5,1:10000)'.^2+num1.sig(6,1:10000)'.^2);
num_samples=length(num1.sig);
x=num1.sig(2,1:num_samples)';
n=num1.sig(4,1:num_samples)';
n1=num1.sig(5,1:num_samples)';
n2=num1.sig(6,1:num_samples)';
% n3=sqrt(n.^2+n1.^2+n2.^2);
% n=filtfilt(d,n);
% n1=filtfilt(d,n1);
%  n2=filtfilt(d,n2);
t1 = linspace(0,length(x)/100,length(x));
plot(t1,num1.sig(1,1:num_samples))
%--------------------------------------------------------------------------
% Filtering
%--------------------------------------------------------------------------
xnew=zeros(num_samples,1);
% for j=1:num_samples-1
%   % xnew(j,:)=(num1.sig(2,j)'+num1.sig(3,j)')/2;
%    % xnew(j,:)=(num(j,1)+num(j,9))/2;
%      xnew(j,:)=(num(j+1,1)-num(j,9))/(t1(j+1)-t1(j));
% 
% 
% end
%plot(t1(1,1:end-1),diff(x))
% Filter Parameters
p       = 155;                % filter order
lambda  = .9999;              % forgetting factor
laminv  = 1/lambda;
delta   = 10.0;              % initialization parameter

% Filter Initialization
w       = zeros(p,1);       % filter coefficients
P       = delta*eye(p);     % inverse correlation matrix
e       = xnew*0;              % error signal
tic
for m = p:length(x)

    % Acquire chunk of data
    y = n(m:-1:m-p+1);

    % Error signal equation
    e(m) = x(m)-w'*y;
    g=w'*y;
    % Parameters for efficiency
    Pi = P*y;
    
    % Filter gain vector update
    k = (Pi)/(lambda+y'*Pi);

    % Inverse correlation matrix update
    P = (P - k*y'*P)*laminv;

    % Filter coefficients adaption
    w = w + k*e(m);

    % Counter to show filter is working
    %if mod(m,1000) == 0
    %    disp([num2str(m/1000) ' of ' num2str(floor(length(x)/1000))])
    %end
    
end
e2=e;
for m = p:length(e2)

    % Acquire chunk of data
    y = n1(m:-1:m-p+1);

    % Error signal equation
    e(m) = e2(m)-w'*y;
    
    % Parameters for efficiency
    Pi = P*y;
    
    % Filter gain vector update
    k = (Pi)/(lambda+y'*Pi);

    % Inverse correlation matrix update
    P = (P - k*y'*P)*laminv;

    % Filter coefficients adaption
    w = w + k*e(m);

    % Counter to show filter is working
    %if mod(m,1000) == 0
    %    disp([num2str(m/1000) ' of ' num2str(floor(length(x)/1000))])
    %end
    
end
e3=e;
for m = p:length(e3)

    % Acquire chunk of data
    y = n2(m:-1:m-p+1);

    % Error signal equation
    e(m) = e3(m)-w'*y;
    
    % Parameters for efficiency
    Pi = P*y;
    
    % Filter gain vector update
    k = (Pi)/(lambda+y'*Pi);

    % Inverse correlation matrix update
    P = (P - k*y'*P)*laminv;

    % Filter coefficients adaption
    w = w + k*e(m);

    % Counter to show filter is working
    %if mod(m,1000) == 0
    %    disp([num2str(m/1000) ' of ' num2str(floor(length(x)/1000))])
    %end
    
end


toc

%--------------------------------------------------------------------------
% Plot
%--------------------------------------------------------------------------
fs=100;
% Plot filter results
t = linspace(0,length(x)/fs,length(x));
figure;

%plot(t,x,t,e,t,num1.sig(1,:)');
plot(t,x,t,e);
title('Result of RLS Filter')
xlabel('Time (s)');
legend('Reference', 'Filtered', 'Location', 'NorthEast');
title('Comparison of Filtered Signal to Reference Input');


%%





t=(1:length(num(:,3)-1))/1000;
plot(t,num(:,14),t,num(:,15));
figure
[acor,lag]=xcorr(num(:,14),num(:,15));
 d = fdesign.lowpass('Fp,Fst,Ap,Ast',.1,2,0.5,40,100);
   Hd = design(d,'equiripple');
   output = filter(Hd,num(:,14));
%plot(lag/1000,acor);
plot(t,output);

figure

f=fft(num(1:end-1,3));
plot(abs(f));
L=(length(num)-1);
figure

P2 = abs(f/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f1 = 114*(0:(L/2))/L;
plot(f1,P1)
axis([.5 60 0 7000]);    
old=f(100,1);
index=0;
for j=10:length(f)-10
    
    if(old<f(j,1))
        
        old=f(j,1);
        index=j;
    end
end
    
    figure
pwelch(num(1:end-1,3))
pxx =pwelch(num(1:end-1,3));
plot(10*log10(pxx))