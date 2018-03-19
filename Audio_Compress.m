clear all; close all;
%This code implements Sampling rate reduction
 [Y, FS]=audioread('1812.wav');
%sound(Y,FS);
 Y1=Y;
M=6;%downsampling rate
L=1%Upsampling rate

[grd_mean,h,w,n ]=myfilter(FS,M,L,-1,-80);
%fvtool(h,1);


%polyphase implementation
ez=poly1(h,M);
x_L=upsample(Y,L);
x_L=x_L';
y_n=[]
y_n2=[]
len=length(x_L);

newx_L=[zeros(1,round(M/2)) x_L zeros(1,round(M/2))];
for s=1:M
    x1=[zeros(1,s-1) newx_L zeros(1,M-s)];
    x_M2=downsample(x1,M);
    y_n2(s,:)=fftfilt(ez(s,:),x_M2);
end
y_n2=ifft(y_n2);
xd2=[zeros(1,length(y_n2))];
for k=1:M
    
    xd2=xd2+y_n2(k,:);
end
y_n2=ifft(y_n2);
for j=1:M
    x=[zeros(1,j-1) x_L(1:len) zeros(1,M-j)];
    x_M=downsample(x,M);
    y_n(j,:)=fftfilt(ez(j,:),x_M);
end
y_n=ifft(y_n);
xd=[zeros(1,length(y_n))];
for k=1:M
    
    xd=xd+y_n(k,:);
end

S=std(xd);
b=8.0;%desired # of bits
delta1=S/(2^(b-2.05));
xQ=delta1*round(xd/delta1);
xQ2=delta1*round(xd2/delta1);
L2=6;
M2=1;
xQ=fft(xQ);
xQ2=fft(xQ2);
xq_M=downsample(xQ,M2);
xq_M2=downsample(xQ2,M2);
ez2=poly1(h,L2);
p=0;
xrest=[];
p1=0;
% for k=1:L2
%   
%    xez2=fftfilt(ez2(k,:),xq_M);
%   
%    xq_L=upsample(xez2,L2);
%    xq_L_delay=[zeros(1,k-1),xq_L,zeros(1,M-k)];
%     if(length(p) ~= length(xq_L_delay))
%  p = [p zeros(1, length(xq_L_delay) - length(p))];
%     end
%   p = p + xq_L_delay;
% end
%p=[zeros(1,round(L2/2)) p zeros(1,round(L2/2))]; 
for k=1:L2
  
   xez3=fftfilt(ez2(k,:),xq_M2);
  
   xq_L2=upsample(xez3,L2);
   xq_L_delay2=[zeros(1,k-1),xq_L2,zeros(1,M-k)];
    if(length(p1) ~= length(xq_L_delay2))
 p = [p1 zeros(1, length(xq_L_delay2) - length(p1))];
    end
  p1 = p1 + xq_L_delay2;
end
% pout=real(p1+p);
morder=441000/500;
figure
[py1,f1]=pyulear(p,morder,441000/500,FS);
plot(f1,10*log10(abs(py1)));


p=[zeros(1,round(n)) p];
Y=[Y' zeros(1,length(p)-441000)];
% pout=[zeros(1,round(n)) pout];
% Y1=[Y1' zeros(1,length(pout)-441000)];
pnew=p-Y;
% pnew2=pout-Y1;
var=var(pnew);
myvar=mean(pnew.^2);
%pause(12)
sound(real(pnew),FS);
morder=441000/500;
figure
[py,f]=pyulear(pnew,morder,441000/500,FS);
plot(f,10*log10(abs(py)));



