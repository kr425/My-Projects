function [xQ,b,delta] = quantizer(xd,M)
%This function performs a uniform quantizer
%b=round(64000/(44100/M));
b=32;
S=std(xd);
delta=S/(2^(b-2.05));
if delta>S
    error('delta to big');
end
xQ=delta*round(xd/delta);

end

