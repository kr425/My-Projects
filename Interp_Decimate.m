


function y_n = Interp_Decimate(x_n)
%
% y_n = srconvert(x_n) Performs sampling rate conversion of input signal, 
% x_n sampled at 11,025 Hz and converts to y_n sampled at 24,000 Hz

    f_samp = 11025; %input sampling rate
    L = 320;        %Upsample amount
    M = 147;        %downsample amount

%-------------------------------------------------------------------------%
%% Approach (1) - Straightforward Implementation                           
%-------------------------------------------------------------------------%
% %            _ _ _        _ _ _ _ _ _        _ _ _ _
% %           |      |     |   L.P.F.   |     |       |
% %  x[n] --> | up L | --> | w_c = pi/L | --> | dwn M | --> y[n]
% %           |_ _ _ |     |_ _ _ _ _ _ |     |_ _ _ _|
% %
% %
% % ------------------------- UNCOMMENT TO USE -------------------------

%Upsample x[n] by L
%     x_e = upsample(x_n, L);
%     
% %Low-Pass Filter
%     %Filter specifications
%         w_p = (pi / L) * f_samp;        %Pass-band
%         w_s = 1.2 * w_p;                %stop-band
%         f_cuts = [w_p , w_s] ./ (2*pi);   %normalized freq band
%         mags =   [ 1  ,  0 ];             %magnitude response
%         dev = db2mag([0.1 -75]);  %passband ripple & stopband attenuation
%         
%     %Filter design
%         [n, Wn, beta, ftype] = kaiserord(f_cuts, mags, dev, f_samp);
%         h_n = L * fir1(n, Wn, ftype, kaiser(n+1, beta), 'noscale');
%         
%     %filter signal
%         y_m = fftfilt(h_n, x_e);
% 
% %Downsample 
% y_n = downsample(y_m, M);


%-------------------------------------------------------------------------%
%% Approach (2) - Polyphase Decomposition of Downsampling                  
%-------------------------------------------------------------------------%
% %            _ _ _           _ _ _ _       _ _ _ _
% %           |      |        |       |     |        |
% %  x[n] --> | up L | -+---> | dwn M | --> | E_0(z) | --------+
% %           |_ _ _ |  |     |_ _ _ _|     |_ _ _ _ |         |
% %                     |z^-1  _ _ _ _       _ _ _ _           |
% %                     |     |       |     |        |         |
% %                     +---> | dwn M | --> | E_1(z) | ----> (sum) --> y[n]
% %                     |     |_ _ _ _|     |_ _ _ _ |         ^
% %                     |z^-1                                  |
% %                     .         .             .              .
% %                     .         .             .              .
% %                     .         .             .              .
% %                     |      _ _ _ _       _ _ _ _ _         |
% %                     |     |       |     |          |       |
% %                     +---> | dwn M | --> | E_M-1(z) | ------+
% %                           |_ _ _ _|     |_ _ _ _ _ |
% %
% %
% % ------------------------- UNCOMMENT TO USE -------------------------

% %Upsample x[n] by L
    x_e = upsample(x_n, L);
% 
% %Low-Pass Filter Design & Polyphase Decomposition
%     %Filter specifications
        w_p = (pi / L) * f_samp;        %Pass-band
        w_s = 1.2 * w_p;                %stop-band
        f_cuts = [w_p , w_s] ./ (2*pi);   %normalized freq band
        mags =   [ 1  ,  0 ];             %magnitude response
        dev = db2mag([0.1 -75]);  %passband ripple & stopband attenuation
%         
%     %Filter design
        [n, Wn, beta, ftype] = kaiserord(f_cuts, mags, dev, f_samp);
         h_n = L * fir1(n, Wn, ftype, kaiser(n+1, beta), 'noscale');
%         
%     %Type I polyphase decomposition of h[n] in M components
         E = poly1(h_n, M);
% 
%         
% %Decimation Filter using Downsampling Identity with Polyphase Decomposition
     y_decomp = [];
     len_e = length(x_e);
%     
     for k = 1:M
%         % Delay Signal by k
             x_ez = [ zeros(1, k-1) x_e(1:len_e) zeros(1, M-k)];
% 
%         % Downsample Signal by M
             x_z = downsample(x_ez, M);
% 
%         % Low-Pass Filter
             y_decomp(k, :) = fftfilt(E(k, :), x_z);
     end
% 
     y_n = zeros(1, size(y_decomp, 2));    %initialize summation
     for row = 1:M
         y_n = y_n + y_decomp(row, :);   %add together polyphase components
     end


%-------------------------------------------------------------------------%
%% Approach (3) - Polyphase Decomposition of Upsampling                    
%-------------------------------------------------------------------------%
% %                _ _ _ _        _ _ _                  _ _ _ _
% %               |        |     |      |               |       |
% %  x[n] --+---> | E_0(z) | --> | up L | --->(sum) --> | dwn M | --> y[n]
% %         |     |_ _ _ _ |     |_ _ _ |       ^       |_ _ _ _|
% %         |      _ _ _ _        _ _ _         |z^-1
% %         |     |        |     |      |       |
% %         +---> | E_1(z) | --> | up L | --->(sum)
% %         |     |_ _ _ _ |     |_ _ _ |       ^
% %         |                                   |
% %         .         .             .           .
% %         .         .             .           .
% %         .         .             .           .
% %         |     _ _ _ _ _        _ _ _        |z^-1
% %         |    |          |     |      |      |
% %         +--> | E_L-1(z) | --> | up L | -----+
% %              |_ _ _ _ _ |     |_ _ _ |
% %
% %
% % ------------------------- UNCOMMENT TO USE -------------------------


% %Low-Pass Filter Design & Polyphase Decomposition
%     %Filter specifications
%         w_p = (pi / L) * f_samp;        %Pass-band
%         w_s = 1.2 * w_p;                %stop-band
%         f_cuts = [w_p , w_s] ./ (2*pi);   %normalized freq band
%         mags =   [ 1  ,  0 ];             %magnitude response
%         dev = db2mag([0.1 -75]);  %passband ripple & stopband attenuation
%         
%     %Filter design
%         [n, Wn, beta, ftype] = kaiserord(f_cuts, mags, dev, f_samp);
%         h_n = L * fir1(n, Wn, ftype, kaiser(n+1, beta), 'noscale');
%         
%     %Type I polyphase decomposition of h[n] in L components
%         E = poly1(h_n, L);
% 
% %Decimation Filter using Upsampling Identity with Polyphase Decomposition
%     y_e = 0;
%     for k = 1:L
%         % Polyphase-decomposed filter
%             y_poly = fftfilt(E(k, :), x_n);
%             
%         % Upsample Signal
%             y_poly_e = upsample(y_poly, L);
%             
%         % Delay by k-1
%             y_poly_ez = [zeros(1, k-1) y_poly_e zeros(1, M-k)];
%             
%         if(length(y_e) ~= length(y_poly_ez))
%             y_e = [y_e zeros(1, length(y_poly_ez) - length(y_e))];
%         end
%         
%         y_e = y_e + y_poly_ez;
%         
%     end
%     
% %Downsample y_e[n] by M    
%     y_n = downsample(y_e, M);
