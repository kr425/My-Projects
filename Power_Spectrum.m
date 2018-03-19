clear all; close all;

 [Y, FS]=audioread('1812.wav');
 [Yb, FSb]=audioread('Bird.wav');
 [Yn, FSn]=audioread('Nocture.wav');
 [Yp, FSp]=audioread('Partita.wav');
 [Yw, FSw]=audioread('Woodwind.wav');
figure
tic
pwelch(Y,256,[],256);
toc
figure
periodogram(Y,hamming(length(Y)),512);
figure
morder=441000/500;
pyulear(Y,morder,441000/500,FS);
figure
tic
pyulear(Y,1000,1024);
toc
figure
pyulear(Yb,100,256);
figure
pyulear(Yn,100,256);
figure
pyulear(Yp,100,256);
figure
pyulear(Yw,100,256);
