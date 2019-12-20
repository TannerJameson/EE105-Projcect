%K,E,T,O
%EE105 Project
%Challenge 1

close all;
clear all;

%read in csv data
ang_mat= csvread('yaw.csv');
 
%plot initial IMU readings to determine frequency thresholds
stem(ang_mat);
title('IMU Data')

%create Bode plot

%take abs val of angle values
ang_mat_mag1 = abs(ang_mat);
%convert to radians
ang_mat_mag= ((10.01)*(pi/180)).*ang_mat_mag1;
%form vector for each frequency
freq_pt5 = ang_mat_mag(1:40);
freq_1= ang_mat_mag(41:60);
freq_1pt5= ang_mat_mag(61:74);
freq_2 = ang_mat_mag(75:84);
freq_3= ang_mat_mag(84:92);
freq_5= ang_mat_mag(93:96);
freq_8= ang_mat_mag(97:100);
freq_10= ang_mat_mag(101:102);
freq_50= ang_mat_mag(103:length(ang_mat_mag));
ang_plot= fft(ang_mat_mag);
%take fft of each vector, find max of fft for each vector
vpt5= max(fft(freq_pt5));
v1= max(fft(freq_1));
v1pt5= max(fft(freq_1pt5));
v2= max(fft(freq_2));
v3= max(fft(freq_3));
v5= max(fft(freq_5));
v8= max(fft(freq_8));
v10= max(fft(freq_10));
v50= max(fft(freq_50));
%form frequency vector and angle vector
freq_vec = 2*pi*[.5 1 1.5 2 3 5 8 10 50];
ang_vec = [vpt5 v1 v1pt5 v2 v3 v5 v8 v10 v50];
figure;
plot(10*log10(freq_vec),20*log10(ang_vec))
title('Frequency Response of Angle Data')
ylabel('Magnitude (dB)')
xlabel('Frequency (dB)')



%find transfer function
s=tf('s');
%make transfer function
num= 1;
denom= s;
H=num/denom;
%plot transfer function
figure;
bode(H);
title("Bode Plot for Transfer Function 1/s");


% Try adding poles and zeros based on our data
H2 = 50*(s-10)*(s-15)*(s-8)/((s-13)*(s-18)*s^2);
figure;
bode(H2);
title("Bode Plot for Transfer Function 50*(s-10)*(s-15)*(s-8)/((s-13)*(s-18)*s^2)");





