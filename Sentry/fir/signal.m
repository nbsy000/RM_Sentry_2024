load('acc1.mat')  
Fs = 100;%100Hz�Ĳ���Ƶ��
N = length(acc1);
x = (0:N-1)/Fs;%����20001��ȡ����
y = acc1;%��ƺ��и�Ƶ�ź����Ƶ�źŵ������ź�
figure(1);
plot(x,y);%���������ź�ͼ��
title('�����ź�');

%ʹ��fft����Ƶ��
y0 = abs(fft(y));
f = (0:N-1)*Fs/N;
figure(2);
plot(f,y0);%fftƵ��
title('Ƶ��');


% tempy = y.*0;
% k = 0.03;
% for i = 2:N
%     tempy(i) = (1-k)*tempy(i-1)+(k)*y(i);
% end
% d = tempy;
Hd = acc_iir_2;%�����˲���,Hd������fir_8�˲����ĸ������
d = filter(Hd,y);%ͨ��filter�������ź�y�������ΪHd���˲���������ź�d
figure(3);
plot(x,d);%����ͨ���˲������ź�d�Ĳ���
title('����ź�');


figure(4);
plot(x(5000:6000),y(5000:6000),'r');%���������ź�ͼ��
hold on;%���ֻ����������ź�ͼ��
plot(x(5000:6000),d(5000:6000),'b');%��������źŲ���
title('����/����ź�');
legend('����ź�','�����ź�');

figure(5);
plot(f,y0,'r');%��������Ƶ��ͼ��
hold on;%���ֻ����������ź�ͼ��
d0 = abs(fft(d));
plot(f,d0,'b');%��������źŲ���
title('����/����ź�');
legend('����ź�','�����ź�');

