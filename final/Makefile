all:
	sudo chmod 777 /var/run/sdp
	g++ arduino.c minmea.c -o arduino -lbluetooth -lstdc++ -lwiringPi
encoder:
	sudo chmod 777 /var/run/sdp
	g++ final_encoder.c minmea.c -o encoder -lbluetooth -lstdc++ -lwiringPi
kalman:
	sudo chmod 777 /var/run/sdp
	g++ final_kalman.c minmea.c kalman.cpp matrix.cpp -o kalman -lbluetooth -lstdc++ -lwiringPi
