#include <wiringSerial.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>

#include "minmea.h"

using namespace std;

typedef struct {
        float latitude, longitude, heading, velocity;
} gps_data_t;

gps_data_t data;

void parse(char[]);
int avail(){
	fd_set readfds;
	FD_ZERO(&readfds);
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	FD_SET(STDIN_FILENO, &readfds);

	int num = select(1, &readfds, NULL, NULL, &timeout);

	if(num > 0){
		return 1;
	} else if(num == -1){
		return 0;
	} else {
		return 0;
	}
}


int fd, i;
int flag = 0;
char msg[MINMEA_MAX_LENGTH] = {0, };

int main(){
	//fd = serialOpen("/dev/ttyACM0", 9600);

	while(1){
		//int ch = serialGetchar(fd);
		if (0 != cin.peek()) {
			char ch;
                        scanf("%c", &ch);
                        if(flag == 0){
                                if(ch == '$'){
                                        flag = 1;
                                        i = 0;
                                        msg[i] = ch;
                                        i++;
                                }
                        }else{
                                msg[i] = ch;
                                i++;
                                if(ch == '\n'){
                                        flag = 0;
                                        msg[i] = '\0';
                                        parse(msg);
                                }
                        }
		}else{
			printf("not avail\n");
			usleep(10000);
		}
	}


	serialClose(fd);
}

void parse(char line[]){
	switch (minmea_sentence_id(line, false)) {
            case MINMEA_SENTENCE_RMC: {
                struct minmea_sentence_rmc frame;
                if (minmea_parse_rmc(&frame, line)) {
                    /*
		    printf("$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                            frame.latitude.value, frame.latitude.scale,
                            frame.longitude.value, frame.longitude.scale,
                            frame.speed.value, frame.speed.scale);
                    printf("$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                            minmea_rescale(&frame.latitude, 1000),
                            minmea_rescale(&frame.longitude, 1000),
                            minmea_rescale(&frame.speed, 1000));
		   */
                    printf("$xxRMC floating point degree coordinates and speed: (%f,%f) %f, %f degree\n",
                            minmea_tocoord(&frame.latitude),
                            minmea_tocoord(&frame.longitude),
                            minmea_tofloat(&frame.speed),
			    minmea_tofloat(&frame.course));
			data.latitude = minmea_tocoord(&frame.latitude);
                        data.longitude = minmea_tocoord(&frame.longitude);
                        data.heading = minmea_tofloat(&frame.course);
                        data.velocity = minmea_tofloat(&frame.speed);
			printf("$xxRMC floating point degree coordinates and speed: (%f,%f) %f, %f degree\n",
                            data.latitude,
                            data.longitude,
                            data.heading,
                            data.velocity);
			printf("$xxRMC floating point degree coordinates and speed: (%d,%d) %d, %d degree\n",
                            isnan(minmea_tocoord(&frame.latitude)),
                            isnan(minmea_tocoord(&frame.longitude)),
                            isnan(minmea_tofloat(&frame.speed)),
                            isnan(minmea_tofloat(&frame.course)));
                }
                else {
                    printf("$xxRMC sentence is not parsed\n");
                }
            } break;
	/*
	case MINMEA_SENTENCE_GGA: {
                struct minmea_sentence_gga frame;
                if (minmea_parse_gga(&frame, line)) {
                    printf("$xxGGA: fix quality: %d\n", frame.fix_quality);
                }
                else {
                    printf("$xxGGA sentence is not parsed\n");
                }
            } break;
	*/
            case MINMEA_SENTENCE_GST: {
                struct minmea_sentence_gst frame;
                if (minmea_parse_gst(&frame, line)) {
                    printf("$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
                            frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
                            frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
                            frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
                    printf("$xxGST fixed point latitude,longitude and altitude error deviation"
                           " scaled to one decimal place: (%d,%d,%d)\n",
                            minmea_rescale(&frame.latitude_error_deviation, 10),
                            minmea_rescale(&frame.longitude_error_deviation, 10),
                            minmea_rescale(&frame.altitude_error_deviation, 10));
                    printf("$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                            minmea_tofloat(&frame.latitude_error_deviation),
                            minmea_tofloat(&frame.longitude_error_deviation),
                            minmea_tofloat(&frame.altitude_error_deviation));
                }
                else {
                    printf("$xxGST sentence is not parsed\n");
                }
            } break;
	/*
	case MINMEA_SENTENCE_GSV: {
                struct minmea_sentence_gsv frame;
                if (minmea_parse_gsv(&frame, line)) {
                    printf("$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                    printf("$xxGSV: sattelites in view: %d\n", frame.total_sats);
                    for (int i = 0; i < 4; i++)
                        printf("$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                            frame.sats[i].nr,
                            frame.sats[i].elevation,
                            frame.sats[i].azimuth,
                            frame.sats[i].snr);
                }
                else {
                    printf("$xxGSV sentence is not parsed\n");
                }
            } break;
	*/
	case MINMEA_SENTENCE_VTG: {
               struct minmea_sentence_vtg frame;
               if (minmea_parse_vtg(&frame, line)) {
                    printf("$xxVTG: true track degrees = %f\n",
                           minmea_tofloat(&frame.true_track_degrees));
                    printf("        magnetic track degrees = %f\n",
                           minmea_tofloat(&frame.magnetic_track_degrees));
                    printf("        speed knots = %f\n",
                            minmea_tofloat(&frame.speed_knots));
                    printf("        speed kph = %f\n",
                            minmea_tofloat(&frame.speed_kph));
               }
               else {
                    printf("$xxVTG sentence is not parsed\n");
               }
            } break;
	/*
	case MINMEA_SENTENCE_ZDA: {
                struct minmea_sentence_zda frame;
                if (minmea_parse_zda(&frame, line)) {
                    printf("$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
                           frame.time.hours,
                           frame.time.minutes,
                           frame.time.seconds,
                           frame.date.day,
                           frame.date.month,
                           frame.date.year,
                           frame.hour_offset,
                           frame.minute_offset);
                }
                else {
                    printf("$xxZDA sentence is not parsed\n");
                }
            } break;
*/
            case MINMEA_INVALID: {
                printf("$xxxxx sentence is not valid\n");
            } break;

            default: {
                //printf("$xxxxx sentence is not parsed\n");
            } break;
        }
}
