#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>
#include <bluetooth/rfcomm.h>
#include <vector>
#include <sstream>
#include <math.h>
#include <algorithm>
#include <utility>

#include <cmath>
#include <stdexcept>

#include <signal.h>

#include <iostream>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>

#include "minmea.h"
#include "matrix.h"
#include "kalman.h"
#include "UTM.h"

#define DELAY_US 100

#define Wa 6378137
#define We 0.081819190842965

using namespace std;

typedef pair<double, double> Point;

typedef struct {
	double lat, lng;
} LatLng;

typedef double Distance;

typedef double Temperature;

typedef struct {
        double x, y, heading, velocity;
	char zone[10];
} utm_data_t;

typedef struct {
	LatLng ll;
	utm_data_t utm;
	Distance distance;
	Temperature l_temp, r_temp;
} DrivingData;

typedef struct {
        float latitude, longitude, heading, velocity;
} gps_data_t;


enum CarState {MOVING, STOPED, STATE_ERROR};
enum SystemState {OPERATING = 1, HEADING_FRONT = 2, HEADING_BACK = 4, NOT_OPERATING = 8, PENDING_GPS = 16};

//////////////////////////////Functions//////////////////////////////
void alarmWakeup(int sig_num);
string make_string(int from, int to, vector<DrivingData>& vec, char lr);
short read_raw_data(int fd, int addr);
int _str2uuid( const char *uuid_str, uuid_t *uuid );
sdp_session_t *register_service(uint8_t rfcomm_channel);
int init_server(int& client);
char *read_server(int client);
void write_server(int client, const char *message, int length);
CarState decodeCarState(short state);
double decodeDelta(int Encoded);
void parse(char[], gps_data_t &data);
void update_distance(int, Temperature, Temperature);
void update_distance_back(int, DrivingData&);
void getCheckPoint(vector<DrivingData> &save, vector<DrivingData> &right_check, vector<DrivingData> &left_check);
void init();
bool isNear(DrivingData& a, DrivingData& b);
bool isNear(vector<DrivingData>& a, DrivingData& b);
void sortByDistance(vector<DrivingData> &object);
void sortByRightTemparature(vector<DrivingData> &object);
void sortByLeftTemparature(vector<DrivingData> &object);
void sortByTemparature(vector<DrivingData> &right, vector<DrivingData> &left);
//////////////////////////////////////////////////////////////////////

#define Left_Thermo     0x5A
#define Right_Thermo    0x5B
#define Arduino         42

//////Thermo Command//////////////
#define MLX90614_TA     0x06
#define MLX90614_TOBJ1  0x07

//////Arduino Command////////////
#define GET_ENCODER     0x01
#define GO_FRONT        0x02
#define GO_BACK         0x04
#define STOP            0x08
#define OPEN_VALVE      0x10
#define CLOSE_VALVE     0x20
#define GET_STATE       0x40
#define SET_DELTA       0xF0

#define ERROR           0xFFFF

#define CONTROL_PERIOD  0.1

///////System info///////////////
#define WHEELRATIO      0.035
#define GEAR_RATIO      22.0
#define PPR             13.0

//#define ENCODER2SPEED(x)	(x/(GEAR_RATIO * PPR))*2*pi*WHEEL_RADIUS/CONTROL_PERIOD
#define ENCODER2METER(x) ((double)x/(GEAR_RATIO * PPR))*2*M_PI*WHEELRATIO

#define K_SIZE 5

#define TR_OFFSET 0.9
#define TL_OFFSET 0.9
////////////////Global Variable////////////////
char input[1024] = { 0 };

int leftThermo;
int rightThermo;
volatile int arduino;

double travelDistance = 0.0;
double targetDistance = 0.0;
int deltaEncoded = 0;
double delta = 0.0;

bdaddr_t bdaddr_any = {0, 0, 0, 0, 0, 0};
bdaddr_t bdaddr_local = {0, 0, 0, 0xff, 0xff, 0xff};

CarState carState = STOPED;
SystemState systemState = NOT_OPERATING;

int gps, gpsCounter, i;
int flag = 0;
char msg[MINMEA_MAX_LENGTH] = {0, };
char ch;
gps_data_t gps_data;
utm_data_t utm_data;

int cnt = -1;

vector<DrivingData> data, r_check, l_check;

DrivingData currentLoc, starting_point;

Kalman filter;
Matrix X(3, 1), X_pre(3, 1), P(3, 3);
////////////////////////////////////////////////////

int main()
{

        gps = serialOpen("/dev/ttyUSB0", 9600);
        if(gps < 0) {
                printf("gps connect failed\n");
                return 1;
        }

        leftThermo = wiringPiI2CSetup(Left_Thermo); /*Initializes I2C with device Address*/
        rightThermo = wiringPiI2CSetup(Right_Thermo); /*Initializes I2C with device Address*/
        arduino = wiringPiI2CSetup(Arduino);

        if(leftThermo < 0 || rightThermo < 0) {
                printf("i2c connect failed\n");
                return 1;
        }

        if(arduino < 0) {
                printf("arduino connect failed\n");
                return 1;
        }

        int sock;

        int client = init_server(sock);

        signal(SIGALRM, alarmWakeup);
        ualarm(CONTROL_PERIOD * 1000000, CONTROL_PERIOD * 1000000);

        while(1)
        {
                //char input[1024] = {0, };
                char *recv_message = read_server(client);
                if ( recv_message == NULL ) {
                        //if(input == NULL)
                        printf("client disconnected\n");
                        usleep(1000000);

                        close(client);

                        struct sockaddr_rc rem_addr = { 0 };

                        socklen_t opt = sizeof(rem_addr);

                        client = accept(sock, (struct sockaddr *)&rem_addr, &opt);
                        if(client == -1) {
                                continue;
                        }
                        printf("accept() returned %d\n", client);


                }else{
                        string recv(recv_message);

                        if(recv.find("@#", 0) != std::string::npos) {
                                if(recv.find("g", 0) != std::string::npos) { //graph
                                        //if(r_save.size() != 0) {
                                        //        RamerDouglasPeucker(r_save, 0.03, r_save);
                                        //        RamerDouglasPeucker(l_save, 0.03, l_save);
                                        //}
                                        string temp = "@#";
                                        temp += "r";
                                        temp += "n" + to_string(data.size());
                                        write_server(client, temp.c_str(), temp.length());
                                        usleep(DELAY_US);

                                        for(int i = 0; temp.find("&*") == string::npos; i++) {
                                                temp = make_string(i*50, (i+1)*50, data, 'r');
                                                write_server(client, temp.c_str(), temp.length());
                                                usleep(DELAY_US);
                                        }

                                        usleep(100000);

                                        temp = "@#";
                                        temp += "l";
                                        temp += "n" + to_string(data.size());
                                        write_server(client, temp.c_str(), temp.length());
                                        usleep(DELAY_US);

                                        for(int i = 0; temp.find("&*") == string::npos; i++) {
                                                temp = make_string(i*50, (i+1)*50, data, 'l');
                                                write_server(client, temp.c_str(), temp.length());
                                                usleep(DELAY_US);
                                        }

                                }else if(recv.find("de", 0) != std::string::npos) { //delta
                                        int st = recv.find("de", 0) + 2;
                                        int end = recv.find("b", 0);

                                        deltaEncoded = stoi(recv.substr(st, end - st));
                                        delta = decodeDelta(deltaEncoded);

                                        printf("\ndelta = %g\n", delta);

                                        int i2c_recv;

					/*
                                        do {
                                                i2c_recv = read_raw_data(arduino, (deltaEncoded | 0xF0));
                                                usleep(DELAY_US);
                                        } while(i2c_recv != deltaEncoded);
					*/

					if(read_raw_data(arduino, (deltaEncoded | 0xF0)) == deltaEncoded) printf("delta received : %d\n", deltaEncoded);
					else printf("delta write error\n");


                                }else if(recv.find("di", 0) != std::string::npos) { //distance
                                        int st = recv.find("di", 0) + 2;
                                        int end = recv.find("b", 0);

                                        targetDistance = stod(recv.substr(st, end - st));

                                        printf("\ntarget distance = %g\n", targetDistance);

                                }else if(recv.find("s", 0) != std::string::npos) { //start
					init();
                                        systemState = static_cast<SystemState>(PENDING_GPS | HEADING_FRONT);
                                }else if(recv.find("p", 0) != std::string::npos) {
                                        systemState = static_cast<SystemState>(NOT_OPERATING);
                                }
                        }else{
                                write_server(client, recv_message, 10);
                        }
                }

                //write_server(client, recv_message, strlen(recv_message));
        }
}

void alarmWakeup(int sig_num)
{
        if(sig_num == SIGALRM) {

                /////////////////////fetch gps data///////////////
		while(serialDataAvail(gps)) {
                        ch = serialGetchar(gps);

                        if(flag == 0) {
                                if(ch == '$') {
                                        flag = 1;
                                        i = 0;
                                        msg[i] = ch;
                                        i++;
                                }
                        }else{
                                msg[i] = ch;
                                i++;
                                if(ch == '\n') {
                                        msg[i] = '\0';
                                        parse(msg, gps_data);
                                        //WGS2UTM(gps_data.latitude, gps_data.longitude, utm_data.x, utm_data.y);
                                        flag = 0;
                                }
                        }
                }
                //////////////////////////////

                /////////////////////control///////////////////
                if((systemState & OPERATING) == OPERATING) {    //OPERATING
                        if((systemState & HEADING_FRONT) == HEADING_FRONT) {    //HEADING FRONT

                                if(travelDistance >= targetDistance) {  //
                                        systemState = static_cast<SystemState>(OPERATING | HEADING_BACK);

                                        r_check.clear();
                                        l_check.clear();
                                        getCheckPoint(data, r_check, l_check);

					sortByTemparature(r_check, l_check);

					if(r_check.size() > 3) r_check.resize(3);
                                        if(l_check.size() > 3) l_check.resize(3);
					sortByDistance(r_check);
					sortByDistance(l_check);

                                }else{
                                        carState = decodeCarState(read_raw_data(arduino, GET_STATE));
                                        usleep(DELAY_US);
                                        if(carState == STATE_ERROR) { //connection error
                                                printf("Cannot get car state : arduino connection error\n");
                                        }else{
                                                if(carState == STOPED) {  //if car stoped
                                                        double left_temp, right_temp;
                                                        //Data Fetch
                                                        left_temp = read_raw_data(leftThermo, MLX90614_TOBJ1)*0.02-273.15;//Temperature conversion
                                                        usleep(DELAY_US);
                                                        right_temp = read_raw_data(rightThermo, MLX90614_TOBJ1)*0.02-273.15;//Temperature conversion
                                                        usleep(DELAY_US);
                                                        int encoder = read_raw_data(arduino, GET_ENCODER);

							left_temp += TL_OFFSET;
							right_temp += TR_OFFSET;

                                                        //refine situation, update distance
                                                        //refineSituation()
                                                        update_distance(encoder, left_temp, right_temp);
                                                        //////////////////////

                                                        //Command
							/*
                                                        while(read_raw_data(arduino, GO_FRONT) != GO_FRONT){
								printf("go front\n");
								usleep(DELAY_US);
							}*/

							while(read_raw_data(arduino, GO_FRONT) != GO_FRONT) {usleep(DELAY_US);}

                                                        ///////////////


                                                        //Save Data
                                                        //printf("left : %g\tright : %g\tdistance : %lf\n", left_temp, right_temp, travelDistance);

                                                        //step += 0.5;

                                                        //r_save.push_back(make_pair(travelDistance, right_temp));
                                                        //l_save.push_back(make_pair(travelDistance, left_temp));
                                                        ///////////////

                                                }else{  //if car is moving

                                                }
                                        }
                                }


                        }else if((systemState & HEADING_BACK) == HEADING_BACK) {  //HEADING BACK, should mark
                                //TODO Moving Back
                                if(isNear(currentLoc, starting_point) && cnt == -1) { //
                                        systemState = static_cast<SystemState>(NOT_OPERATING);
                                        travelDistance = 0;
                                        printf("\nSTOPED\n");
                                }else{
                                        carState = decodeCarState(read_raw_data(arduino, GET_STATE));
                                        usleep(DELAY_US);
                                        if(carState == STATE_ERROR) { //connection error
                                                printf("Cannot get car state : arduino connection error\n");
                                        }else{
                                                if(carState == STOPED) {  //if car stoped

                                                        //Data Fetch
                                                        int encoder = read_raw_data(arduino, GET_ENCODER);
							usleep(DELAY_US);
                                                        //refine situation
                                                        //refineSituation()
                                                        //travelDistance = travelDistance - ENCODER2METER(encoder);

							//DrivingData currentLoc;
							if(cnt == -1){
                                                        	update_distance_back(encoder, currentLoc);
							}
                                                        //////////////////////
                                                        //////////////

							if(!(r_check.empty() || l_check.empty())){
								if(isNear(r_check, currentLoc) || isNear(l_check, currentLoc)){
									while(OPEN_VALVE != read_raw_data(arduino, OPEN_VALVE)){usleep(DELAY_US);}
                                                                        printf("\nopen valve\n");

                                                                        cnt = 0;
								}else{
                                                                        //go back
                                                                        if(cnt > 3){
                                                                                //printf("try closing valve both\n");
                                                                                while(CLOSE_VALVE != read_raw_data(arduino, CLOSE_VALVE)){usleep(DELAY_US);}
                                                                                cnt = -1;
                                                                                printf("\nclose valve\n");
                                                                        }else if(cnt == -1){
                                                                                //printf("go back both\n");
                                                                                while(GO_BACK != read_raw_data(arduino, GO_BACK)){usleep(DELAY_US);}
                                                                                //printf("go back end\n");
                                                                        }else{
                                                                                cnt++;
                                                                        }
                                                                }

							}else if(!r_check.empty()){
								if(isNear(r_check, currentLoc)){
                                                                        //solenoid open
                                                                        while(OPEN_VALVE != read_raw_data(arduino, OPEN_VALVE)){usleep(DELAY_US);}
                                                                        printf("\nopen valve\n");

                                                                        cnt = 0;
                                                                }else{
                                                                        //go back
                                                                        if(cnt > 3){
                                                                                //printf("try closing valve right\n");
                                                                                while(CLOSE_VALVE != read_raw_data(arduino, CLOSE_VALVE)){usleep(DELAY_US);}
                                                                                cnt = -1;
                                                                                printf("\nclose valve\n");
                                                                        }else if(cnt == -1){
                                                                                //printf("go back right\n");
                                                                                while(GO_BACK != read_raw_data(arduino, GO_BACK)){usleep(DELAY_US);}
                                                                        }else{
                                                                                cnt++;
                                                                        }
                                                                }
							}else if(!l_check.empty()){
								if(isNear(l_check, currentLoc)){
                                                                        //solenoid open
                                                                        while(OPEN_VALVE != read_raw_data(arduino, OPEN_VALVE)){usleep(DELAY_US);}
                                                                        printf("\nopen valve\n");

                                                                        cnt = 0;
                                                                }else{
                                                                        //go back
                                                                        if(cnt > 3){
                                                                                //printf("try closing valve left");
                                                                                while(CLOSE_VALVE != read_raw_data(arduino, CLOSE_VALVE)){usleep(DELAY_US);}
                                                                                cnt = -1;
                                                                                printf("\nclose valve\n");
                                                                        }else if(cnt == -1){
                                                                                //printf("go back left\n");
                                                                                while(GO_BACK != read_raw_data(arduino, GO_BACK)){usleep(DELAY_US);}
                                                                        }else{
                                                                                cnt++;
                                                                        }
                                                                }
							}else{          //nothing contained
                                                                if(cnt > 3){
                                                                        //printf("try closing valve last");
                                                                        while(CLOSE_VALVE != read_raw_data(arduino, CLOSE_VALVE)){usleep(DELAY_US);}
                                                                        cnt = -1;
                                                                        printf("\nclose valve\n");
                                                                }else if(cnt == -1){
                                                                        //printf("go back last\n");
                                                                        while(GO_BACK != read_raw_data(arduino, GO_BACK)){usleep(DELAY_US);}
                                                                }else{
                                                                        cnt++;
                                                                }
                                                                //while(GO_BACK != read_raw_data(arduino, GO_BACK)){usleep(DELAY_US);}
                                                        }


                                                        //Save Data
                                                        //printf("left : %g\tright : %g\tencoder : %d\n", left_temp, right_temp, encoder);
                                                        //printf("%lf\n", travelDistance);
                                                        //step += 0.5;

                                                        //v.push_back(make_pair(step, (left_temp + right_temp)/2));
                                                        ///////////////

                                                }else{  //if car is moving

                                                }
                                        }
                                }

                        }




                }else if((systemState & NOT_OPERATING) == NOT_OPERATING) {

                }else if((systemState & PENDING_GPS) == PENDING_GPS) {
			printf("lat : %f\n", gps_data.latitude);
			if(!isnan(gps_data.latitude)){
				init();
				systemState = static_cast<SystemState>(OPERATING | HEADING_FRONT);
			}
                }
        }

}

double decodeDelta(int Encoded){
        switch(Encoded) {
        case 1:
                return 0.1;
        case 2:
                return 0.5;
        case 3:
                return 1.0;
        case 4:
                return 2.0;
        default:
                return 0;
        }
}

CarState decodeCarState(short state){
	//printf("%d", state);
        switch(state) {
        case 1:
                return STOPED;
        case 2:
                return MOVING;
        default:
                return STATE_ERROR;
        }
}

string make_string(int from, int to, vector<DrivingData>& vec, char lr){
        string temp = "";
	if(lr == 'r'){
	        if(to > vec.size()) {
        	        for(int i = from; i < vec.size(); i++) {
                	        string x = to_string(vec.at(i).distance);
                	        string y = to_string(vec.at(i).r_temp);
                	        x.erase ( x.find_last_not_of('0') + 1, string::npos );
                	        y.erase ( y.find_last_not_of('0') + 1, std::string::npos );
                	        temp += "x" + x + "y" + y;
                	}
                	temp += "x&*b\n";
        	}else{
                	for(int i = from; i < to; i++) {
                        	string x = to_string(vec.at(i).distance);
                        	string y = to_string(vec.at(i).r_temp);
                        	x.erase ( x.find_last_not_of('0') + 1, string::npos );
                        	y.erase ( y.find_last_not_of('0') + 1, std::string::npos );
                        	temp += "x" + x + "y" + y;
                	}
                	temp += "xb\n";
        	}
	}else if(lr == 'l'){
		if(to > vec.size()) {
                        for(int i = from; i < vec.size(); i++) {
                                string x = to_string(vec.at(i).distance);
                                string y = to_string(vec.at(i).l_temp);
                                x.erase ( x.find_last_not_of('0') + 1, string::npos );
                                y.erase ( y.find_last_not_of('0') + 1, std::string::npos );
                                temp += "x" + x + "y" + y;
                        }
                        temp += "x&*b\n";
                }else{
                        for(int i = from; i < to; i++) {
                                string x = to_string(vec.at(i).distance);
                                string y = to_string(vec.at(i).l_temp);
                                x.erase ( x.find_last_not_of('0') + 1, string::npos );
                                y.erase ( y.find_last_not_of('0') + 1, std::string::npos );
                                temp += "x" + x + "y" + y;
                        }
                        temp += "xb\n";
                }
	}
        //temp += "x&*b\n";

        return temp;
}

short read_raw_data(int fd, int addr){
        return wiringPiI2CReadReg16(fd, addr);
}

int _str2uuid( const char *uuid_str, uuid_t *uuid ) {
        /* This is from the pybluez stack */

        uint32_t uuid_int[4];
        char *endptr;

        if( strlen( uuid_str ) == 36 ) {
                char buf[9] = { 0 };

                if( uuid_str[8] != '-' && uuid_str[13] != '-' &&
                    uuid_str[18] != '-' && uuid_str[23] != '-' ) {
                        return 0;
                }
                // first 8-bytes
                strncpy(buf, uuid_str, 8);
                uuid_int[0] = htonl( strtoul( buf, &endptr, 16 ) );
                if( endptr != buf + 8 ) return 0;
                // second 8-bytes
                strncpy(buf, uuid_str+9, 4);
                strncpy(buf+4, uuid_str+14, 4);
                uuid_int[1] = htonl( strtoul( buf, &endptr, 16 ) );
                if( endptr != buf + 8 ) return 0;

                // third 8-bytes
                strncpy(buf, uuid_str+19, 4);
                strncpy(buf+4, uuid_str+24, 4);
                uuid_int[2] = htonl( strtoul( buf, &endptr, 16 ) );
                if( endptr != buf + 8 ) return 0;

                // fourth 8-bytes
                strncpy(buf, uuid_str+28, 8);
                uuid_int[3] = htonl( strtoul( buf, &endptr, 16 ) );
                if( endptr != buf + 8 ) return 0;

                if( uuid != NULL ) sdp_uuid128_create( uuid, uuid_int );
        } else if ( strlen( uuid_str ) == 8 ) {
                // 32-bit reserved UUID
                uint32_t i = strtoul( uuid_str, &endptr, 16 );
                if( endptr != uuid_str + 8 ) return 0;
                if( uuid != NULL ) sdp_uuid32_create( uuid, i );
        } else if( strlen( uuid_str ) == 4 ) {
                // 16-bit reserved UUID
                int i = strtol( uuid_str, &endptr, 16 );
                if( endptr != uuid_str + 4 ) return 0;
                if( uuid != NULL ) sdp_uuid16_create( uuid, i );
        } else {
                return 0;
        }

        return 1;

}



sdp_session_t *register_service(uint8_t rfcomm_channel) {

        /* A 128-bit number used to identify this service. The words are ordered from most to least
         * significant, but within each word, the octets are ordered from least to most significant.
         * For example, the UUID represneted by this array is 00001101-0000-1000-8000-00805F9B34FB. (The
         * hyphenation is a convention specified by the Service Discovery Protocol of the Bluetooth Core
         * Specification, but is not particularly important for this program.)
         *
         * This UUID is the Bluetooth Base UUID and is commonly used for simple Bluetooth applications.
         * Regardless of the UUID used, it must match the one that the Armatus Android app is searching
         * for.
         */
        const char *service_name = "Armatus Bluetooth server";
        const char *svc_dsc = "A HERMIT server that interfaces with the Armatus Android app";
        const char *service_prov = "Armatus";

        uuid_t root_uuid, l2cap_uuid, rfcomm_uuid, svc_uuid,
               svc_class_uuid;
        sdp_list_t *l2cap_list = 0,
                   *rfcomm_list = 0,
                   *root_list = 0,
                   *proto_list = 0,
                   *access_proto_list = 0,
                   *svc_class_list = 0,
                   *profile_list = 0;
        sdp_data_t *channel = 0;
        sdp_profile_desc_t profile;
        sdp_record_t record = { 0 };
        sdp_session_t *session = 0;

        // set the general service ID
        //sdp_uuid128_create(&svc_uuid, &svc_uuid_int);
        _str2uuid("00001101-0000-1000-8000-00805F9B34FB",&svc_uuid);
        sdp_set_service_id(&record, svc_uuid);

        char str[256] = "";
        sdp_uuid2strn(&svc_uuid, str, 256);
        printf("Registering UUID %s\n", str);

        // set the service class
        sdp_uuid16_create(&svc_class_uuid, SERIAL_PORT_SVCLASS_ID);
        svc_class_list = sdp_list_append(0, &svc_class_uuid);
        sdp_set_service_classes(&record, svc_class_list);

        // set the Bluetooth profile information
        sdp_uuid16_create(&profile.uuid, SERIAL_PORT_PROFILE_ID);
        profile.version = 0x0100;
        profile_list = sdp_list_append(0, &profile);
        sdp_set_profile_descs(&record, profile_list);

        // make the service record publicly browsable
        sdp_uuid16_create(&root_uuid, PUBLIC_BROWSE_GROUP);
        root_list = sdp_list_append(0, &root_uuid);
        sdp_set_browse_groups(&record, root_list);

        // set l2cap information
        sdp_uuid16_create(&l2cap_uuid, L2CAP_UUID);
        l2cap_list = sdp_list_append(0, &l2cap_uuid);
        proto_list = sdp_list_append(0, l2cap_list);

        // register the RFCOMM channel for RFCOMM sockets
        sdp_uuid16_create(&rfcomm_uuid, RFCOMM_UUID);
        channel = sdp_data_alloc(SDP_UINT8, &rfcomm_channel);
        rfcomm_list = sdp_list_append(0, &rfcomm_uuid);
        sdp_list_append(rfcomm_list, channel);
        sdp_list_append(proto_list, rfcomm_list);

        access_proto_list = sdp_list_append(0, proto_list);
        sdp_set_access_protos(&record, access_proto_list);

        // set the name, provider, and description
        sdp_set_info_attr(&record, service_name, service_prov, svc_dsc);

        // connect to the local SDP server, register the service record,
        // and disconnect
        session = sdp_connect(&bdaddr_any, &bdaddr_local, SDP_RETRY_IF_BUSY);
        sdp_record_register(session, &record, 0);

        // cleanup
        sdp_data_free(channel);
        sdp_list_free(l2cap_list, 0);
        sdp_list_free(rfcomm_list, 0);
        sdp_list_free(root_list, 0);
        sdp_list_free(access_proto_list, 0);
        sdp_list_free(svc_class_list, 0);
        sdp_list_free(profile_list, 0);

        return session;
}


int init_server(int& sock) {
        int port = 3, result, client, bytes_read, bytes_sent;
        struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
        char buffer[1024] = { 0 };
        socklen_t opt = sizeof(rem_addr);



        // local bluetooth adapter
        loc_addr.rc_family = AF_BLUETOOTH;
        loc_addr.rc_bdaddr = bdaddr_any;
        loc_addr.rc_channel = (uint8_t) port;

        // register service
        sdp_session_t *session = register_service(port);
        // allocate socket
        sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
        printf("socket() returned %d\n", sock);

        // bind socket to port 3 of the first available
        result = bind(sock, (struct sockaddr *)&loc_addr, sizeof(loc_addr));
        printf("bind() on channel %d returned %d\n", port, result);

        // put socket into listening mode
        result = listen(sock, 1);
        printf("listen() returned %d\n", result);

        //sdpRegisterL2cap(port);

        // accept one connection
        printf("calling accept()\n");
        client = accept(sock, (struct sockaddr *)&rem_addr, &opt);
        printf("accept() returned %d\n", client);

        ba2str(&rem_addr.rc_bdaddr, buffer);
        fprintf(stderr, "accepted connection from %s\n", buffer);
        memset(buffer, 0, sizeof(buffer));

        return client;
}

char *read_server(int client) {
        // read data from the client
        int bytes_read;
        bytes_read = read(client, input, sizeof(input));
        if (bytes_read > 0) {
                printf("received [%s]\n%d", input, bytes_read);
                return input;
        } else {
                return NULL;
        }
}

void write_server(int client, const char *message, int length) {
        // send data to the client
        char messageArr[1024] = { 0 };
        int bytes_sent;
        strncpy(messageArr, message, length);

        bytes_sent = write(client, messageArr, strlen(messageArr));
        if (bytes_sent > 0) {
                printf("sent [%s] %d\n", messageArr, bytes_sent);
        }
}

void parse(char line[], gps_data_t &data){
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
			/*
                        printf("$xxRMC floating point degree coordinates and speed: (%f,%f) %f, %f degree\n",
                               minmea_tocoord(&frame.latitude),
                               minmea_tocoord(&frame.longitude),
                               minmea_tofloat(&frame.speed),
                               minmea_tofloat(&frame.course));
			*/
			data.latitude = minmea_tocoord(&frame.latitude);
			data.longitude = minmea_tocoord(&frame.longitude);
			data.heading = minmea_tofloat(&frame.course);
			data.velocity = minmea_tofloat(&frame.speed);
			if(isnan(data.latitude)) printf("It's nan\n");
//			printf("lat : %f,\tlong : %f,\theading : %f degree,\tvel : %f m/s\n", data.latitude, data.longitude, data.heading, data.velocity);
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

void update_distance(int encoder, Temperature left, Temperature right){
	utm_data_t utm_data;
	UTM::LLtoUTM(gps_data.latitude, gps_data.longitude, utm_data.y, utm_data.x, utm_data.zone);

	Matrix Z(3, 1);
	Z(0, 0) = utm_data.x;
	Z(1, 0) = utm_data.y;
	Z(2, 0) = gps_data.heading * M_PI / 180.0;

	X = filter.step(Z, ENCODER2METER(encoder));

	travelDistance += sqrt(pow(X(0, 0) - X_pre(0, 0), 2.0) + pow(X(1, 0) - X_pre(1, 0), 2.0));

	X_pre = X;

	double Lat, Long;

	UTM::UTMtoLL(X(1, 0), X(0, 0), utm_data.zone, Lat, Long);

	DrivingData temp;
	temp.ll.lat = Lat;
	temp.ll.lng = Long;
	temp.utm.x = X(0, 0);
	temp.utm.y = X(1, 0);
	temp.distance = travelDistance;
	temp.l_temp = left;
	temp.r_temp = right;

	data.push_back(temp);
	printf("up = [up; %f %f %lf %f %f %f];\n", temp.ll.lat, temp.ll.lng, travelDistance, left, right, X(2, 0));
}

void update_distance_back(int encoder, DrivingData& current){
	utm_data_t utm_data;
        UTM::LLtoUTM(gps_data.latitude, gps_data.longitude, utm_data.y, utm_data.x, utm_data.zone);

	Matrix Z(3, 1);
        Z(0, 0) = utm_data.x;
        Z(1, 0) = utm_data.y;
        Z(2, 0) = gps_data.heading * M_PI / 180.0;

        X = filter.step(Z, ENCODER2METER(encoder));

        travelDistance -= sqrt(pow(X(0, 0) - X_pre(0, 0), 2.0) + pow(X(1, 0) - X_pre(1, 0), 2.0));

        X_pre = X;

	double Lat, Long;

        UTM::UTMtoLL(X(1, 0), X(0, 0), utm_data.zone, Lat, Long);

	current.ll.lat = Lat;
        current.ll.lng = Long;
        current.utm.x = X(0, 0);
        current.utm.y = X(1, 0);
        current.distance = travelDistance;

	printf("down = [down; %f %f %f];\n", Lat, Long, X(2, 0));
}

void getmax(vector<DrivingData> &vec, int begin, int end, int &maxIndexr, int &maxIndexl){
        vector<DrivingData>::iterator it;
        double rmax = -10000, lmax = -10000;
        int n = 0;
        for(it = vec.begin() + begin; it != vec.begin() + end; it++, n++) {
                if(rmax < it->r_temp) {
                        rmax = it->r_temp;
                        maxIndexr = begin + n;
                }
		if(lmax < it->l_temp) {
                        lmax = it->l_temp;
                        maxIndexl = begin + n;
                }
        }
}

void getCheckPoint(vector<DrivingData> &save, vector<DrivingData> &right_check, vector<DrivingData> &left_check){
        int rmax = 0, lmax = 0;
        int len = save.size();

        for(int i = 0; i < K_SIZE; i++){
                getmax(save, 0, i + K_SIZE, rmax, lmax);
                if(rmax == i){
                        right_check.push_back(save.at(rmax));
                }
                if(lmax == i){
                        left_check.push_back(save.at(lmax));
                }
        }

        for(int i = K_SIZE; i < (len - K_SIZE); i++){
                getmax(save, i - K_SIZE + 1, i + K_SIZE, rmax, lmax);
                if(rmax == i){
                        right_check.push_back(save.at(rmax));
                }
                if(lmax == i){
                        left_check.push_back(save.at(lmax));
                }
        }

        for(int i = len - K_SIZE; i < len; i++){
                getmax(save, i - K_SIZE + 1, len, rmax, lmax);
                if(rmax == i){
                        right_check.push_back(save.at(rmax));
                }
                if(lmax == i){
                        left_check.push_back(save.at(lmax));
                }
        }
}

void init(){
	printf("init started\n");
        r_check.clear();
        l_check.clear();
	data.clear();

	utm_data_t utm_data;
        UTM::LLtoUTM(gps_data.latitude, gps_data.longitude, utm_data.y, utm_data.x, utm_data.zone);

	X(0, 0) = utm_data.x;
	X(1, 0) = utm_data.y;
	X(2, 0) = gps_data.heading * M_PI / 180.0;
	X_pre = X;

	for(int i = 0; i < 3; i++){
		P(i, i) = 1;
	}

	filter.init(X, P);

	cnt = -1;

	starting_point.ll.lat = gps_data.latitude;
	starting_point.ll.lng = gps_data.longitude;
	travelDistance = 0;

	printf("init ended\n");
}

void sortByDistance(vector<DrivingData> &object){
	sort(object.begin(), object.end(), [] (DrivingData const& a, DrivingData const& b) { return a.distance < b.distance; });
}

void sortByTemparature(vector<DrivingData> &right, vector<DrivingData> &left){
	sort(right.begin(), right.end(), [] (DrivingData const& a, DrivingData const& b) { return a.r_temp > b.r_temp; });
	sort(left.begin(), left.end(), [] (DrivingData const& a, DrivingData const& b) { return a.l_temp > b.l_temp; });
}

bool isNear(DrivingData& a, DrivingData& b){
	return UTM::distanceEarth(a.ll.lat, a.ll.lng, b.ll.lat, b.ll.lng) < delta*1.5;
}

bool isNear(vector<DrivingData>& a, DrivingData& b){
        vector<DrivingData>::iterator it;
        bool flag = false;
        for(it = a.begin(); it != a.end();) {
                if (UTM::distanceEarth(it->ll.lat, it->ll.lng, b.ll.lat, b.ll.lng) < delta){
                        it = a.erase(it);
                        flag = true;
                }else{
                        ++it;
                }
        }
        return flag;
}

