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

#include <utility>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <cmath>
#include <stdexcept>

#include <signal.h>

#include <iostream>

#define DELAY_US 100

using namespace std;

typedef pair<double, double> Point;

enum CarState {MOVING, STOPED, STATE_ERROR};
enum SystemState {OPERATING = 1, HEADING_FRONT = 2, HEADING_BACK = 4, NOT_OPERATING = 8};

//////////////////////////////Functions//////////////////////////////
void alarmWakeup(int sig_num);
string make_string(int, int, vector<Point >&);
double PerpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd);
void RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out);
short read_raw_data(int fd, int addr);
int _str2uuid( const char *uuid_str, uuid_t *uuid );
sdp_session_t *register_service(uint8_t rfcomm_channel);
int init_server(int& client);
char *read_server(int client);
void write_server(int client, const char *message, int length);
CarState decodeState(short state);
double decodeDelta(int Encoded);
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
#define WHEEL_RADIUS    0.035
#define GEAR_RATIO      22.0
#define PPR             13.0

//#define ENCODER2SPEED(x)	(x/(GEAR_RATIO * PPR))*2*pi*WHEEL_RADIUS/CONTROL_PERIOD

////////////////Global Variable////////////////
char input[1024] = { 0 };

int connected = 0;

int leftThermo;
int rightThermo;
int arduino;

int encoder;

vector<Point > v;
double step = 0.0;

double targetDistance = 0.0;
int deltaEncoded = 0;
double delta = 0.0;

bdaddr_t bdaddr_any = {0, 0, 0, 0, 0, 0};
bdaddr_t bdaddr_local = {0, 0, 0, 0xff, 0xff, 0xff};

CarState carState = STOPED;
SystemState systemState = NOT_OPERATING;
////////////////////////////////////////////////////

int main()
{
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

                        int port = 3, result, client, bytes_read, bytes_sent;
                        struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
                        char buffer[1024] = { 0 };
                        socklen_t opt = sizeof(rem_addr);

                        sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
                        printf("socket() returned %d\n", sock);


                        loc_addr.rc_family = AF_BLUETOOTH;
                        loc_addr.rc_bdaddr = bdaddr_any;
                        loc_addr.rc_channel = (uint8_t) port;


                        //const int t = 1;
                        //setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &t, sizeof(int));
                        //bdaddr_any = {0, 0, 0, 0, 0, 0};
                        //shutdown(sock, SHUT_RDWR);
                        close(sock);
                        //client = init_server(sock);

                        client = accept(sock, (struct sockaddr *)&rem_addr, &opt);
                        printf("accept() returned %d\n", client);

                        ba2str(&rem_addr.rc_bdaddr, buffer);
                        fprintf(stderr, "accepted connection from %s\n", buffer);
                        memset(buffer, 0, sizeof(buffer));


                }else{
                        string recv(recv_message);

                        if(recv.find("@#", 0) != std::string::npos) {
                                if(recv.find("g", 0) != std::string::npos) { //graph

                                        RamerDouglasPeucker(v, 0.03, v);
                                        string temp = "@#";
                                        temp += "n" + to_string(v.size());
                                        write_server(client, temp.c_str(), temp.length());
                                        usleep(DELAY_US);

                                        for(int i = 0; temp.find("&*") == string::npos; i++) {
                                                temp = make_string(i*50, (i+1)*50, v);
                                                write_server(client, temp.c_str(), temp.length());
                                                usleep(DELAY_US);
                                        }

                                }else if(recv.find("de", 0) != std::string::npos) { //delta
                                        int st = recv.find("de", 0) + 2;
                                        int end = recv.find("b", 0);

                                        deltaEncoded = stoi(recv.substr(st, end - st));
                                        delta = decodeDelta(deltaEncoded);

                                        int i2c_recv;

                                        do {
                                                i2c_recv = read_raw_data(arduino, delta & SET_DELTA);
                                                usleep(DELAY_US);
                                        } while(i2c_recv != delta);


                                }else if(recv.find("di", 0) != std::string::npos) { //distance
                                        int st = recv.find("di", 0) + 2;
                                        int end = recv.find("b", 0);

                                        targetDistance = stod(recv.substr(st, end - st));
                                }else if(recv.find("s", 0) != std::string::npos) { //start
                                        systemState = OPERATING | HEADING_FRONT;
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
                if(systemState & OPERATING == OPERATING) {    //OPERATING
                        if(systemState & HEADING_FRONT == HEADING_FRONT) {    //HEADING FRONT

                                if(distance >= targetDistance) {  //
                                        systemState = OPERATING | HEADING_BACK;
                                }else{
                                        carState = decodeState(read_raw_data(arduino, GET_STATE));
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
                                                        encoder = read_raw_data(arduino, GET_ENCODER);

                                                        //refine situation
                                                        refineSituation()

                                                        //////////////////////

                                                        //Command


                                                        ///////////////


                                                        //Save Data
                                                        printf("left : %g\tright : %g\tencoder : %d\n", left_temp, right_temp, encoder);

                                                        step += 0.5;

                                                        v.push_back(make_pair(step, (left_temp + right_temp)/2));
                                                        ///////////////

                                                }else{  //if car is moving

                                                }
                                        }
                                }


                        }else if(systemState & HEADING_BACK == HEADING_BACK) {  //HEADING BACK, should mark
                                //TODO Moving Back
                        }





                }else if(systemState & NOT_OPERATING == NOT_OPERATING) {

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

State decodeCarState(short state){
        switch(state) {
        case 1:
                return STOPED;
        case 2:
                return MOVING;
        default:
                return STATE_ERROR;
        }
}

string make_string(int from, int to, vector<Point >& vec){
        string temp = "";
        if(to > vec.size()) {
                for(int i = from; i < vec.size(); i++) {
                        string x = to_string(vec.at(i).first);
                        string y = to_string(vec.at(i).second);
                        x.erase ( x.find_last_not_of('0') + 1, string::npos );
                        y.erase ( y.find_last_not_of('0') + 1, std::string::npos );
                        temp += "x" + x + "y" + y;
                }
                temp += "x&*b\n";
        }else{
                for(int i = from; i < to; i++) {
                        string x = to_string(vec.at(i).first);
                        string y = to_string(vec.at(i).second);
                        x.erase ( x.find_last_not_of('0') + 1, string::npos );
                        y.erase ( y.find_last_not_of('0') + 1, std::string::npos );
                        temp += "x" + x + "y" + y;
                }
                temp += "xb\n";
        }

        //temp += "x&*b\n";

        return temp;
}

double PerpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd)
{
        double dx = lineEnd.first - lineStart.first;
        double dy = lineEnd.second - lineStart.second;

        //Normalise
        double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
        if(mag > 0.0)
        {
                dx /= mag; dy /= mag;
        }

        double pvx = pt.first - lineStart.first;
        double pvy = pt.second - lineStart.second;

        //Get dot product (project pv onto normalized direction)
        double pvdot = dx * pvx + dy * pvy;

        //Scale line direction vector
        double dsx = pvdot * dx;
        double dsy = pvdot * dy;

        //Subtract this from pv
        double ax = pvx - dsx;
        double ay = pvy - dsy;

        return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out)
{
        if(pointList.size()<2)
                throw invalid_argument("Not enough points to simplify");

        // Find the point with the maximum distance from line between start and end
        double dmax = 0.0;
        size_t index = 0;
        size_t end = pointList.size()-1;
        for(size_t i = 1; i < end; i++)
        {
                double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
                if (d > dmax)
                {
                        index = i;
                        dmax = d;
                }
        }

        // If max distance is greater than epsilon, recursively simplify
        if(dmax > epsilon)
        {
                // Recursive call
                vector<Point> recResults1;
                vector<Point> recResults2;
                vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
                vector<Point> lastLine(pointList.begin()+index, pointList.end());
                RamerDouglasPeucker(firstLine, epsilon, recResults1);
                RamerDouglasPeucker(lastLine, epsilon, recResults2);

                // Build the result list
                out.assign(recResults1.begin(), recResults1.end()-1);
                out.insert(out.end(), recResults2.begin(), recResults2.end());
                if(out.size()<2)
                        throw runtime_error("Problem assembling output");
        }
        else
        {
                //Just return start and end points
                out.clear();
                out.push_back(pointList[0]);
                out.push_back(pointList[end]);
        }
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
