#include <stdio.h>
#include <stdlib.h>
#include <gps.h>

int main(void) {
    // Open the communication
    gps_init();

    loc_t data;

    while (1) {
        gps_location(&data);

        // You can extends adding data.speed, data.altitude, data.course (the direction)
        printf("%lf %lf\n", data.latitude, data.longitude);
    }

    return EXIT_SUCCESS;
}

