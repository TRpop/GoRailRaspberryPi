#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <nmea.h>
#include <nmea/gpgll.h>
#include <nmea/gpgga.h>

int
main(void)
{

	// Sentence string to be parsed
	char sentence[] = "$GPGLL,4916.45,N,12311.12,W,225444,A,*1D\n\n";

	// Pointer to struct containing the parsed data
	nmea_s *data;

	// Parse it...
	data = nmea_parse(sentence, strlen(sentence), 0);
	if (NULL == data) {
		exit(EXIT_FAILURE);
	}

	if (NMEA_GPGGA == data->type) {
	nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;

	printf("GPGGA Sentence\n");
	printf("Number of satellites: %d\n", gpgga->n_satellites);
	printf("Altitude: %d %c\n", gpgga->altitude, gpgga->altitude_unit);
}

if (NMEA_GPGLL == data->type) {
	nmea_gpgll_s *gpgll = (nmea_gpgll_s *) data;

	printf("GPGLL Sentence\n");
	printf("Longitude:\n");
	printf("  Degrees: %d\n", gpgll->longitude.degrees);
	printf("  Minutes: %f\n", gpgll->longitude.minutes);
	printf("  Cardinal: %c\n", (char) gpgll->longitude.cardinal);
	printf("Latitude:\n");
	printf("  Degrees: %d\n", gpgll->latitude.degrees);
	printf("  Minutes: %f\n", gpgll->latitude.minutes);
	printf("  Cardinal: %c\n", (char) gpgll->latitude.cardinal);
}
	nmea_free(data);
	return 0;
}
