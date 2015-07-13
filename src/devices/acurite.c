#include "rtl_433.h"

// buffer to hold localized timestamp YYYY-MM-DD HH:MM:SS
#define LOCAL_TIME_BUFLEN	32

// ** Acurite 5n1 functions **

const float acurite_winddirections[] =
    { 337.5, 315.0, 292.5, 270.0, 247.5, 225.0, 202.5, 180,
      157.5, 135.0, 112.5, 90.0, 67.5, 45.0, 22.5, 0.0 };

static int acurite_raincounter = 0;

static int acurite_crc(uint8_t row[BITBUF_COLS], int cols) {
    // sum of first n-1 bytes modulo 256 should equal nth byte
    int i;
    int sum = 0;
    for ( i=0; i < cols; i++)
        sum += row[i];
    if ( sum % 256 == row[cols] )
        return 1;
    else
        return 0;
}

static int acurite_detect(uint8_t *pRow) {
    int i;
    if ( pRow[0] != 0x00 ) {
        // invert bits due to wierd issue
        for (i = 0; i < 8; i++)
            pRow[i] = ~pRow[i] & 0xFF;
        pRow[0] |= pRow[8];  // fix first byte that has mashed leading bit

        if (acurite_crc(pRow, 7))
            return 1;  // passes crc check
    }
    return 0;
}

static float acurite_getTemp (uint8_t highbyte, uint8_t lowbyte) {
    // range -40 to 158 F
    int highbits = (highbyte & 0x0F) << 7 ;
    int lowbits = lowbyte & 0x7F;
    int rawtemp = highbits | lowbits;
    float temp = (rawtemp - 400) / 10.0;
    return temp;
}

static int acurite_getWindSpeed (uint8_t highbyte, uint8_t lowbyte) {
    // range: 0 to 159 kph
	// TODO: sensor does not seem to be in kph, e.g.,
	// a value of 49 here was registered as 41 kph on base unit
	// value could be rpm, etc which may need (polynomial) scaling factor??
	int highbits = ( highbyte & 0x1F) << 3;
    int lowbits = ( lowbyte & 0x70 ) >> 4;
    int speed = highbits | lowbits;
    return speed;
}

static float acurite_getWindDirection (uint8_t byte) {
    // 16 compass points, ccw from (NNW) to 15 (N)
    int direction = byte & 0x0F;
    return acurite_winddirections[direction];
}

static int acurite_getHumidity (uint8_t byte) {
    // range: 1 to 99 %RH
    int humidity = byte & 0x7F;
    return humidity;
}

static int acurite_getRainfallCounter (uint8_t hibyte, uint8_t lobyte) {
    // range: 0 to 99.99 in, 0.01 in incr., rolling counter?
	int raincounter = ((hibyte & 0x7f) << 7) | (lobyte & 0x7F);
    return raincounter;
}

static int acurite5n1_callback(uint8_t bb[BITBUF_ROWS][BITBUF_COLS],int16_t bits_per_row[BITBUF_ROWS]) {
    // acurite 5n1 weather sensor decoding for rtl_433
    // Jens Jensen 2014
    int i;
    uint8_t *buf = NULL;
    // run through rows til we find one with good crc (brute force)
    for (i=0; i < BITBUF_ROWS; i++) {
        if (acurite_detect(bb[i])) {
            buf = bb[i];
            break; // done
        }
    }

    if (buf) {
        // decode packet here
        fprintf(stdout, "Detected Acurite 5n1 sensor, %d bits\n",bits_per_row[1]);
        if (debug_output) {
            for (i=0; i < 8; i++)
                fprintf(stdout, "%02X ", buf[i]);
            fprintf(stdout, "CRC OK\n");
        }

        if ((buf[2] & 0x0F) == 1) {
            // wind speed, wind direction, rainfall

            float rainfall = 0.00;
            int raincounter = acurite_getRainfallCounter(buf[5], buf[6]);
            if (acurite_raincounter > 0) {
                // track rainfall difference after first run
                rainfall = ( raincounter - acurite_raincounter ) * 0.01;
            } else {
                // capture starting counter
                acurite_raincounter = raincounter;
            }

            fprintf(stdout, "wind speed: %d kph, ",
                acurite_getWindSpeed(buf[3], buf[4]));
            fprintf(stdout, "wind direction: %0.1f°, ",
                acurite_getWindDirection(buf[4]));
            fprintf(stdout, "rain gauge: %0.2f in.\n", rainfall);

        } else if ((buf[2] & 0x0F) == 8) {
            // wind speed, temp, RH
            fprintf(stdout, "wind speed: %d kph, ",
                acurite_getWindSpeed(buf[3], buf[4]));
            fprintf(stdout, "temp: %2.1f° F, ",
                acurite_getTemp(buf[4], buf[5]));
            fprintf(stdout, "humidity: %d%% RH\n",
                acurite_getHumidity(buf[6]));
        }
    } else {
    	return 0;
    }

    if (debug_output)
    	debug_callback(bb, bits_per_row);

    return 1;
}

static int acurite_rain_gauge_callback(uint8_t bb[BITBUF_ROWS][BITBUF_COLS], int16_t bits_per_row[BITBUF_ROWS]) {
    // This needs more validation to positively identify correct sensor type, but it basically works if message is really from acurite raingauge and it doesn't have any errors
    if ((bb[0][0] != 0) && (bb[0][1] != 0) && (bb[0][2]!=0) && (bb[0][3] == 0) && (bb[0][4] == 0)) {
	    float total_rain = ((bb[0][1]&0xf)<<8)+ bb[0][2];
		total_rain /= 2; // Sensor reports number of bucket tips.  Each bucket tip is .5mm
        fprintf(stdout, "AcuRite Rain Gauge Total Rain is %2.1fmm\n", total_rain);
		fprintf(stdout, "Raw Message: %02x %02x %02x %02x %02x\n",bb[0][0],bb[0][1],bb[0][2],bb[0][3],bb[0][4]);
        return 1;
    }
    return 0;
}

static int acurite_th_detect(uint8_t *buf){
    if(buf[5] != 0) return 0;
    uint8_t sum = (buf[0] + buf[1] + buf[2] + buf[3]) & 0xff;
    if(sum == 0) return 0;
    return sum == buf[4];
}
static float acurite_th_temperature(uint8_t *s){
    uint16_t shifted = (((s[1] & 0x0f) << 8) | s[2]) << 4; // Logical left shift
    return (((int16_t)shifted) >> 4) / 10.0; // Arithmetic right shift
}
static int acurite_th_callback(uint8_t bb[BITBUF_ROWS][BITBUF_COLS], int16_t bits_per_row[BITBUF_ROWS]) {
    uint8_t *buf = NULL;
    int i;
    for(i = 0; i < BITBUF_ROWS; i++){
	if(acurite_th_detect(bb[i])){
            buf = bb[i];
            break;
        }
    }
    if(buf){
        fprintf(stdout, "Temperature event:\n");
        fprintf(stdout, "protocol      = Acurite Temp&Humidity\n");
        fprintf(stdout, "temp          = %.1f°C\n", acurite_th_temperature(buf));
        fprintf(stdout, "humidity      = %d%%\n\n", buf[3]);
        return 1;
    }

    return 0;
}

/*
 * Acurite 00592TXR / 06002RM Temperature and Humidity Sensor
 * FCC ID: RNERF100VTXR
 * IC: 6608A-RF100VTXR
 * 
 * Documented temperature range of the device is -40C to 70C
 * Humidity is 1% to 99% relative humidity
 * The sensor ID is set by a switch in the battery compartment, it appears to changes the high 2 bits of byte 0.
 * Updates every 16 to 18 seconds.
 * Only tested with one device, values in bytes 0 through 2 might not be correct/consistent.  
 *
 * 7 byte message:
 * [0]: Sensor ID, set by switch.  'ce' for 'A', '8e' for 'B', '0e' for 'C'
 * [1]: 1c?
 * [2]: 44?
 * [3]: humidity, bit 7 is parity.
 * [4]: temperature, high 4 bits, bit 7 is parity.
 * [5]: temperature, low 7 bits, bit 7 is parity.
 * [6]: checksum, bytes 0-5 summed.
 *
 * The temperature is (C+100)*10
 * The humidity is a percentage, no conversion needed.
 * 
 */

static int acurite_th592_parity_check(const uint8_t test_byte) {
  int parity_bit = 0;

  int bit;
  for(bit=0;bit<7;++bit) {
    if(test_byte & (1 << bit)) {
      ++parity_bit;
    }
  }

  parity_bit &= 0x01;

  if(parity_bit == (test_byte >> 7)) return 1;
  return 0;
}

static int acurite_th592_detect(const uint8_t *buf) {
    int i;

    // sanity check, make sure it's not an empty row, and that it doesn't include extra data.

    if(buf[0] == 0x00 || buf[7] != 0x00) return 0;

    // verify the checksum, it should be equal to the sum of the first six bytes.

    uint8_t sum = 0;
    for(i=0;i<6;++i) {
        sum = (sum + buf[i]) & 0xff;
    }

    if(sum != buf[6]) {
        return 0;
    }

    // verify the parity bits for the humidity and temperature bytes.

    if(acurite_th592_parity_check(buf[3]) == 0 ||
       acurite_th592_parity_check(buf[4]) == 0 ||
       acurite_th592_parity_check(buf[5]) == 0) {
        return 0;
    }

    // everything checks out OK

    return 1;
}

static float acurite_th592_temperature(const uint8_t high_byte, const uint8_t low_byte){
    int shifted = ((high_byte & 0x07f) << 7) | (low_byte & 0x07f);
    shifted = shifted - 1000;
    float temperature = shifted / 10.0;
    return temperature;
}

static int acurite_th592_callback(uint8_t bb[BITBUF_ROWS][BITBUF_COLS], int16_t bits_per_row[BITBUF_ROWS]) {
    uint8_t *buf = NULL;
    uint8_t sensor_id;
    int i;

    for(i = 0; i < BITBUF_ROWS; i++){
	if(bits_per_row[i] > 0 && acurite_th592_detect(bb[i])){
            buf = bb[i];
            break;
        }
    }

    if(buf){
	time_t time_now;
	char time_str[LOCAL_TIME_BUFLEN];
	struct tm *tm_info;

	sensor_id = buf[0];

	time(&time_now);
	tm_info = localtime(&time_now);
	strftime(time_str, LOCAL_TIME_BUFLEN, "%Y-%m-%d %H:%M:%S", tm_info);

	float temp_c = acurite_th592_temperature(buf[4],buf[5]);
	float temp_f = temp_c * 1.8 + 32;
	float humidity = (buf[3] & 0x07f);

	printf("%s Acurite 592TXR Sensor %02x: Temperature %3.1f C / %3.1f F\n",
		time_str, sensor_id, temp_c, temp_f);
	printf("%s Acurite 592TXR Sensor %02x: Humidity %3.1f%%\n",
		time_str, sensor_id, humidity);

        return 1;
    }

    return 0;
}

r_device acurite5n1 = {
    /* .name           = */ "Acurite 5n1 Weather Station",
    /* .modulation     = */ OOK_PWM_P,
    /* .short_limit    = */ 70,
    /* .long_limit     = */ 240,
    /* .reset_limit    = */ 21000,
    /* .json_callback  = */ &acurite5n1_callback,
    /* .disabled       = */ 0,
    /* .demod_arg      = */ 0 
};

r_device acurite_rain_gauge = {
    /* .name           = */ "Acurite 896 Rain Gauge",
    /* .modulation     = */ OOK_PWM_D,
    /* .short_limit    = */ 1744/4,
    /* .long_limit     = */ 3500/4,
    /* .reset_limit    = */ 5000/4,
    /* .json_callback  = */ &acurite_rain_gauge_callback,
    /* .disabled       = */ 0,
    /* .demod_arg      = */ 0 
};

r_device acurite_th = {
    /* .name           = */ "Acurite Temperature and Humidity Sensor",
    /* .modulation     = */ OOK_PWM_D,
    /* .short_limit    = */ 300,
    /* .long_limit     = */ 550,
    /* .reset_limit    = */ 2500,
    /* .json_callback  = */ &acurite_th_callback,
    /* .disabled       = */ 0,
    /* .demod_arg      = */ 0 
};

r_device acurite_th_592txr = {
    /* .name           = */ "Acurite Temperature and Humidity Sensor (00592TXR)",
    /* .modulation     = */ OOK_PULSE_PWM_TERNARY,
    /* .short_limit    = */ 70,
    /* .long_limit     = */ 120,
    /* .reset_limit    = */ 800,
    /* .json_callback  = */ &acurite_th592_callback,
    /* .disabled       = */ 0,
    /* .demod_arg      = */ 2 
};

