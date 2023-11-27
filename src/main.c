#include <devctl.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <hw/i2c.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/dispatch.h>

static bool endless = false; // Flag for endless mode
static char *filename = NULL;
static const i2c_addr_t alt_sensor = {.addr = 0x76, .fmt = I2C_ADDRFMT_7BIT};
#define BUFFER_SIZE 100

int main(int argc, char **argv) {

    int c; // Holder for choice
    opterr = 0;

    /* Get command line options. */
    while ((c = getopt(argc, argv, ":e:")) != -1) {
        switch (c) {
        case 'e':
            endless = true;
            filename = optarg;
            break;
        case ':':
            fprintf(stderr, "Option -%c requires an argument.\n", optopt);
            exit(EXIT_FAILURE);
        case '?':
            fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            exit(EXIT_FAILURE);
        default:
            fputs("Something went wrong. Please check 'use fetcher' to see example usage.", stderr);
            exit(EXIT_FAILURE);
        }
    }

    /* Open I2C. */
    int bus = open("/dev/i2c1", O_RDWR);
    if (bus < 0) {
        fprintf(stderr, "Could not open I2C bus.\n");
        exit(EXIT_FAILURE);
    }

    /* Data for I2C sending and receiving */
    typedef struct my_send_rcv_t {
        i2c_sendrecv_t sendrcv;
        uint8_t buf[4];
    } MySendRcv;

    /* Data for I2C sending */
    typedef struct my_send_t {
        i2c_send_t send;
        uint8_t buf[4];
    } MySend;

    // Conversion command
    MySend i2c_send_data = {
        .send = {.stop = 1, .slave = alt_sensor, .len = 1},
        .buf = {0},
    };
    i2c_send_data.buf[0] = 0x40; // Command to convert
    errno_t result = devctl(bus, DCMD_I2C_SEND, &i2c_send_data, sizeof(i2c_send_data), NULL);
    if (result != EOK) {
        fprintf(stderr, "Could not request ADC conversion: %s\n", strerror(result));
        exit(EXIT_FAILURE);
    }
    usleep(900); // Wait for conversion

    // Attempt read
    MySendRcv i2c_sendrcv_data = {.sendrcv = {.stop = 1, .slave = alt_sensor, .send_len = 1, .recv_len = 3},
                                  .buf = {0}};
    i2c_sendrcv_data.buf[0] = 0x00; // Command to read

    result = devctl(bus, DCMD_I2C_SENDRECV, &i2c_sendrcv_data, sizeof(i2c_sendrcv_data), NULL);
    if (result != EOK) {
        fprintf(stderr, "Error while writing command I2C: %s\n", strerror(result));
        exit(EXIT_FAILURE);
    }
    int32_t temperature = 0;
    memcpy(&temperature, i2c_sendrcv_data.buf, 3);
    printf("data: %x\n", temperature);

    // Only read from a file if in endless mode
    if (endless) {

        /* Open file for reading. */
        FILE *f = fopen(filename, "r");
        if (f == NULL) {
            fprintf(stderr, "File '%s' cannot be opened.\n", filename);
            exit(EXIT_FAILURE);
        }

        /* Loop over file to get data. */
        uint8_t buffer[BUFFER_SIZE] = {0};
        for (;;) {
            size_t items_read = fread(&buffer, sizeof(uint8_t), BUFFER_SIZE, f);
            fwrite(&buffer, sizeof(uint8_t), items_read, stdout);

            // In endless mode, go back to the file start when we reach the end of the file
            if (feof(f)) {
                if (endless) {
                    rewind(f);
                } else {
                    return 0; // Otherwise complete
                }
            }
        }
    }

    return 0;
}
