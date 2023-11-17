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
    uint32_t speed = I2C_SPEED_STANDARD;
    printf("speed: %s\n", strerror(devctl(bus, DCMD_I2C_SET_BUS_SPEED, &speed, sizeof(speed), NULL)));

    i2c_driver_info_t info;
    printf("Drv res: %s\n", strerror(devctl(bus, DCMD_I2C_DRIVER_INFO, &info, sizeof(info), NULL)));
    printf("Driver info: %u, %u, %u\n", info.addr_mode, info.verbosity, info.speed_mode);

    /* Send request */
    i2c_send_t send = {
        .slave = alt_sensor,
        .stop = 1,
        .len = 1,
    };
    uint8_t command[sizeof(send) + 1] = {0};
    memcpy(command, &send, sizeof(send));
    command[sizeof(send)] = 0x48; // Conversion request command
    int sent = devctl(bus, DCMD_I2C_SEND, command, sizeof(command), NULL);
    printf("%s\n", strerror(sent));

    /* Read response. */
    uint8_t buf[100] = {0};
    i2c_recv_t receive = {.slave = alt_sensor, .len = sizeof(buf) - sizeof(receive), .stop = 1};
    memcpy(buf, &receive, sizeof(receive));

    int result = devctl(bus, DCMD_I2C_RECV, &buf, sizeof(buf), NULL);
    printf("Return: %s, %d\n", strerror(result), result);

    for (uint8_t i = 0; i < sizeof(receive); i++) {
        printf("%x\n", buf[i]);
    }
    putchar('\n');
    for (uint8_t i = sizeof(receive) - 1; i < 50; i++) {
        printf("%x\n", buf[i]);
    }

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
