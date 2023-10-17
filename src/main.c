#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    int c;
    bool endless = false;

    opterr = 0;

    while ((c = getopt(argc, argv, "e")) != -1)
        switch (c) {
        case 'e':
            endless = true;
            break;
        case '?':
            fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            exit(EXIT_FAILURE);
        default:
            fputs("Something went wrong. Please check 'use fetcher' to see example usage.", stderr);
            exit(EXIT_FAILURE);
        }

    if (optind >= argc) {
        fputs("You must provide a file argument.", stderr);
        exit(EXIT_FAILURE);
    }
    char *filename = argv[optind];

    /* Open file for reading. */
    FILE *f = fopen(filename, "r");
    if (f == NULL) {
        fprintf(stderr, "File '%s' cannot be opened.\n", filename);
        exit(EXIT_FAILURE);
    }

    /* Loop over file to get data. */
    uint8_t buffer[100];
    for (;;) {
        fread(&buffer, sizeof(uint8_t), 100, f);
        fwrite(&buffer, sizeof(uint8_t), 100, stdout);

        // In endless mode, go back to the file start when we reach the end of the file
        if (feof(f)) {
            if (endless) {
                fseek(f, 0, SEEK_SET);
            } else {
                return 0; // Otherwise complete
            }
        }
    }

    return 0;
}
