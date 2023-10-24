#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define BUFFER_SIZE 100

int main(int argc, char **argv) {

    int c;                // Holder for choice
    bool endless = false; // Flag for endless mode
    opterr = 0;

    /* Get command line options. */
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

    /* Check to see if we also received the file argument after all the flags. */
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

    return 0;
}
