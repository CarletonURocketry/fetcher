#include <fcntl.h>
#include <hw/i2c.h>
#include <stdlib.h>

/**
 * Initialize the master I2C interface.
 * @param argc Should always be set to 1.
 * @param argv A pointer to the I2C device handle string.
 * @return The handle that is passed to all other functions. NULL if failed.
 */
static void *init(int argc, char *argv[]) {
    if (argc != 1) return NULL;

    int fd = open(argv[0], O_RDWR);
    if (fd == -1) return NULL;

    int *handle = malloc(sizeof(int));
    *handle = fd;
    return handle;
}

/** Frees the memory associated with hdl. */
static void fini(void *hdl) { free(hdl); }

/**
 * Request info about the driver.
 * @return 0 if successful, -1 if not.
 */
static int driver_info(void *hdl, i2c_driver_info_t *info) {
    return devctl(*(int *)hdl, DCMD_I2C_DRIVER_INFO, &info, sizeof(i2c_driver_info_t), NULL);
}

/**
 * Gets the function table required for interacting with our I2C devices.
 * @param funcs The function table to fill in. The library initializes this table before calling this function. If you
 * haven't implemented a function in the table, leave its entry unchanged; don't set it to NULL.
 * @param tabsize The size of the structure that funcs points to, in bytes.
 * @return 0 for success, -1 for failure.
 */
int i2c_master_getfuncs(i2c_master_funcs_t *funcs, int tabsize) {
    funcs->size = tabsize;
    funcs->init = &init;
    funcs->fini = &fini;
    funcs->driver_info = &driver_info;
    return 0;
}
