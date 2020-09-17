#include <stdio.h>
#include <string.h>
#include "spi_driver.h"

int _fileexists(const char * filename){
    /* try to open file to read */
    FILE *file;
    if ((file = fopen(filename, "r"))){
        fclose(file);
        return 1;
    }
    return 0;
}

int gpio_export(const char* gpio)
{
    char path[80] = "";
    sprintf(path, "/sys/class/gpio/gpio%s/direction", gpio);
    if (_fileexists(path)){
        printf("GPIO %s already exported\n", gpio);
        return 0;
    }

    sprintf(path, "%s", "/sys/class/gpio/export");
    FILE* fp = fopen(path, "a");
    if (fp == NULL) {
        printf("Failed to open %s\n", path);
        return -1;
    }

    int ret = fwrite(gpio, sizeof(char), strlen(gpio), fp);
    if (ret != strlen(gpio)) {
        printf("Failed to write to %s\n", path);
        return -2;
    }

    if (fp != NULL) {
        ret = fclose(fp);
        // if (ret != 0) {
        //     printf("Failed to close %s\n", path);
        //     return -3;
        // }
    }
    delay_MicroSeconds(100000);
    return 0;
}

int gpio_unexport(const char* gpio)
{
    char path[80] = "";
    sprintf(path, "/sys/class/gpio/gpio%s/direction", gpio);
    if (_fileexists(path) == 0){
        printf("GPIO %s already unexported\n", gpio);
        return 0;
    }

    sprintf(path, "%s", "/sys/class/gpio/unexport");
    FILE* fp = fopen(path, "a");
    if (fp == NULL) {
        printf("Failed to open %s\n", path);
        return -1;
    }

    int ret = fwrite(gpio, sizeof(char), strlen(gpio), fp);
    if (ret != strlen(gpio)) {
        printf("Failed to write to %s\n", path);
        return -2;
    }
    
    // delay_MicroSeconds(100000);
    if (fp != NULL) {
        ret = fclose(fp);
        // if (ret != 0) {
        //     printf("Failed to close %s\n", path);
        //     return -3;
        // }
    }
    return 0;
}

int gpio_set_direction(const char* gpio, const char* direction)
{
    char path[80] = "";
    sprintf(path, "/sys/class/gpio/gpio%s/direction", gpio);

    if (_fileexists(path) == 0){
        printf("GPIO %s needs to be exported first\n", gpio);
        // if (gpio_export(gpio) < 0){
        //     printf("GPIO %s failed to export\n", gpio);
            return -1;
        // }
    }

    FILE* fp = fopen(path, "a");
    if (fp == NULL) {
        printf("Failed to open %s\n", path);
        return -1;
    }

    int ret = fwrite(direction, sizeof(char), strlen(direction), fp);
    if (ret != strlen(direction)) {
        printf("Failed to write to %s\n", path);
        fclose(fp);
        return -2;
    }
    
    // delay_MicroSeconds(100000);
    if (fp != NULL) {
        ret = fclose(fp);
        if (ret != 0) {
            printf("Failed to close %s\n", path);
            return -3;
        }
    }
    return 0;
}

int gpio_read(const char* gpio, char* val)
{
    char path[80] = "";
    sprintf(path, "/sys/class/gpio/gpio%s/value", gpio);

    if (_fileexists(path) == 0){
        printf("GPIO %s needs to be exported first\n", gpio);
        return -1;
    }

    FILE* fp = fopen(path, "r");
    if (fp == NULL) {
        printf("Failed to open %s\n", path);
        return -1;
    }

    int ret = fread(val, sizeof(char), 1, fp);
    if (ret != 1) {
        printf("Failed to read from %s %d\n", path, ret);
        fclose(fp);
        return -2;
    }
    val[1] = '\0';
    
    // delay_MicroSeconds(100000);
    if (fp != NULL) {
        ret = fclose(fp);
        // if (ret != 0) {
        //     printf("Failed to close %s\n", path);
        //     return -3;
        // }
    }
    return 0;
}

int gpio_write(const char* gpio, const char* val)
{
    char path[80] = "";
    sprintf(path, "/sys/class/gpio/gpio%s/value", gpio);

    if (_fileexists(path) == 0){
        printf("GPIO %s needs to be exported first\n", gpio);
        return -1;
    }

    FILE* fp = fopen(path, "a");
    if (fp == NULL) {
        printf("Failed to open %s\n", path);
        return -1;
    }

    int ret = fwrite(val, sizeof(char), strlen(val), fp);
    if (ret != strlen(val)) {
        printf("Failed to write to %s\n", path);
        fclose(fp);
        return -2;
    }
    
    // delay_MicroSeconds(100000);
    if (fp != NULL) {
        ret = fclose(fp);
        // if (ret != 0) {
        //     printf("Failed to close %s\n", path);
        //     return -3;
        // }
    }
    return 0;
}
