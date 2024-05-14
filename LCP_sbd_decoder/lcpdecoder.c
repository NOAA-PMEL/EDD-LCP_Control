/* LCP sbd message decoder */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

int main(int argc, char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <filename>\n", argv[0]);
        return 1; 
    }

    /* open the file in binary read */
    FILE *sourcefile = fopen(argv[1], "rb");

    if (sourcefile == NULL)
    {
        perror("File is missing or not able to open");
        return EXIT_FAILURE;
    }

    /* open another file to put data into */
    FILE *destfile = fopen("data.txt", "w");

    if (destfile == NULL)
    {
        perror("Error opening destination file");
        fclose(sourcefile);
        return EXIT_FAILURE;
    }

    uint8_t buf[1024];
    uint16_t readbytes;
    uint16_t i = 0;

    while ((readbytes = fread(buf, 1, sizeof(buf), sourcefile)) > 0)
    {
        for (i = 0; i < readbytes; i++)
        {
            fprintf(destfile, "%02X ", buf[i]);
            //printf("0x%02X \n", (uint8_t)buf[i]);
        }
    }


    uint8_t id        = buf[0];
    uint16_t fw       = buf[1]<< 8 | (buf[2]&0xFF);
    uint16_t fw_date  = buf[3]<< 8 | (buf[4]&0xFF);
    int32_t latitude  = (int32_t) (buf[5]<<24|buf[6]<<16|buf[7]<<8|buf[8]) ;
    int32_t longitude = (int32_t) (buf[9]<<24|buf[10]<<16|buf[11]<<8|buf[12]) ;
    uint8_t lcp_var   = buf[13];
    uint32_t ser      = (uint32_t) (buf[14]<<16|buf[15]<<8|buf[16]) ;
    uint8_t mode      = buf[17];
    uint8_t page      = buf[18];
    uint32_t start    = (uint32_t) (buf[19]<<24|buf[20]<<16|buf[21]<<8|buf[22]) ;
    uint32_t stop     = (uint32_t) (buf[23]<<24|buf[24]<<16|buf[25]<<8|buf[26]) ;
    
    printf("\n\tLCP Profiler Information\n");
    printf("=======================================\n");
    printf("ID              :   %d\n", id);
    printf("Firmware        :   %u.%u.%u-dev\n", fw>>12&0xF, fw>>8&0xF, fw&0xFF);
    printf("Firmware Date   :   %u.%u.%u\n", fw_date>>5&0xF, fw_date&0x1F,  fw_date>>9&0x7F);
    printf("Latitude        :   %.7f\n", (float)(latitude)/(1000000.0) );
    printf("Longitude       :   %.7f\n", (float)(longitude)/(1000000.0) );
    printf("LCP Variant     :   %u\n", lcp_var);
    printf("LCP Serial      :   %c%u%u\n", ser>>16&0xFF, ser>>8&0xFF, ser&0xFF);
    printf("Start Time      :   %u\n", start);
    printf("Stop Time       :   %u\n", stop);

    if (mode == 1)
    {
        printf("Measurements    :   %s, mode %u\n", "Park", mode);
    }
    else if (mode == 2)
    {
        printf("Measurements    :   %s, mode %u\n", "Profile", mode);
    }
    printf("Page Nr.        :   %u\n", page);



    printf("\n");

    float pressure = 0.0;
    float temp = 0.0;

    printf("Sr.No.\tPressure(bar)\tTemperature(°C)\n");
    printf("=======================================\n");

    uint16_t m = 0;

    for (uint16_t j=0+27; j<i; j=j+3)
    {
        m++;
        printf("%u\t", m);
        uint8_t p = buf[j];
        pressure = (float)(p)/(10.0);
        printf("%.3f\t\t", pressure);

        int16_t t = (buf[j+1]) << 8 | buf[j+2];
        float temp = (float)(t)/(100.0);
        printf("%.3f", temp);
        printf("\n");
    }

    printf("\ntotal measurements=%u\n\n", (i-27)/3);

    fclose(sourcefile);
    fclose(destfile);

    return EXIT_SUCCESS;
}
