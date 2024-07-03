/* @file lcpdecoder.c
 *
 *  @brief LCP Iridium SBD message decoder
 *
 *  @author Basharat Martin, Basharat.martin@noaa.gov
 *  @date April 30, 2024
 *  @version 0.0.0
 *
 *  @copyright National Oceanic and Atmospheric Administration
 *  @copyright Pacific Marine Environmental Lab
 *  @copyright Environmental Development Division
 *
 *  @note
 *
 *  @bug  No known bugs
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>

void open_directory(char *dirpath);
void parse_sbd_to_txt(char *sbdfile, char *extension);
void decode_sbd_message(char *sbdfile, char *txtfile);
void unpack_measurements(float *p, float *t, uint8_t *buf, uint16_t length);
char *pwd = NULL;

int16_t unpack_data(uint16_t temp)
{
    int16_t t = temp - 500;
    return t;
}

int main(int argc, char *argv[])
{
    struct dirent *entry;
    DIR *dir;

    if (argc != 2) {
        fprintf(stderr, "LCP :: Usage, %s <directory> \n", argv[0]);
        exit(EXIT_FAILURE);
    }

    char *Extension = ".sbd";
    uint8_t Extension_Len = strlen(Extension);

    if ((dir = opendir(argv[1])) == NULL)
    {
        perror("LCP :: ERROR, opening directory");
        exit(EXIT_FAILURE);
    }

    pwd = argv[1];
    //printf("LCP :: directory parth %s \n", pwd);

    while ((entry = readdir(dir)) != NULL)
    {
        char *Filename = entry->d_name;
        uint8_t Filename_Len = strlen(Filename);

        if ( strcmp(Filename + Filename_Len - Extension_Len, Extension) == 0 )
        {
            parse_sbd_to_txt(Filename, Extension);
            //printf("file -> %s , Extension = %s\n", Filename, Extension);
        }
    }

    closedir(dir);
    return EXIT_SUCCESS;
}

void parse_sbd_to_txt(char *sbdfile, char *extension)
{
    char tempfile [32] = {0};
    strcpy (tempfile, sbdfile);

    /* remove .sbd extension */
    char *remove_extension = strstr(tempfile, extension);

    if (remove_extension != NULL)
    {
        *remove_extension = '\0';
        char *txtfile = strcat(tempfile, ".txt");
        //printf("file -> %s , Extension = %s\n", txtfile, extension);
        decode_sbd_message(sbdfile, txtfile);
    }
    else
    {
        perror("LCP :: Error, removing sbd extension");
        exit(EXIT_FAILURE);
    }
}

void decode_sbd_message(char *sbdfile, char *txtfile)
{
    uint8_t buf[512];
    uint16_t readbytes;
    uint16_t total_bytes = 0;

    printf("LCP :: sbdfile = %s\n", sbdfile);
    printf("LCP :: txtfile = %s\n", txtfile);

    /* check if sbd file exists and able to open */
    FILE *sourcefile = fopen(sbdfile, "rb");
    if (sourcefile == NULL)
    {
        printf("LCP :: ERROR, Either %s file is missing or not able to open", sbdfile);
        /* close sbd file */
        fclose(sourcefile);
        exit(EXIT_FAILURE);
    }

    /* read and copy sbd message bytes into the local buffer (buf) */
    while ((readbytes = fread(buf, 1, sizeof(buf), sourcefile)) > 0)
    {
        for (total_bytes = 0; total_bytes < readbytes; total_bytes++)
        {
            //printf("0x%02X , total_bytes=%u\n", (uint8_t)buf[total_bytes], total_bytes);
        }
    }

    //printf("\nlength of i=%u, readbytes=%u\n", i, readbytes);

    /* close sbd file */
    fclose(sourcefile);

    /* parse bytes and save into the txt file */
    uint8_t id        = buf[0];
    uint16_t fw       = buf[1]<< 8 | (buf[2]&0xFF);
    uint16_t fw_date  = buf[3]<< 8 | (buf[4]&0xFF);
    int32_t latitude  = (int32_t) (buf[5]<<24|buf[6]<<16|buf[7]<<8|buf[8]) ;
    int32_t longitude = (int32_t) (buf[9]<<24|buf[10]<<16|buf[11]<<8|buf[12]) ;
    uint8_t lcp_var   = buf[13];
    uint32_t ser      = (uint32_t) (buf[14]<<16|buf[15]<<8|buf[16]) ;
    uint8_t profNr    = buf[17];
    uint8_t mlength   = buf[18];
    uint8_t mode      = buf[19]>>4&0xFF;
    uint8_t pageNr    = buf[19]&0x0F;
    uint32_t start    = (uint32_t) (buf[20]<<24|buf[21]<<16|buf[22]<<8|buf[23]) ;
    uint32_t stop     = (uint32_t) (buf[24]<<24|buf[25]<<16|buf[26]<<8|buf[27]) ;

    /* separate the files profile and park directories */
    struct stat st;
    char *profile_dir = "profile";
    char *park_dir = "park";
    char *path;
    FILE *destfile;

    if (mode == 0x00)
    {
        char park_path[512];
        sprintf (park_path, "%s%s", pwd, park_dir);
        /* park mode */
        if (stat(park_path, &st) == 0 && S_ISDIR(st.st_mode))
        {
            //printf("'%s' exists\n", park_path);
        }
        else
        {
            /* make directory */
            mkdir(park_path, 0777);
        }

        size_t path_len = strlen(park_path) + strlen(txtfile) + 2;
        path = malloc(path_len);
        if (path == NULL)
        {
            free(path);
            printf("LCP :: ERROR, Memory error\n");
            exit(EXIT_FAILURE);
        }
        snprintf (path, path_len, "%s/%s", park_path, txtfile);
        /* check if text file is able to open */
        //destfile = fopen(txtfile, "w");
        destfile = fopen(path, "w");
        if (destfile == NULL)
        {
            perror("LCP :: ERROR, opening txt file");
            fclose(destfile);
            exit(EXIT_FAILURE);
        }
    }
    else if (mode == 0x01)
    {
        /* profile mode */
        char profile_path[512];
        sprintf (profile_path, "%s%s", pwd, profile_dir);
        /* park mode */
        if (stat(profile_path, &st) == 0 && S_ISDIR(st.st_mode))
        {
            //printf("'%s' exists\n", profile_path);
        }
        else
        {
            /* make directory */
            mkdir(profile_path, 0777);
        }

        /* check if text file is able to open */
        size_t path_len = strlen(profile_path) + strlen(txtfile) + 2;
        path = malloc(path_len);
        if (path == NULL)
        {
            printf("LCP :: ERROR, Memory error\n");
            free(path);
            exit(EXIT_FAILURE);
        }
        snprintf (path, path_len, "%s/%s", profile_path, txtfile);
        destfile = fopen(path, "w");
        if (destfile == NULL)
        {
            perror("LCP :: ERROR, opening txt file");
            fclose(destfile);
            exit(EXIT_FAILURE);
        }
    }

    fprintf(destfile, "\n\tLCP Profiler Information\n");
    fprintf(destfile, "=======================================\n");
    fprintf(destfile, "ID              :   %d\n", id);
    fprintf(destfile, "Firmware        :   %u.%u.%u-dev\n", fw>>12&0xF, fw>>8&0xF, fw&0xFF);
    fprintf(destfile, "Firmware Date   :   %u.%u.%u\n", fw_date>>5&0xF, fw_date&0x1F,  fw_date>>9&0x7F);
    fprintf(destfile, "Latitude        :   %.7f\n", (float)(latitude)/(1000000.0) );
    fprintf(destfile, "Longitude       :   %.7f\n", (float)(longitude)/(1000000.0) );
    fprintf(destfile, "LCP Variant     :   %u\n", lcp_var);
    fprintf(destfile, "LCP Serial      :   %c%u%u\n", ser>>16&0xFF, ser>>8&0xFF, ser&0xFF);
    fprintf(destfile, "Profile Nr      :   %u\n", profNr);
    fprintf(destfile, "Measurements    :   %u\n", mlength);

    if (mode == 0x00)
    {
        //fprintf(destfile, "Measurements    :   %s, mode %u\n", "Park", mode);
        fprintf(destfile, "Mode            :   %u, %s\n", mode, "Park Mode");
    }
    else if (mode == 0x01)
    {
        fprintf(destfile, "Mode            :   %u, %s\n", mode, "Profile Mode");
        //fprintf(destfile, "Measurements    :   %s, mode %u\n", "Profile", mode);
    }
    fprintf(destfile, "Page Nr         :   %u\n", pageNr);
    fprintf(destfile, "Start Time      :   %u\n", start);
    fprintf(destfile, "Stop Time       :   %u\n", stop);
    fprintf(destfile, "\n");

    float pressure = 0.0;
    float temp = 0.0;

    fprintf(destfile, "Sr.No.\tPressure(bar)\tTemperature(°C)\n");
    fprintf(destfile, "=======================================\n");

    //if (mlength > 124)
    //{
    //    mlength = 124;
    //}

    /* copy buf from 28th position to local buffer up to mlength from */
    //uint16_t bytes = (mlength*(12+8) +7) / 8;
    uint16_t bytes = total_bytes - 28;
    uint8_t lbuf[bytes];
    uint16_t len = (bytes*8)/(12+8);
    float t[len];
    float p[len];

    for (uint16_t i=0; i<bytes; i++)
    {
        lbuf[i] = buf[28+i];
        //printf("lbuf[%u]=0x%02X, buf[%u]=0x%02X \n", i, lbuf[i], 28+i, buf[28+i]);
    }

    unpack_measurements(p, t, lbuf, len);

    for (uint16_t j=0; j<len; j++)
    {
        fprintf(destfile, "%u\t", j+1);
        fprintf(destfile, "%.3f\t\t", p[j]);
        fprintf(destfile, "%.3f", t[j]);
        fprintf(destfile, "\n");
    }
    fprintf(destfile, "\ntotal measurements=%u\n\n", len);

    //uint16_t m = 0;
    //for (uint16_t j=0+28; j<i; j=j+3)
    //{
    //    m++;
    //    fprintf(destfile, "%u\t", m);
    //    uint8_t p = buf[j];
    //    pressure = (float)(p)/(10.0);
    //    fprintf(destfile, "%.3f\t\t", pressure);

    //    int16_t t = (buf[j+1]) << 8 | buf[j+2];
    //    float temp = (float)(t)/(100.0);
    //    fprintf(destfile, "%.3f", temp);
    //    fprintf(destfile, "\n");
    //}
    //fprintf(destfile, "\ntotal measurements=%u\n\n", (i-28)/3);

    fclose(destfile);
    free(path);
}

void unpack_measurements(float *p, float *t, uint8_t *buf, uint16_t length)
{
    /*  12 + 8 bits for temperature and pressure = 20bits
        fit bitwise so no space can be left */
    uint16_t bytes = 0;
    uint32_t bitpos = 0;
    uint16_t T = 0;
    uint8_t P = 0;

    for (uint16_t i=0; i<length; i++)
    {
        T = 0, P = 0;

        for (uint8_t j=0; j<12; j++)
        {
            //printf("buf[%u] , T = 0x%04X\n", bytes, T);
            if (buf[bytes]&(1<<(7-(bitpos%8))))
            {
                T |= 1<<(11-j);
                //printf("buf[%u] , T = 0x%04X\n", buf[bytes], T);
            }

            //printf("buf[%u] = 0x%02X, T=0x%04X\n", bytes, buf[bytes], (T&(1<<(11-j))) >> bitwise);
            bitpos++;

            if ((bitpos%8) == 0)
            {
                //printf("bitpos (%u) , bytes (%u), T = 0x%04X\n", bitpos, bytes, T);
                bytes++;
            }
        }
        t[i] = (float)((float)unpack_data(T)/100.0);

        for (uint8_t j=0; j<8; j++)
        {
            if (buf[bytes] & (1<<(7-(bitpos %8))))
            {
                P |= 1<<(7-j);
            }

            //printf("buf[%u] = 0x%02X, P=0x%02X\n", bytes, buf[bytes], P&(1<<(7-j)));
            bitpos++;

            if ( (bitpos%8) == 0)
            {
                bytes++;
            }
        }

        p[i] = ((float)P/10.0);
        //printf("i= %u, T = %.2f, 0x%04X, P= %.2f, 0x%02X\n", i, t[i], T, p[i], P);
    }
    //for (uint16_t i=0; i<length; i++)
    //{
    //    printf("temp = %.2f, pressure = %.2f \n", t[i], p[i]);
    //}
    printf("\n");
}
