#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
        perror(s);
        abort();
}

#define TRUE 1
#define FALSE 0

#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

#define NUM_MAXBUF  4
#define DIR_MAXSIZ  60


#define PIN_DC 17
#define PIN_RESET 25
#define PIN_BL 24

#define SYSFS_GPIO_DEBUG 1
#if SYSFS_GPIO_DEBUG
        #define SYSFS_GPIO_Debug(__info,...) printf("Debug: " __info,##__VA_ARGS__)
#else
        #define SYSFS_GPIO_Debug(__info,...)
#endif

int SYSFS_GPIO_Export(int Pin)
{
    char buffer[NUM_MAXBUF];
    int len;
    int fd;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        SYSFS_GPIO_Debug( "Export Failed: Pin%d\n", Pin);
        return -1;
    }

    len = snprintf(buffer, NUM_MAXBUF, "%d", Pin);
    write(fd, buffer, len);

    SYSFS_GPIO_Debug( "Export: Pin%d\r\n", Pin);

    close(fd);
    return 0;
}

int SYSFS_GPIO_Unexport(int Pin)
{
    char buffer[NUM_MAXBUF];
    int len;
    int fd;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        SYSFS_GPIO_Debug( "unexport Failed: Pin%d\n", Pin);
        return -1;
    }

    len = snprintf(buffer, NUM_MAXBUF, "%d", Pin);
    write(fd, buffer, len);

    SYSFS_GPIO_Debug( "Unexport: Pin%d\r\n", Pin);

    close(fd);
    return 0;
}

int SYSFS_GPIO_Direction(int Pin, int Dir)
{
    const char dir_str[]  = "in\0out";
    char path[DIR_MAXSIZ];
    int fd;

    snprintf(path, DIR_MAXSIZ, "/sys/class/gpio/gpio%d/direction", Pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        SYSFS_GPIO_Debug( "Set Direction failed: Pin%d\n", Pin);
        return -1;
    }

    if (write(fd, &dir_str[Dir == IN ? 0 : 3], Dir == IN ? 2 : 3) < 0) {
        SYSFS_GPIO_Debug("failed to set direction!\r\n");
        return -1;
    }

    if(Dir == IN){
        SYSFS_GPIO_Debug("Pin%d:intput\r\n", Pin);
    }else{
        SYSFS_GPIO_Debug("Pin%d:Output\r\n", Pin);
    }

    close(fd);
    return 0;
}

int SYSFS_GPIO_Read(int Pin)
{
    char path[DIR_MAXSIZ];
    char value_str[3];
    int fd;

    snprintf(path, DIR_MAXSIZ, "/sys/class/gpio/gpio%d/value", Pin);
    fd = open(path, O_RDONLY);
    if (fd < 0) {
        SYSFS_GPIO_Debug( "Read failed Pin%d\n", Pin);
        return -1;
    }

    if (read(fd, value_str, 3) < 0) {
        SYSFS_GPIO_Debug( "failed to read value!\n");
        return -1;
    }

    close(fd);
    return(atoi(value_str));
}

int SYSFS_GPIO_Write(int Pin, int value)
{
    const char s_values_str[] = "01";
    char path[DIR_MAXSIZ];
    int fd;

    SYSFS_GPIO_Debug( "GPIO: Write : Pin%d,value = %d\n", Pin, value);

    snprintf(path, DIR_MAXSIZ, "/sys/class/gpio/gpio%d/value", Pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        SYSFS_GPIO_Debug( "Write failed : Pin%d,value = %d\n", Pin, value);
        return -1;
    }

    if (write(fd, &s_values_str[value == LOW ? 0 : 1], 1) < 0) {
        SYSFS_GPIO_Debug( "failed to write value!\n");
        return -1;
    }

    close(fd);
    return 0;
}



static const char *device = "/dev/spidev0.0";
uint8_t mode = 3;
uint8_t bits = 8;
uint32_t speed = 1000000;
uint16_t delay = 0;


#define ST7789_NOP 0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID 0x04
#define ST7789_RDDST 0x09

#define ST7789_SLPIN 0x10
#define ST7789_SLPOUT 0x11
#define ST7789_PTLON 0x12
#define ST7789_NORON 0x13

#define ST7789_INVOFF 0x20
#define ST7789_INVON 0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON 0x29

#define ST7789_CASET 0x2A
#define ST7789_RASET 0x2B
#define ST7789_RAMWR 0x2C
#define ST7789_RAMRD 0x2E

#define ST7789_PTLAR 0x30
#define ST7789_MADCTL 0x36
#define ST7789_COLMOD 0x3A

#define ST7789_FRMCTR1 0xB1
#define ST7789_FRMCTR2 0xB2
#define ST7789_FRMCTR3 0xB3
#define ST7789_INVCTR 0xB4
#define ST7789_DISSET5 0xB6

#define ST7789_GCTRL 0xB7
#define ST7789_GTADJ 0xB8
#define ST7789_VCOMS 0xBB

#define ST7789_LCMCTRL 0xC0
#define ST7789_IDSET 0xC1
#define ST7789_VDVVRHEN 0xC2
#define ST7789_VRHS 0xC3
#define ST7789_VDVS 0xC4
#define ST7789_VMCTR1 0xC5
#define ST7789_FRCTRL2 0xC6
#define ST7789_CABCCTRL 0xC7

#define ST7789_RDID1 0xDA
#define ST7789_RDID2 0xDB
#define ST7789_RDID3 0xDC
#define ST7789_RDID4 0xDD

#define ST7789_GMCTRP1 0xE0
#define ST7789_GMCTRN1 0xE1

#define ST7789_PWCTR6 0xFC

#define CHUNKSIZE 4096
unsigned char txbuf[CHUNKSIZE] = {0x80, };
unsigned char rxbuf[CHUNKSIZE] = {0x80, };

void send_chunk(int fd, int size){
    if(size>CHUNKSIZE) size = CHUNKSIZE;
        struct spi_ioc_transfer tr;
        memset (&tr, 0, sizeof (tr)) ;
        
                tr.tx_buf = (unsigned long)txbuf;
                tr.rx_buf = (unsigned long)rxbuf;
                tr.len = size;
                tr.delay_usecs = 0;
                tr.speed_hz = speed;
                tr.bits_per_word = bits;
                tr.cs_change = 0;
       
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1)
                pabort("can't send spi message");
}

void send_byte(int fd, int b){
    txbuf[0] = b;
    send_chunk(fd, 1);
}

int data_command = 99;
/* msleep(): Sleep for the requested number of milliseconds. */
int msleep(long msec){
    struct timespec ts;
    int res;

    if (msec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}


void set_DC(int data){
    if(data == data_command) return;
    if(data){
        // set DC=high
        SYSFS_GPIO_Write(PIN_DC, HIGH);
    } else {
        // set DC=low
        SYSFS_GPIO_Write(PIN_DC, LOW);
    }
    data_command = data;
    //msleep(10);
}

void send_command_byte(int fd, int c){
    set_DC(FALSE);
    send_byte(fd, c);
}
void send_data_byte(int fd, int c){
    set_DC(TRUE);
    send_byte(fd, c);
}




void reset_st7789(){

    SYSFS_GPIO_Write(PIN_RESET, HIGH);
    msleep(500);
    SYSFS_GPIO_Write(PIN_RESET, LOW);
    msleep(500);
    SYSFS_GPIO_Write(PIN_RESET, HIGH);
    msleep(500);
}


int diff(struct timespec start, struct timespec end, struct timespec *temp)
{
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp->tv_sec = end.tv_sec-start.tv_sec-1;
        temp->tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
        return 1;
    } else {
        temp->tv_sec = end.tv_sec-start.tv_sec;
        temp->tv_nsec = end.tv_nsec-start.tv_nsec;
        return 0;
    }
    
}


void init_st7789(){


    SYSFS_GPIO_Export(PIN_DC);
    SYSFS_GPIO_Export(PIN_RESET);
    SYSFS_GPIO_Export(PIN_BL);

    SYSFS_GPIO_Direction(PIN_DC, OUT);
    SYSFS_GPIO_Direction(PIN_RESET, OUT);
    SYSFS_GPIO_Direction(PIN_BL, OUT);

    SYSFS_GPIO_Write(PIN_BL, LOW);
    msleep(150);
    SYSFS_GPIO_Write(PIN_BL, HIGH);

    reset_st7789();


    int fd = open(device, O_WRONLY);
    if (fd < 0)
                pabort("can't open device");

    /*
         * spi mode
         */
        int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
        if (ret == -1)
                pabort("can't set spi mode w");

    ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
        if (ret == -1)
        pabort("can't get spi mode r" );
        

        /*
         * bits per word
         */
        ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        if (ret == -1)
                pabort("can't set bits per word w ");

        ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
        if (ret == -1)
                pabort("can't get bits per word r");

    /*
         * max speed hz
         */
        ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (ret == -1)
                pabort("can't set max speed hz w");

        ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
        if (ret == -1)
                pabort("can't get max speed hz r");
                

    send_command_byte(fd, ST7789_SWRESET); // Software Reset
    msleep(250);

    send_command_byte(fd, ST7789_MADCTL);
    send_data_byte(fd, 0x70);

    send_command_byte(fd, ST7789_FRMCTR2);
    send_data_byte(fd, 0x0C);
    send_data_byte(fd, 0x0C);
    send_data_byte(fd, 0x00);
    send_data_byte(fd, 0x33);
    send_data_byte(fd, 0x33);

    send_command_byte(fd, ST7789_COLMOD);
    send_data_byte(fd, 0x05);

    send_command_byte(fd, ST7789_GCTRL);
    send_data_byte(fd, 0x14);

    send_command_byte(fd, ST7789_VCOMS);
    send_data_byte(fd, 0x37);

    send_command_byte(fd, ST7789_LCMCTRL); // 0xc0    # Power control
    send_data_byte(fd, 0x2C);

    send_command_byte(fd, ST7789_VDVVRHEN); //  0xc2  # Power control
    send_data_byte(fd, 0x01);

    send_command_byte(fd, ST7789_VRHS); // 0xc3      # Power control
    send_data_byte(fd, 0x12);

    send_command_byte(fd, ST7789_VDVS); // 0xc4      # Power control
    send_data_byte(fd, 0x20);

    send_command_byte(fd, 0xD0);
    send_data_byte(fd, 0xA4);
    send_data_byte(fd, 0xA1);

    send_command_byte(fd, ST7789_FRCTRL2); // 0xc6
    send_data_byte(fd, 0x0F);

    send_command_byte(fd, ST7789_GMCTRP1); // 0xe0   # Set Gamma
    send_data_byte(fd, 0xD0);
    send_data_byte(fd, 0x04);
    send_data_byte(fd, 0x0D);
    send_data_byte(fd, 0x11);
    send_data_byte(fd, 0x13);
    send_data_byte(fd, 0x2B);
    send_data_byte(fd, 0x3F);
    send_data_byte(fd, 0x54);
    send_data_byte(fd, 0x4C);
    send_data_byte(fd, 0x18);
    send_data_byte(fd, 0x0D);
    send_data_byte(fd, 0x0B);
    send_data_byte(fd, 0x1F);
    send_data_byte(fd, 0x23);

    send_command_byte(fd, ST7789_GMCTRN1); // 0xe1   # Set Gamma
    send_data_byte(fd, 0xD0);
    send_data_byte(fd, 0x04);
    send_data_byte(fd, 0x0C);
    send_data_byte(fd, 0x11);
    send_data_byte(fd, 0x13);
    send_data_byte(fd, 0x2C);
    send_data_byte(fd, 0x3F);
    send_data_byte(fd, 0x44);
    send_data_byte(fd, 0x51);
    send_data_byte(fd, 0x2F);
    send_data_byte(fd, 0x1F);
    send_data_byte(fd, 0x1F);
    send_data_byte(fd, 0x20);
    send_data_byte(fd, 0x23);

    send_command_byte(fd, ST7789_INVON);//   # Invert display
    //send_command_byte(fd, ST7789_INVOFF); // 0x20 # Don't invert display

    send_command_byte(fd, ST7789_SLPOUT);

    send_command_byte(fd, ST7789_DISPON);
    msleep(1000);


    int x0 = 0;
    int y0 = 0;
    int x1 = 239;
    int y1 = 239;
    send_command_byte(fd, ST7789_CASET);//       # Column addr set
    send_data_byte(fd, x0 >> 8);
    send_data_byte(fd, x0 & 0xFF);//             # XSTART
    send_data_byte(fd, x1 >> 8);//
    send_data_byte(fd, x1 & 0xFF);//             # XEND
    send_command_byte(fd, ST7789_RASET);//       # Row addr set
    send_data_byte(fd, y0 >> 8);//
    send_data_byte(fd, y0 & 0xFF);//             # YSTART
    send_data_byte(fd, y1 >> 8);//
    send_data_byte(fd, y1 & 0xFF);//             # YEND
    //send_command_byte(fd,ST7789_RAMWR);       # write to RAM


    
    char *fbuf;

#define FILESIZE 115200

    fbuf=malloc(FILESIZE);



    for(int i=0; i<7; i++){
        
            
        char fn[256];
        sprintf(fn, "0%d.bin", i);
        FILE *fp = fopen(fn, "r");
        if (fp != NULL) {
            size_t newLen;
            int ptr=0;
            do {
                newLen = fread(&fbuf[ptr], sizeof(uint8_t), CHUNKSIZE, fp);
                ptr+=newLen;
                if ( ferror( fp ) != 0 ) {
                    fputs("Error reading file", stderr);
                }
                
            } while(newLen == CHUNKSIZE);

            fclose(fp);
        }

        int ptr=0;
        send_command_byte(fd, ST7789_RAMWR);
        set_DC(TRUE);

        struct timespec time0;
        struct timespec time1;
        struct timespec timed;

        clock_gettime(CLOCK_REALTIME, &time0);

        do {
            int nsize=CHUNKSIZE;
            if((ptr+nsize) > FILESIZE)
                nsize=FILESIZE-ptr;
            memcpy(txbuf,&fbuf[ptr],nsize);
            send_chunk(fd, nsize);
            ptr+=CHUNKSIZE;
        } while(ptr<FILESIZE);
        clock_gettime(CLOCK_REALTIME, &time1);
        diff(time0,time1,&timed);

        printf("TIME: %d.%09ld\n", timed.tv_sec, timed.tv_nsec);



    }


        close(fd);
}


int main(int argc, char *argv[])
{
        init_st7789();

        return 0;
}
