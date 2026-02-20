//
// ui.c 
// controls zoom and pan of mandelbrot 
//
// compile with gcc ui.c -o gr -O2 -lm 
//
// Demetrios Gavalas

#include <bits/pthreadtypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 
#include <pthread.h>
#include <stdbool.h>



// PIO port addresses from QSYS
#define FPGA_AXI_BASE      0xC0000000
#define FPGA_AXI_SPAN      0x00001000
#define FPGA_PIO_RESET     0x00000010
#define FPGA_PIO_ZOOM      0x00000020
#define FPGA_PIO_PAN_X     0x00000030
#define FPGA_PIO_PAN_Y     0x00000040
#define FPGA_PIO_START     0x00000050
#define FPGA_PIO_FINISH    0x00000060

// pointers 
void* h2p_virtual_base;
volatile unsigned int*  f_reset_ptr;
volatile unsigned int*  f_zoom_ptr;
volatile signed int*    f_pan_x_ptr;
volatile signed int*    f_pan_y_ptr;
volatile signed int*    f_start_ptr;
volatile signed int*    f_finish_ptr;

// log start and end times for frame loads
struct timeval start_time, end_time;
// file id
int fd;

// threads
void* mouse_handler(void* arg); // handles mouse input for zoom and pan
void* fpga_handler(void* arg); // handles communication with FPGA via PIO ports

// global variables shared between threads
volatile int zoom_level = 0;
volatile double pan_x = -2;
volatile double pan_y = 1.14;
volatile bool running = false;

// Mutex to protect shared state (zoom_level, pan_x, pan_y)
pthread_mutex_t state_mutex;

// convert float to 4.23 fixed point, with 5 padded 0's for the LSB so its 32 bit
#define DOUBLE_TO_FIXED_4_23_P5(d) ( (int32_t) ( (d) * 8388608.0 ) )


// main
int main(void)
{
    // === get FPGA addresses ==================
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
	// get virtual address for AXI bus  
	h2p_virtual_base = mmap( NULL, FPGA_AXI_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_AXI_BASE); 	
	if( h2p_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}

    f_reset_ptr = (unsigned int *)(h2p_virtual_base + FPGA_PIO_RESET);
    f_zoom_ptr = (unsigned int *)(h2p_virtual_base + FPGA_PIO_ZOOM);
    f_pan_x_ptr = (signed int *)(h2p_virtual_base + FPGA_PIO_PAN_X);
    f_pan_y_ptr = (signed int *)(h2p_virtual_base + FPGA_PIO_PAN_Y);
    f_start_ptr = (signed int *)(h2p_virtual_base + FPGA_PIO_START);
    f_finish_ptr = (signed int *)(h2p_virtual_base + FPGA_PIO_FINISH);

    
    // initialize mutex
    pthread_mutex_init(&state_mutex, NULL);

    // create threads for mouse handling and FPGA communication
    pthread_t mouse_thread, fpga_thread;
    pthread_create(&mouse_thread, NULL, mouse_handler, NULL);
    pthread_create(&fpga_thread, NULL, fpga_handler, NULL);

    // wait for threads to finish
    pthread_join(mouse_thread, NULL);
    pthread_join(fpga_thread, NULL); 
    // clean up mutex
    pthread_mutex_destroy(&state_mutex);
    return 0;
}

// pthread function to handle mouse input for zoom and pan
void* mouse_handler(void* arg) {
    int fd, bytes;
    unsigned char data[3];
    const char *pDevice = "/dev/input/mice";

    fd = open(pDevice, O_RDWR); 
    if(fd == -1) {
        printf("ERROR Opening %s\n", pDevice);
        return NULL;
    }

    int left, middle, right;
    int prev_left = 0, prev_right = 0; 
    int x, y;

    while(1) {
        bytes = read(fd, data, sizeof(data));
        if(bytes > 0) {
            left = data[0] & 0x1;
            right = data[0] & 0x2;
            middle = data[0] & 0x4;
            
            // extract sign bits from data[0]
            x = data[1] - ((data[0] << 4) & 0x100);
            y = data[2] - ((data[0] << 3) & 0x100);

            // lock mutex to safely update shared state
            pthread_mutex_lock(&state_mutex);

            // edge detection for zoom
            if (left) {
                zoom_level += zoom_level < 16 ? 1 : 0;
                printf("Zoomed in");
            } 
            if (right) {
                zoom_level -= zoom_level > 1 ? 1 : 0;
                printf("Zoomed out");
            }

            prev_left = left;
            prev_right = right;

            if (middle) {
                pan_x += (double)x * 0.01; // scale mouse movement
                pan_y += (double)y * 0.01;

                // clamp to prevent Q4.23 overflow
                if(pan_x > 7.9) pan_x = 7.9;
                if(pan_x < -8.0) pan_x = -8.0;
                if(pan_y > 7.9) pan_y = 7.9;
                if(pan_y < -8.0) pan_y = -8.0;

                printf("Panned to (%.2f, %.2f)\n", pan_x, pan_y);
            }

            // unlock mutex after updating shared state
            pthread_mutex_unlock(&state_mutex);
        }
    }
    return NULL; 
}

// pthread function to write to PIO ports and measure frame load time
void* fpga_handler(void* arg) {
    int local_zoom;
    double local_pan_x, local_pan_y;
    while(1) {

        // lock mutex to safely read shared state
        pthread_mutex_lock(&state_mutex);
        // read from PIO ports and print to console
        local_zoom = zoom_level;
        local_pan_x = pan_x;
        local_pan_y = pan_y;

        // release mutex 
        pthread_mutex_unlock(&state_mutex);
        *f_reset_ptr = 0xFFFF;
        
        *f_zoom_ptr = local_zoom;
        *f_pan_x_ptr = DOUBLE_TO_FIXED_4_23_P5(local_pan_x); //DOUBLE_TO_FIXED_4_23_P5(local_pan_x);
        *f_pan_y_ptr = DOUBLE_TO_FIXED_4_23_P5(local_pan_y); //DOUBLE_TO_FIXED_4_23_P5(local_pan_y);

        usleep(1);
        *f_reset_ptr = 0x0000;
        usleep(1);


        // time the frame load
        if (!running && *f_start_ptr) {
            gettimeofday(&start_time, NULL);
            running = true;
            printf("Started frame load\n");
        }

        if (running && *f_finish_ptr) {
            gettimeofday(&end_time, NULL);
            long seconds = end_time.tv_sec - start_time.tv_sec;
            long microseconds = end_time.tv_usec - start_time.tv_usec;
            double elapsed = seconds + microseconds*1e-6;
            printf("Frame load time: %.6f seconds\n", elapsed);
            running = false;
        }

        // unlock mutex after reading shared state
        usleep(1000000); 
    }
    return NULL;
}