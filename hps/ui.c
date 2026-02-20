//
// ui.c
// controls zoom and pan of mandelbrot
//
// compile with gcc ui.c -o gr -O2 -lm
//
// Demetrios Gavalas

#include <bits/pthreadtypes.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

// PIO port addresses from QSYS
#define FPGA_AXI_BASE 0xC0000000
#define FPGA_AXI_SPAN 0x00001000
#define FPGA_PIO_RESET 0x00000010
#define FPGA_PIO_ZOOM 0x00000020
#define FPGA_PIO_PAN_X 0x00000030
#define FPGA_PIO_PAN_Y 0x00000040
#define FPGA_PIO_START 0x00000050
#define FPGA_PIO_FINISH 0x00000060

// View constants — must match Verilog defines
#define X_PIXELS 640
#define Y_PIXELS 480
// BASE_STEP = 39000 / 2^23 ≈ 0.004650116 per pixel at zoom=0
#define BASE_STEP_FLOAT (39000.0 / 8388608.0)

// pointers
void *h2p_virtual_base;
volatile unsigned int *f_reset_ptr;
volatile unsigned int *f_zoom_ptr;
volatile signed int *f_pan_x_ptr;
volatile signed int *f_pan_y_ptr;
volatile signed int *f_start_ptr;
volatile signed int *f_finish_ptr;

// log start and end times for frame loads
struct timeval start_time, end_time;
// file id
int fd;

// threads
void *mouse_handler(void *arg); // handles mouse input for zoom and pan
void *fpga_handler(void *arg);  // handles communication with FPGA via PIO ports

// State is tracked as the CENTER of the view in complex-plane coordinates.
// The corner (x_start, y_start) is computed from center + half-extent.
volatile int zoom_level = 0;
volatile double center_x = -0.5; // center of the standard Mandelbrot view
volatile double center_y = 0.0;
volatile bool state_dirty = true; // set true when any parameter changes
volatile bool running = false;

// Mutex to protect shared state
pthread_mutex_t state_mutex;

// convert double to 4.23 fixed point (sign-extended to 32 bits)
#define DOUBLE_TO_FIXED_4_23(d) ((int32_t)((d) * 8388608.0))

// Compute the per-pixel step at a given zoom level
static inline double step_at_zoom(int zoom) {
  return BASE_STEP_FLOAT / (double)(1 << zoom);
}

// main
int main(void) {
  // === get FPGA addresses ==================
  // Open /dev/mem
  if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
    printf("ERROR: could not open \"/dev/mem\"...\n");
    return (1);
  }

  // get virtual address for AXI bus
  h2p_virtual_base = mmap(NULL, FPGA_AXI_SPAN, (PROT_READ | PROT_WRITE),
                          MAP_SHARED, fd, FPGA_AXI_BASE);
  if (h2p_virtual_base == MAP_FAILED) {
    printf("ERROR: mmap3() failed...\n");
    close(fd);
    return (1);
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
void *mouse_handler(void *arg) {
  int fd, bytes;
  unsigned char data[3];
  const char *pDevice = "/dev/input/mice";

  fd = open(pDevice, O_RDWR);
  if (fd == -1) {
    printf("ERROR Opening %s\n", pDevice);
    return NULL;
  }

  int left, middle, right;
  int prev_left = 0, prev_right = 0;
  int x, y;

  while (1) {
    bytes = read(fd, data, sizeof(data));
    if (bytes > 0) {
      left = data[0] & 0x1;
      right = data[0] & 0x2;
      middle = data[0] & 0x4;

      // extract sign bits from data[0]
      x = data[1] - ((data[0] << 4) & 0x100);
      y = data[2] - ((data[0] << 3) & 0x100);

      // lock mutex to safely update shared state
      pthread_mutex_lock(&state_mutex);

      // Edge-detected zoom: only act on the press transition
      // Center is preserved automatically since we track center_x/center_y
      if (left && !prev_left) {
        if (zoom_level < 16) {
          zoom_level++;
          state_dirty = true;
          printf("Zoom in  -> level %d\n", zoom_level);
        }
      }
      if (right && !prev_right) {
        if (zoom_level > 0) {
          zoom_level--;
          state_dirty = true;
          printf("Zoom out -> level %d\n", zoom_level);
        }
      }

      prev_left = left;
      prev_right = right;

      // Pan with middle button — sensitivity scales with zoom
      if (middle) {
        double step = step_at_zoom(zoom_level);
        // Scale mouse counts to complex-plane distance.
        // At zoom 0, one mouse count ≈ 2 pixels of movement in the complex
        // plane.
        double pan_scale = step * 2.0;

        center_x += (double)x * pan_scale;
        center_y -=
            (double)y * pan_scale; // mouse y is inverted vs complex plane

        // clamp to prevent Q4.23 overflow (center must stay in representable
        // range)
        if (center_x > 7.0)
          center_x = 7.0;
        if (center_x < -7.0)
          center_x = -7.0;
        if (center_y > 7.0)
          center_y = 7.0;
        if (center_y < -7.0)
          center_y = -7.0;

        state_dirty = true;
        printf("Pan center (%.4f, %.4f)  zoom=%d\n", center_x, center_y,
               zoom_level);
      }

      // unlock mutex after updating shared state
      pthread_mutex_unlock(&state_mutex);
    }
  }
  return NULL;
}

// pthread function to write to PIO ports and measure frame load time
void *fpga_handler(void *arg) {
  int local_zoom;
  double local_cx, local_cy;
  bool need_reset;

  while (1) {
    // Snapshot shared state under the lock
    pthread_mutex_lock(&state_mutex);
    local_zoom = zoom_level;
    local_cx = center_x;
    local_cy = center_y;
    need_reset = state_dirty;
    state_dirty = false; // consume the flag
    pthread_mutex_unlock(&state_mutex);

    if (need_reset) {
      // Compute the top-left corner from center and view half-extents
      double step = step_at_zoom(local_zoom);
      double half_w = (X_PIXELS / 2.0) * step; // half width in complex coords
      double half_h = (Y_PIXELS / 2.0) * step; // half height in complex coords
      double x_start = local_cx - half_w;
      double y_start =
          local_cy + half_h; // top edge (y decreases downward in Verilog)

      printf("Resetting FPGA: center=(%.4f,%.4f) corner=(%.4f,%.4f) zoom=%d "
             "step=%.6f\n",
             local_cx, local_cy, x_start, y_start, local_zoom, step);

      // Assert reset, write parameters, then deassert
      *f_reset_ptr = 0xFFFF;
      *f_zoom_ptr = local_zoom;
      *f_pan_x_ptr = DOUBLE_TO_FIXED_4_23(x_start);
      *f_pan_y_ptr = DOUBLE_TO_FIXED_4_23(y_start);
      usleep(1);
      *f_reset_ptr = 0x0000;

      running = false; // new frame starting
    }

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
      double elapsed = seconds + microseconds * 1e-6;
      printf("Frame load time: %.6f seconds\n", elapsed);
      running = false;
    }

    usleep(10000); // 10ms poll — no need to spin harder than this
  }
  return NULL;
}