///////////////////////////////////////
/// 640x480 version!
/// test VGA with hardware video input copy to VGA
///////////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 
#include "address_map_arm_brl4.h"
#include <stdint.h>


/* function prototypes */
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_disc (int, int, int, short);
int  VGA_read_pixel(int, int) ;
int  video_in_read_pixel(int, int);
void draw_delay(void) ;

#define ROW_OFFSET    0x00000050
#define COL_OFFSET    0x00000060
#define COLOR_OFFSET  0x00000070
#define STATE_OFFSET  0x00000080
#define SINGLE_OFFSET 0x00000000
#define INTEGRAL_OFFSET 0x00000090
#define FLAG_OFFSET     0x00000100
#define INDEX_OFFSET     0x00000110

#define DATA_RECIEVED_OFFSET   0x00000170
#define UPPER_LX               0x00000120
#define UPPER_LY               0x00000130
#define LOWER_RX               0x00000140
#define LOWER_RY               0x00000150
#define FEATURE_READY          0x00000160
#define GRAYSCALE              0x00000180
#define REQ_VAL                0x00000190
#define REQ_RDY                0x00000200
#define RESP_RDY               0x00000210
#define RESP_VAL               0x00000220
#define CLK_OFFSET             0x00000230
#define RESET_OFFSET           0x00000240

#define SRAM_GRAYSCALE_OFFSET  0x00020000
#define SRAM_GRAYSCALE_END     0x00032c13

volatile unsigned int * pio_row = NULL ; 
volatile unsigned int * pio_col = NULL ; 
volatile unsigned int * pio_color = NULL ; 
volatile unsigned int * pio_state = NULL ; 
volatile unsigned int * pio_single = NULL ; 
volatile unsigned int * pio_integral_data = NULL ; 


volatile unsigned int * pio_flag = NULL ; 
volatile unsigned int * pio_data_r = NULL ;

volatile unsigned int * feature_ready = NULL ;
volatile unsigned int * grayscale = NULL ;

volatile unsigned int * pio_ul_x = NULL ;
volatile unsigned int * pio_ul_y = NULL ;
volatile unsigned int * pio_lr_x = NULL ;
volatile unsigned int * pio_lr_y = NULL ;
volatile unsigned int * pio_index = NULL ;

volatile unsigned int * pio_req_val = NULL ;
volatile unsigned int * pio_req_rdy = NULL ;
volatile unsigned int * pio_resp_rdy = NULL ;
volatile unsigned int * pio_resp_val = NULL ;

volatile unsigned int * pio_clk = NULL ;
volatile unsigned int * pio_reset = NULL ;


volatile unsigned int * gray_sdram = NULL;



// the light weight buss base
void *h2p_lw_virtual_base;
volatile unsigned int *h2p_lw_video_in_control_addr=NULL;
volatile unsigned int *h2p_lw_video_in_resolution_addr=NULL;
//volatile unsigned int *h2p_lw_video_in_control_addr=NULL;
//volatile unsigned int *h2p_lw_video_in_control_addr=NULL;

volatile unsigned int *h2p_lw_video_edge_control_addr=NULL;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;

volatile unsigned int * gray_image_values = NULL ;
void *gray_virtual_base;

void *vga_pixel_virtual_base;

// video input buffer
volatile unsigned int * video_in_ptr = NULL ;
void *video_in_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

// shared memory 
key_t mem_key=0xf0;
int shared_mem_id; 
int *shared_ptr;
int shared_time;
int shared_note;
char shared_str[64];

// pixel macro
#define VGA_PIXEL(x,y,color) do{\
	char  *pixel_ptr ;\
	pixel_ptr = (char *)vga_pixel_ptr + ((y)<<10) + (x) ;\
	*(char *)pixel_ptr = (color);\
} while(0)
	
#define VIDEO_IN_PIXEL(x,y,color) do{\
	char  *pixel_ptr ;\
	pixel_ptr = (char *)video_in_ptr + ((y)<<9) + (x) ;\
	*(char *)pixel_ptr = (color);\
} while(0)
	

// measure time
struct timeval t1, t2;
double elapsedTime;
struct timespec delay_time ;
	
int main(void)
{
  //delete file because not needed
  const char* file_path_del = "/home/root/video/temp.txt";
  remove(file_path_del);
  
  uint8_t gray[240][320];
  
  uint32_t intergral_img[240][320];
  
  
  int32_t corners[4];
  int32_t eyes_array[8];
	delay_time.tv_nsec = 10 ;
	delay_time.tv_sec = 0 ;

	// Declare volatile pointers to I/O registers (volatile 	// means that IO load and store instructions will be used 	// to access these pointer locations, 
	// instead of regular memory loads and stores) 
  	
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
  
	// === get FPGA addresses ==================
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
    // get virtual addr that maps to physical
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN + 0x10000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap1() failed...\n" );
		close( fd );
   
		return(1);
	}
    h2p_lw_video_in_control_addr=(volatile unsigned int *)(h2p_lw_virtual_base+VIDEO_IN_BASE+0x0c);
	h2p_lw_video_in_resolution_addr=(volatile unsigned int *)(h2p_lw_virtual_base+VIDEO_IN_BASE+0x08);
	*(h2p_lw_video_in_control_addr) = 0x04 ; // turn on video capture
	*(h2p_lw_video_in_resolution_addr) = 0x00f00140 ;  // high 240 low 320
	h2p_lw_video_edge_control_addr=(volatile unsigned int *)(h2p_lw_virtual_base+VIDEO_IN_BASE+0x10);
	*h2p_lw_video_edge_control_addr = 0x01 ; // 1 means edges
	*h2p_lw_video_edge_control_addr = 0x00 ; // 1 means edges
	
	// === get VGA char addr =====================
	// get virtual addr that maps to physical
	vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );	
	if( vga_char_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap2() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the character 
	vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

	// === get VGA pixel addr ====================
	// get virtual addr that maps to physical
	// SDRAM
	vga_pixel_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_BASE); //SDRAM_BASE	
	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    // Get the address that maps to the FPGA pixel buffer
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);
 
  
 
 
 
 
	// === get video input =======================
	// on-chip RAM
	video_in_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_ONCHIP_BASE); 
	if( video_in_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
	// format the pointer
	video_in_ptr =(unsigned int *)(video_in_virtual_base);
 
   printf("change\n");
   
 
 //for reference!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 //#define SRAM_GRAYSCALE_OFFSET   0xa000000
 //#define SRAM_GRAYSCALE_END      0xa012c13
 
 ////-------------------------------------------------------------------------------------------------------------
gray_virtual_base = mmap( NULL, (FPGA_ONCHIP_SPAN + 0x0900), ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_BASE + SRAM_GRAYSCALE_OFFSET); //SRAM_GRAYSCALE_OFFSET
 	if( gray_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
 

   
  
  //gray_image_values = (unsigned int *)(gray_virtual_base);
  
  //I also tried these
  //gray_image_values = (unsigned int *)(video_in_virtual_base + SRAM_GRAYSCALE_OFFSET);
  //gray_image_values = (unsigned int *)(vga_pixel_virtual_base + SRAM_GRAYSCALE_OFFSET);
  
  
 
	// ===========================================

	/* create a message to be displayed on the VGA 
          and LCD displays */
	char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
 
	char text_bottom_row[40] = "Cornell ece5760\0";
	char num_string[20], time_string[50] ;
	
	// a pixel from the video
	int pixel_color;
	// video input 
 
 
	int i,j;
 
	
	// clear the screen
	VGA_box (0, 0, 639, 479, 0x03);
	// clear the text
	VGA_text_clear();
	VGA_text (1, 56, text_top_row);
	VGA_text (1, 57, text_bottom_row);
	
	// start timer
    //gettimeofday(&t1, NULL);
    
  pio_row = (unsigned int *)(h2p_lw_virtual_base + ROW_OFFSET);
  pio_col = (unsigned int *)(h2p_lw_virtual_base + COL_OFFSET);
  pio_color = (unsigned int *)(h2p_lw_virtual_base + COLOR_OFFSET);
  pio_state = (unsigned int *)(h2p_lw_virtual_base + STATE_OFFSET);
  pio_single = (unsigned int *)(h2p_lw_virtual_base + SINGLE_OFFSET);
  pio_integral_data = (unsigned int *)(h2p_lw_virtual_base + INTEGRAL_OFFSET);
  pio_flag = (unsigned int *)(h2p_lw_virtual_base + FLAG_OFFSET);
  pio_data_r = (unsigned int *)(h2p_lw_virtual_base + DATA_RECIEVED_OFFSET);
  pio_ul_x = (unsigned int *)(h2p_lw_virtual_base + UPPER_LX);
  pio_ul_y = (unsigned int *)(h2p_lw_virtual_base + UPPER_LY);
  pio_lr_x = (unsigned int *)(h2p_lw_virtual_base + LOWER_RX);
  pio_lr_y = (unsigned int *)(h2p_lw_virtual_base + LOWER_RY);
  pio_index = (unsigned int *)(h2p_lw_virtual_base + INDEX_OFFSET);
  
  grayscale = (unsigned int *)(h2p_lw_virtual_base + GRAYSCALE);
  feature_ready = (unsigned int *)(h2p_lw_virtual_base + FEATURE_READY);
  
  pio_req_val = (unsigned int *)(h2p_lw_virtual_base + REQ_VAL); ;
  pio_req_rdy = (unsigned int *)(h2p_lw_virtual_base + REQ_RDY); ;
  pio_resp_rdy = (unsigned int *)(h2p_lw_virtual_base + RESP_RDY); 
  pio_resp_val = (unsigned int *)(h2p_lw_virtual_base + RESP_VAL);
  
  pio_clk = (unsigned int *)(h2p_lw_virtual_base + CLK_OFFSET);
  pio_reset = (unsigned int *)(h2p_lw_virtual_base + RESET_OFFSET);
  
  gray_image_values = (unsigned int *)(h2p_lw_virtual_base + SRAM_GRAYSCALE_OFFSET);
  
  //gray_image_values = (unsigned char *)(h2p_lw_virtual_base + SRAM_GRAYSCALE_OFFSET);
  
  char loc_string[50], color_string[50], state_string[50] ;
 
 
 
 
int row, col;
 
 //-------------------------------------------------------------
 
 //test


//printf("color: %d \n", VGA_read_pixel(20, 20));
while(*(pio_resp_val) != 1){
}
//VGA_box(100, 50, 420, 290, 0x780);
 int offset = 0;
 int blue, green, red, color;
 uint8_t grey;
  for(row = 0; row < 240; row++){
    for(col = 0; col < 320; col++){
      
      color = VGA_read_pixel(col + 100, row + 50);
      blue = (color&0x1F)*((255.0/31.0));
      green = ((color>>5)&0x3F)*(255.0/63.0);
      red = ((color>>11)&0x1F)*((255.0/31.0));
      
      grey = (blue + green + red)/3;
      //grey = sdram_grayscale(row, col);
      
      gray[row][col] = grey;
      //printf("gray value %d\n", (*(gray_image_values + offset)));
      offset++;
    }
  }
 
 //-------------------------------------------------------------
 
 printf("got data\n");
 
  for(row = 0; row < 240; row++){
   for(col = 0; col < 320; col++){
   
   printf("grayscale: %d \n", gray[row][col]);
     }
  }
 
 
 
 
 
 //send to windows side
   FILE * snap_data = fopen("data.dat", "wb");
    
    if(snap_data == NULL){
      printf("Error");
      return 1;
    }
    
    
    fwrite(gray, sizeof(uint8_t), 320*240, snap_data);
    
    fclose(snap_data);
    
    
    
    
    
    
    
    
    //waiting to be finished this is very iffy but seems to do the trick
    const char* file_path_wait = "/home/root/video/temp.txt";
    int mode = F_OK | R_OK;
    while(access(file_path_wait, mode) != 0){}
    printf("done\n");
    
    
    
    
//-------------------------------------------------------------------------------------------------
    
    //read corners
    FILE *corn = fopen("corners.dat", "rb");
    
    printf("here\n");
    if (corn == NULL) {
        printf("Error: could not open file\n");
        return 1;
    }

    fread(corners, sizeof(int32_t), 4, corn);

    fclose(corn);
    
    
    //offset for x is 100 and y is 50 for vga
    *pio_ul_x = corners[0]; //this is without vga offset 
    *pio_ul_y = corners[1]; //this is without vga offset 
    *pio_lr_x = corners[2] + corners[0]; //this is without vga offset 
    *pio_lr_y = corners[3] + corners[1]; //this is without vga offset 
    
    
    printf("x1: %d, y1: %d, x2: %d, y2: %d\n", *pio_ul_x, *pio_ul_y, *pio_lr_x, *pio_lr_y);
    
    
    //VGA_box(100 + (*pio_ul_x), 50 + (*pio_ul_y), 100 + (*pio_lr_x), 50 +(*pio_lr_y), 0x780);

//-------------------------------------------------------------------------------------------------

    int eye1_ulx, eye1_uly, eye1_lrx, eye1_lry;
    int eye2_ulx, eye2_uly, eye2_lrx, eye2_lry;
    //read eyes
    FILE *eyes = fopen("eyes.dat", "rb");
    
    printf("here\n");
    if (eyes == NULL) {
        printf("Error: could not open file\n");
        return 1;
    }

    fread(eyes_array, sizeof(int32_t), 8, eyes);

    fclose(eyes);
    
    eye1_ulx = eyes_array[0] + (*pio_ul_x);
    eye1_uly = eyes_array[1] + (*pio_ul_y);
    eye1_lrx = eyes_array[2] + eyes_array[0] + (*pio_ul_x);
    eye1_lry = eyes_array[3] + eyes_array[1] + (*pio_ul_y);
    
    int half_eye1x = eyes_array[2]/2;
    int half_eye1y = eyes_array[3]/2;
    
    //location on face of eyes
    eye2_ulx = eyes_array[4] + (*pio_ul_x);
    eye2_uly = eyes_array[5] + (*pio_ul_y);
    eye2_lrx = eyes_array[6] + eyes_array[4] + (*pio_ul_x);
    eye2_lry = eyes_array[7] + eyes_array[5] + (*pio_ul_y);
    
    int half_eye2x = eyes_array[6]/2;
    int half_eye2y = eyes_array[7]/2;

    
    printf("x1: %d, y1: %d, x2: %d, y2: %d\n", eye1_ulx, eye1_uly, eye1_lrx, eye1_lry);
    printf("x1: %d, y1: %d, x2: %d, y2: %d\n", eye2_ulx, eye2_uly, eye2_lrx, eye2_lry);
    
    
    //VGA_box(100 + eye1_ulx, 50 + eye1_uly, 100 + eye1_lrx, 50 + eye1_lry, 0x780);
    //VGA_box(100 + eye1_ulx , 50 + eye1_uly, 100 + eye1_lrx, 50 + eye1_lry , 0x780);
    
    //VGA_box(100 + eye2_ulx , 50 + eye2_uly, 100 + eye2_lrx, 50 + eye2_lry , 0x780);
    
    //VGA_disc((100 + center_eye1x + eye1_ulx), (50 + center_eye1y + eye1_uly), center_eye1y, 0x780);
    
    //VGA_disc((100 + center_eye2x + (*pio_ul_x)), (50 + center_eye2y + (*pio_ul_y)), center_eye2y, 0x780);
    
    //these do not have offset
    int centerx_eye1 = eye1_ulx + half_eye1x;
    int centery_eye1 = eye1_uly + half_eye1y;
    
    int centerx_eye2 = eye2_ulx + half_eye2x;
    int centery_eye2 = eye2_uly + half_eye2y;
    
    
    
    VGA_disc(100 + centerx_eye1, 50 + centery_eye1, half_eye1y, 0x780);
    VGA_disc(100 + centerx_eye2, 50 + centery_eye2, half_eye2y, 0x780);










//-------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------
    
    
    
    *feature_ready = 1;
    
    

	
	while(1) 
	{
		gettimeofday(&t1, NULL);
		 
		// note that this version of VGA_disk
		// has THROTTLED pixel write
		// VGA_disc((rand()&0x3ff), (rand()&0x1ff), rand()&0x3f, rand()&0xff) ;
		
		// software copy test.
		// in production, hardware does the copy
		// put a few  pixel in input buffer
		 //VIDEO_IN_PIXEL(160,120,0xff);
		 //VIDEO_IN_PIXEL(0,0,0xff);
		 //VIDEO_IN_PIXEL(319,239,0xff);
		 //VIDEO_IN_PIXEL(300,200,0xff);
		
		// read/write video input -- copy to VGA display
		// for (i=0; i<320; i++) {
			// for (j=0; j<240; j++) {
				// pixel_color = video_in_read_pixel(i,j);
				// VGA_PIXEL(i+100,j+50,pixel_color);
			// }
		// }
   


    // printf("v: ", *(pio_row), ", ", *(pio_col), ": ", *(pio_color)) ;
    //printf("state: %d, current: %d, %d, color: %d, single: %d, integral: %d \n", *(pio_state), *(pio_row), *(pio_col), *(pio_color), *(pio_single), *(pio_integral_data));
    

		// stop timer
//		 gettimeofday(&t2, NULL);
//		 elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
//		 elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
        pixel_color = VGA_read_pixel(160,120);
//		 sprintf(time_string, "T=%3.0fmS  color=%x    ", elapsedTime, pixel_color);
//     sprintf(loc_string, "row=%d, col=%d    ", *(pio_row), *(pio_col)); 
//     sprintf(color_string, "color=%x    ", *(pio_color)); 
//		// VGA_text (10, 3, num_string);
//		 VGA_text (1, 58, time_string);
//      VGA_text(1, 1, loc_string);
//      VGA_text(1, 2, color_string);
		
	} // end while(1)
} // end main

/****************************************************************************************
 * Subroutine to read a pixel from the video input 
****************************************************************************************/
int  video_in_read_pixel(int x, int y){
	char  *pixel_ptr ;
	pixel_ptr = (char *)video_in_ptr + ((y)<<9) + (x) ;
	return *pixel_ptr ;
}

int sdram_grayscale(int x, int y){
  char  *pixel_ptr ;
	pixel_ptr = (char *)gray_image_values + ((y)<<9) + (x) ;
	return *pixel_ptr ;
}

/****************************************************************************************
 * Subroutine to read a pixel from the VGA monitor 
****************************************************************************************/
int  VGA_read_pixel(int x, int y){
	char  *pixel_ptr ;
	pixel_ptr = (char *)vga_pixel_ptr + ((y)<<10) + (x) ;
	return *pixel_ptr ;
}

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset;
	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		// write to the character buffer
		*(character_buffer + offset) = *(text_ptr);	
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor 
****************************************************************************************/
void VGA_text_clear()
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset, x, y;
	for (x=0; x<79; x++){
		for (y=0; y<59; y++){
	/* assume that the text string fits on one line */
			offset = (y << 7) + x;
			// write to the character buffer
			*(character_buffer + offset) = ' ';		
		}
	}
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//640x480
			pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
			// set pixel color
			*(char *)pixel_ptr = pixel_color;		
		}
}

/****************************************************************************************
 * Draw a filled circle on the VGA monitor 
****************************************************************************************/

void VGA_disc(int x, int y, int r, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++)
		for (xc = -r; xc <= r; xc++)
		{
			col = xc;
			row = yc;
			// add the r to make the edge smoother
			if(col*col+row*row <= rsqr+r){
				col += x; // add the center point
				row += y; // add the center point
				//check for valid 640x480
				if (col>639) col = 639;
				if (row>479) row = 479;
				if (col<0) col = 0;
				if (row<0) row = 0;
				pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
				// set pixel color
				//nanosleep(&delay_time, NULL);
				draw_delay();
				*(char *)pixel_ptr = pixel_color;
			}
					
		}
}

// =============================================
// === Draw a line
// =============================================
//plot a line 
//at x1,y1 to x2,y2 with color 
//Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
	int e;
	signed int dx,dy,j, temp;
	signed int s1,s2, xchange;
     signed int x,y;
	char *pixel_ptr ;
	
	/* check and fix line coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
        
	x = x1;
	y = y1;
	
	//take absolute value
	if (x2 < x1) {
		dx = x1 - x2;
		s1 = -1;
	}

	else if (x2 == x1) {
		dx = 0;
		s1 = 0;
	}

	else {
		dx = x2 - x1;
		s1 = 1;
	}

	if (y2 < y1) {
		dy = y1 - y2;
		s2 = -1;
	}

	else if (y2 == y1) {
		dy = 0;
		s2 = 0;
	}

	else {
		dy = y2 - y1;
		s2 = 1;
	}

	xchange = 0;   

	if (dy>dx) {
		temp = dx;
		dx = dy;
		dy = temp;
		xchange = 1;
	} 

	e = ((int)dy<<1) - dx;  
	 
	for (j=0; j<=dx; j++) {
		//video_pt(x,y,c); //640x480
		pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x; 
		// set pixel color
		*(char *)pixel_ptr = c;	
		 
		if (e>=0) {
			if (xchange==1) x = x + s1;
			else y = y + s2;
			e = e - ((int)dx<<1);
		}

		if (xchange==1) y = y + s2;
		else x = x + s1;

		e = e + ((int)dy<<1);
	}
}

/////////////////////////////////////////////

#define NOP10() asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop")

void draw_delay(void){
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10(); //16
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10(); //32
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10(); //48
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10(); //64
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10(); //68
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10(); //80
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	// NOP10(); NOP10(); NOP10(); NOP10();
	NOP10(); NOP10(); NOP10(); NOP10(); //96
}

/// /// ///////////////////////////////////// 
/// end /////////////////////////////////////