#include "main.h"
#include "kb.h"
#include "serial.h"
#include "speaker.h"

#define NORMAL 0xFF
#define DEBUG 0xFE

#define BUFFSZ 8

#define NEXT_CH 0

#define NUM_SZ 3


unsigned char READ_FIFO[BUFFSZ] = {0};
unsigned char ir = 0;

unsigned char num_size = 0;
int currentNumber = 0;


void print_error(){
    EA = 0;
    type(EOL);
    type("Invalid arguments.");
    type(EOL);
    EA = 1;
}

void print_num(char num) {
    uart_s_write(num + '0');
}

void print_result() {
	int array[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int count=0;
	int i = 0;
	while (currentNumber > 0) {
		array[count] = currentNumber % 2;
		currentNumber = currentNumber / 2;
		count++;
	}
	for (i = 7; i >= 0; i--) {
		print_num(array[i]);
	}

    ir = 0;
    type(EOL);
}

static int to_num(int *num, unsigned char size) {
    if(size == 3) *num = (READ_FIFO[0] - '0') * 100 + (READ_FIFO[1] - '0') *10 + (READ_FIFO[2] - '0');
    else if (size == 2) *num = (READ_FIFO[0] - '0') * 10 + (READ_FIFO[1] - '0');
	else *num = READ_FIFO[fifo_pos] - '0';
    if(*num > 255) {
        return -1;
    }
    return 0;
}

static int add_char(unsigned char button) {
    READ_FIFO[ir++] = button;
	if(READ_FIFO[ir - 1] == '*') {
       if(num_size == 0) 
        return -1;
	   return 1;
    }
	num_size++;
	return 0;
}

static void reset() {
    num_size = 0;
    ir = 0;
    ET0 = 1;
    currentNumber = 0;
}

static void fail() {
    reset();
    type(EOL);
    print_error();
}


void main() {
    unsigned char dip = 0, button = 0, j = 0;
    int rc = 0;

    uart_s_init(S9600);
    init_kb_timer();
    initialize_speaker();
    
    EA = 1;

    while (1) {
        dip = readdip();
        if (dip == NORMAL) {
            if (!is_queue_empty()) {
                ET0 = 0;
				
                button = get_input();
                if(button=='*') 
                {
                    uart_s_write('=');
                }    
                else (if button >= '0' && button <='9')
                {
                    uart_s_write(button);
                }    
                else 
                {
                    fail();
                    continue;
                }   
                    
				rc = add_char(button);
				if(rc < 0) {
					fail();
					continue;
				}
				
				if(rc==1) {
                    rc = to_num(&currentNumber, num_size);
                    if(rc < 0)
                    {
                        fail();
                        continue;
                    }
					print_result();
					reset();
				}

                ET0 = 1;
            }
        } else if (dip == DEBUG) {
            if (!is_queue_empty()) {
                ET0 = 0;
                uart_s_write(get_input());
                type(EOL);
                ET0 = 1;
            }
        }
        else {
            leds(0xAA);
        }
    }
}
