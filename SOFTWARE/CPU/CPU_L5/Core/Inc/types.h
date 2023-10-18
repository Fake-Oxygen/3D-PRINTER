#ifndef GC_TYPES_H
#define GC_TYPES_H

#include <stdint.h>

#define BUFF_SIZE 20
#define FREE 0
#define RUNNING 1
#define WAIT_FOR_BED_TEMP 2
#define WAIT_FOR_HE1_TEMP 3

#define ABSOLUTE_MODE 0
#define RELATIVE_MODE 1

#define PI 3.141592

typedef struct gc_reader {
    uint8_t state;
    uint8_t buff[BUFF_SIZE];
    uint32_t buff_iter;
    uint8_t curr_param_type;
    uint8_t found_checksum;

    uint8_t code_type;
    uint32_t code_id;
    
    uint32_t T;
    double S;
    uint32_t P;
    
    double X;
    double Y;
    double Z;
    double U;
    double V;
    double W;
    
    double I;
    double J;
    double D;
    double H;

    double F;
    double R;
    double E;

    uint32_t line_number;
    uint32_t checksum;
    uint32_t read_checksum;
} gc_reader;

typedef struct vector2D
{
	double x;
	double y;
} vector2D;

typedef struct header {

	uint16_t temp_he0;
	uint16_t temp_he1;
	uint16_t temp_bed;

	double X;
	double Y;
	double Z;
	double E;
	double F;

	uint32_t last_time_E;
	uint32_t last_time_X;
	uint32_t last_time_Y;
	uint32_t last_time_Z;

	uint8_t isrunning;

	uint16_t CurStepsX;
	uint16_t CurStepsY;
	uint16_t CurStepsE;
	uint16_t CurStepsZ;

	uint16_t StepsX;
	uint16_t StepsY;
	uint16_t StepsE;
	uint16_t StepsZ;

	uint8_t extruder_mode;
	uint16_t normal_accel;
	uint16_t retract_accel;

	vector2D AB_delta;
	vector2D d;

} header;



#endif
