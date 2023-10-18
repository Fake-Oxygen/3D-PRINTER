#include "reader.h"

int is_end_char(char c) {
    return c == '\0' || c == END_SYMBOL || c == END_SYMBOL2;
}

int is_type_char(char c) {
    return c == GCODE_SYMBOL || c == MCODE_SYMBOL;
}

int read_to_buff(char c, gc_reader *reader) {
    if(reader->buff_iter >= BUFF_SIZE - 1) {
        return 1;
    }
    reader->buff[reader->buff_iter++] = c;
    reader->buff[reader->buff_iter] = '\0';
    return 0;
}

void set_param(gc_reader *reader) {
    switch(reader->curr_param_type) {
    case 'T':
        reader->T = atoi(reader->buff);
        break;
    case 'S':
        reader->S = atof(reader->buff);
        break;
    case 'P':
        reader->P = atoi(reader->buff);
        break;
    case 'X':
        reader->X = atof(reader->buff);
        break;
    case 'Y':
        reader->Y = atof(reader->buff);
        break;
    case 'Z':
        reader->Z = atof(reader->buff);
        break;
    case 'U':
        reader->U = atof(reader->buff);
        break;
    case 'V':
        reader->V = atof(reader->buff);
        break;
    case 'W':
        reader->W = atof(reader->buff);
        break;
    case 'I':
        reader->I = atof(reader->buff);
        break;
    case 'J':
        reader->J = atof(reader->buff);
        break;
    case 'D':
        reader->D = atof(reader->buff);
        break;
    case 'H':
        reader->H = atof(reader->buff);
        break;
    case 'F':
        reader->F = atof(reader->buff);
        break;
    case 'R':
        reader->R = atof(reader->buff);
        break;
    case 'E':
        reader->E = atof(reader->buff);
        break;
    }
    reader->buff_iter = 0;
}

int is_checksum_ok(char *input, gc_reader *reader) {
    int cs = 0;
    for(int i = 0; input[i] != CHECKSUM_SYMBOL && input[i] != '\0'; i++) {
        cs = cs ^ input[i];
    }
    cs &= 0xff;
    return cs == reader->checksum;
}

int read_code(gc_reader *reader, char input, int *is_done) {
    if(input == COMMENT_SYMBOL) {
        reader->state = GC_READER_STATE_IGNORE_INPUT;
    }
    if(is_end_char(input)) {
        *is_done = 1;
        reader->read_checksum &= 0xff;
    } else if(input != CHECKSUM_SYMBOL && reader->state != GC_READER_STATE_READ_CHECKSUM) {
        reader->read_checksum = reader->read_checksum ^ input;
    }


    int should_read_checksum_next = 0;
    if(input == CHECKSUM_SYMBOL) {
        reader->found_checksum = 1;
        should_read_checksum_next = 1;
    }

    if(!*is_done && !should_read_checksum_next) {
        switch(reader->state) {
        case GC_READER_STATE_READ_TYPE:
            if(input == LINE_NUMBER_SYMBOL) {
                reader->state = GC_READER_STATE_READ_LINE_NUMBER;
                break;
            } else if(!is_type_char(input)) {
                return GC_READER_ERROR_TYPE_READ;
            }
            reader->code_type = input;
            reader->state = GC_READER_STATE_READ_CODE;
            break;
        case GC_READER_STATE_READ_CODE:
            if(input == SEPARATOR_SYMBOL) {
                if(reader->buff_iter > 0) {
                    reader->code_id = atoi(reader->buff);
                } else {
                    return GC_READER_ERROR_CODE_READ;
                }
                reader->buff_iter = 0;
                reader->state = GC_READER_STATE_READ_PARAM_TYPE;
                break;
            }
            if(read_to_buff(input, reader)) {
                return GC_READER_ERROR_BUFF_OVERFLOW;
            }
            break;
        case GC_READER_STATE_READ_PARAM_TYPE:
            reader->curr_param_type = input;
            reader->state = GC_READER_STATE_READ_PARAM;
            break;
        case GC_READER_STATE_READ_PARAM:
            if(input == SEPARATOR_SYMBOL) {
                if(reader->buff_iter > 0) {
                    set_param(reader);
                } else {
                    return GC_READER_ERROR_PARAM_READ;
                }
                reader->state = GC_READER_STATE_READ_PARAM_TYPE;
                break;
            }
            if(read_to_buff(input, reader)) {
                return GC_READER_ERROR_BUFF_OVERFLOW;
            }
            break;
        case GC_READER_STATE_READ_LINE_NUMBER:
            if(input == SEPARATOR_SYMBOL) {
                reader->line_number = atoi(reader->buff);
                reader->buff_iter = 0;
                reader->state = GC_READER_STATE_READ_TYPE;
                break;
            }
            if(read_to_buff(input, reader)) {
                return GC_READER_ERROR_BUFF_OVERFLOW;
            }
            break;
        case GC_READER_STATE_READ_CHECKSUM:
            if(input == SEPARATOR_SYMBOL) {
                reader->checksum = atoi(reader->buff);
                reader->buff_iter = 0;
                reader->state = GC_READER_STATE_IGNORE_INPUT;
                break;
            }
            if(read_to_buff(input, reader)) {
                return GC_READER_ERROR_BUFF_OVERFLOW;
            }
            break;
        }
    }

    if((*is_done && reader->buff_iter != 0) || should_read_checksum_next) {
        switch(reader->state) {
        case GC_READER_STATE_READ_CODE:
            reader->code_id = atoi(reader->buff);
            reader->buff_iter = 0;
            break;
        case GC_READER_STATE_READ_PARAM:
            set_param(reader);
            break;
        case GC_READER_STATE_READ_LINE_NUMBER:
            reader->line_number = atoi(reader->buff);
            reader->buff_iter = 0;
            break;
        case GC_READER_STATE_READ_CHECKSUM:
            reader->checksum = atoi(reader->buff);
            reader->buff_iter = 0;
            reader->state = GC_READER_STATE_IGNORE_INPUT;

            return GC_READER_ERROR_NOT_OCCURED;
            break;
        }
        reader->state = !should_read_checksum_next ? GC_READER_STATE_READ_TYPE : GC_READER_STATE_READ_CHECKSUM;
    }

    return GC_READER_ERROR_NOT_OCCURED;
}
