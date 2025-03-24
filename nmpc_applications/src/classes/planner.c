#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#define CONTIGOUS_CELLS 8
#define MAP 1

#if MAP == 1
    #define SHAPE_I 499
    #define SHAPE_J 499
    #define MAPDIMX 47.0
    #define MAPDIMY 47.0
#endif

//  Compile with: gcc -shared -fPIC -o planner.so planner.c -lm
//  Tested for Ubuntu 24.04

/*
            Pixels to meters transformation (real frame)

            (0, 25) -----------------------------
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
        (500, -25)  -----------------------------
                (0, -25)                    (500, 25)
*/

#define M_I2Y ( -MAPDIMY/2.0 - MAPDIMY/2.0 ) / SHAPE_I
#define B_I2Y MAPDIMY/2.0
#define M_J2X ( MAPDIMX / 2 - (-MAPDIMX / 2) ) / SHAPE_J
#define B_J2X -MAPDIMY/2.0

/*
    Meters to pixels transformation (from real frame)

            (25, 0) |---------------------------|
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
        (-25, 500)  |---------------------------|
                (-25, 0)                    (25, 500)
*/

#define M_Y2I SHAPE_I / (-MAPDIMY)
#define B_Y2I -M_Y2I * MAPDIMY / 2.0
#define M_X2J SHAPE_J / MAPDIMX
#define B_X2J -M_X2J * (-MAPDIMX / 2.0)
#define MAXTIME 999999.0

typedef struct pos {
    double x, y, yaw;
    struct pos* next;
} POSITION;

typedef POSITION* heapPositions;

typedef struct stime {
    double t;
    struct stime* next;
} TEMPO;

typedef TEMPO* heapTimes;

typedef struct ssettings {
    int nrows, ncols, maxCycles;
    double startingPoint[2], goalPoint[2], maskValue, pathGap, goalCheck;
} SETTINGS;

void _transformation(double *x, double *y, char* option)
{
    if (option == NULL) {
        printf("Error: NULL pointer received.\n");
        return;
    }

    if( strcmp(option, "pix2MetersRealFrame" ) == 0 ) {
        double aux_x = M_J2X * (*y) + B_J2X;
        double aux_y = M_I2Y * (*x) + B_I2Y;

        *x = aux_x;
        *y = aux_y;
    }
    if( strcmp(option, "meters2PixRealFrame" ) == 0 ) {
        int aux_x = (int) (M_Y2I * (*y) + B_Y2I);
        int aux_y = (int) (M_X2J * (*x) + B_X2J);

        *x = aux_x;
        *y = aux_y;
    }
}

void* insert_new_time(heapTimes* times, double time) {

    /*
        Insert new time of heap of times
    */

    if( *times == NULL) {
        *times = (heapTimes) malloc(sizeof(TEMPO));

        if(*times == NULL) {
            return NULL;
        }
        
        (*times)->t = time;
        (*times)->next = NULL;

    }
    else {
        insert_new_time(&(*times)->next, time);
        
    }
}

void* _insert_new_point(heapPositions* path, double x, double y, double yaw) {

    /*
        Insert new point on heap of positions
    */

    if( *path == NULL ) {
        *path = (heapPositions) malloc(sizeof(POSITION));

        if(*path == NULL) {
            return NULL;
        }

        (*path)->x = x;
        (*path)->y = y;
        (*path)->yaw = yaw;
        (*path)->next = NULL;

    }
    else {
        _insert_new_point(&(*path)->next, x, y, yaw);

    }
}

void list_times(heapTimes times) {

    if(times == NULL) {
        printf("Não existem elementos.\n");
        return;
    }

    while(times != NULL) {
        printf("Time: %f\n", times->t);
        times = times-> next;
    }
}

void list_masked_times(double* timemasked, int size) {

    double* end = timemasked + size;

    while(timemasked < end) {
        printf("Masked time: %f\n", *timemasked);
        timemasked ++;
    }
}

int return_min_time(heapTimes times, double* times_masked, double* min_time) {

    /*
        Return minimum time cell and minimum cell coordinante among contiguous cells with respect to current cell
    */

    bool flag = false;

    if(times == NULL) {
        printf("Não existem elementos.\n");
        return -1;
    }

    while(times != NULL) {
        if(flag == false) {
            *min_time = times->t;
            flag = true;
        }
        else {
            if(times->t <= *min_time) {
                *min_time = times->t;
            }
        }

        times = times->next;
    }

    if(times_masked == NULL) {
        printf("Não foi encontrado tempo mínimo.\n");
        return -1;
    }

    int min_index = 0;

    while(*times_masked != *min_time) {
        min_index++;
        times_masked++;

        if(times_masked == NULL) {
            printf("Não foi encontrado tempo mínimo.\n");
            return -1;
        }
    }

    return min_index;
}

void _freePath(heapPositions path) {

    /*
        Free alocated memory that saves heap of points (path)
    */

    heapPositions temp;

    while (path != NULL)
    {
        temp = path;
        path = path->next;
        free(temp);
    }
}

void _freeTimes(heapTimes times) {

    /*
        Free alocated memory that saves heap of times
    */

    heapTimes temp;

    while (times != NULL)
    {
        temp = times;
        times = times->next;
        free(temp);
    }
}

heapPositions _getPath2Follow(double* timeMap, SETTINGS data_settings) {

    /*
        Generate path to follow

        :data_settings it contains parameters that guide path generation
    */

    double start_x = data_settings.startingPoint[0];
    double start_y = data_settings.startingPoint[1];

    _transformation(&start_x, &start_y, "meters2PixRealFrame");

    int start_i = start_x;
    int start_j = start_y;

    double new_i, new_j;

    if(start_i < 0 || start_i > data_settings.ncols - 1 || start_j < 0 || start_j > data_settings.nrows - 1) {
        printf("| Out of bonds position.\n");
        return NULL;
    }

    if(timeMap[start_i * data_settings.ncols + start_j] == data_settings.maskValue) {
        printf("| Starting position is masked.\n");
        return NULL;
    }

    //  Initialize path (head of positions)
    heapPositions path = NULL;

    double last_time = timeMap[start_i * data_settings.ncols + start_j];
    double prev_x_mt = data_settings.startingPoint[0];
    double prev_y_mt = data_settings.startingPoint[1];

    //printf("| Compute path by descent of time matrix.\n");
    int index = 0;
    int min_index;
    bool flagMask;
    double aux_time, next_time, next_yaw;

    while(index <= data_settings.maxCycles) {
        int border_indexes[CONTIGOUS_CELLS][2] = { {start_i-1, start_j}, {start_i-1, start_j-1}, {start_i-1, start_j+1},\
                                                    {start_i, start_j - 1}, {start_i, start_j + 1},\
                                                    {start_i + 1, start_j}, {start_i + 1, start_j - 1}, {start_i + 1, start_j + 1} };

        double timesMasked[CONTIGOUS_CELLS];
        heapTimes times = NULL;
        flagMask = false;

        //  Collect times on contiguous cells
        for(int index2 = 0; index2 < CONTIGOUS_CELLS; index2++) {

            if( border_indexes[index2][0] < 0 || border_indexes[index2][0] > data_settings.nrows - 1 ||
                border_indexes[index2][1] < 0 || border_indexes[index2][1] > data_settings.ncols - 1 ) {
                    aux_time = MAXTIME;
            }
            else {
                aux_time = timeMap[ border_indexes[index2][0] * data_settings.ncols + border_indexes[index2][1] ];

                if(aux_time != data_settings.maskValue && aux_time < MAXTIME) {
                    insert_new_time(&times, aux_time);
                    flagMask = true;
                }
            }

            timesMasked[index2] = aux_time;
        }

        if(flagMask == false) {
            printf("| Positions around current positions are all masked. Motion is impossible.\n");
            _freeTimes(times);
            _freePath(path);
            return NULL;
        }

        //  Save last time
        if(index > 0) {
            last_time = next_time;
        }
        
        //  Get new time
        min_index = return_min_time(times, timesMasked, &next_time);

        new_i = (double) border_indexes[min_index][0];
        new_j = (double) border_indexes[min_index][1];

        start_i = (int) new_i;
        start_j = (int) new_j;

        //  Convert pixel coordinates to meters with respect to real frame
        _transformation(&new_i, &new_j, "pix2MetersRealFrame");

        //printf("Index: %d\n", index);
        //printf("%f %f\n", new_i, new_j);
        //printf("%f %f\n", data_settings.goalPoint[0], data_settings.goalPoint[1]);
        //printf("%f\n", data_settings.goalCheck);

        //  Check if global minimum was achieved
        if( sqrt( pow(new_i - data_settings.goalPoint[0], 2) + pow(new_j - data_settings.goalPoint[1], 2) ) <= data_settings.goalCheck  ) {
            printf("| Global minimum was achieved.\n");
            break;
        }
        
        if(next_time > last_time) {
            printf("| Local minimum was achieved.\n");
            break;
        }
        
        //  Add point to path
        if( sqrt( pow(new_i - prev_x_mt, 2) + pow(new_j - prev_y_mt, 2) ) >= data_settings.pathGap ) {
            next_yaw = atan2(new_j - prev_y_mt, new_i - prev_x_mt);
            _insert_new_point(&path, prev_x_mt, prev_y_mt, next_yaw);
            
            prev_x_mt = new_i;
            prev_y_mt = new_j;
            
            index += 1;
        }

        _freeTimes(times);
    }

    //  Fill path until data_settings.maxCycles number of points are in path
    while(index < data_settings.maxCycles) {
        _insert_new_point(&path, new_i, new_j, next_yaw);
        index = index + 1;
    }

    return path;
}