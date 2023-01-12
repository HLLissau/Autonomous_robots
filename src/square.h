#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "componentserver.h"
#include "rhd.h"
#include "xmlio.h"
struct xml_in *xmldata;
struct xml_in *xmllaser;
struct
{
    double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];
volatile int running = 1;
int robot_port;

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *getinputref(const char *sym_name, symTableElement *tab) {
    int i;
    for (i = 0; i < getSymbolTableSize('r'); i++)
        if (strcmp(tab[i].name, sym_name) == 0)
            return &tab[i];
    return 0;
}

symTableElement *getoutputref(const char *sym_name, symTableElement *tab) {
    int i;
    for (i = 0; i < getSymbolTableSize('w'); i++)
        if (strcmp(tab[i].name, sym_name) == 0)
            return &tab[i];
    return 0;
}
/*****************************************
 * odometry
 */
#define WHEEL_DIAMETER 0.06522 /* m */
#define WHEEL_SEPARATION 0.26  /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define DEFAULT_ROBOTPORT 24902

#define FREQUENCY 100
#define ACCELLERATION 0.5
#define TICK_ACCELLERATION ACCELLERATION / FREQUENCY
#define K 0.18
#define K2 3  //
#define LINE_SENSOR_DATA_LENGTH 8
#define LINESENSORDIST 0.0185

time_t start, stop;
double speed = 0.2;
double line_array[LINE_SENSOR_DATA_LENGTH];  // variable som line sensor data skal lægges ind i 7.1
double jarray[LINE_SENSOR_DATA_LENGTH];      // normalisered værdi af line sensor.
typedef struct
{                             // input signals
    int left_enc, right_enc;  // encoderticks
    // parameters
    double w;       // wheel separation
    double cr, cl;  // meters per encodertick
                    // output signals
    double right_pos, left_pos;
    double x, y, theta;                               // tilført 3.2
    double Delta_theta, Delta_U, delta_Ur, delta_Ul;  // tilført 3.2
    int len;                                          // Tilført 5.using zoneobst with square
    double delta_v, theta_ref, theta_ls;              // Tilført 7.1
    int location_line_sensor;                         // 7.2
    // internal variables
    int left_enc_old, right_enc_old;
    float COM;  // 7.3
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);
void writeToFile();
void sm_saveArray();
void calibrateLinesensor();
void read_linesensor();
float center_of_mass(double *intensity_array);
int fwd(double dist, double speed, int time_, int detectLine, int wall_detection, int wall_end_detection);
int rev(double dist, double speed, int time_);
int turn(double angle, double speed, int time_);
int follow_line(double dist, double speed, int time_, int follow, int gate_on_the_loose);
int follow_line_left(double dist, double speed, int time_, int follow);
int follow_line_right(double dist, double speed, int time_, int follow);
double find_laser_min();
int crossdetection(double *array);
int linedetection(double *array);
int detect_gate_on_the_loose();
int detect_wall();
int detect_wall_end();
int substate_box(double dist);
int substate_gate(double dist);
int substate_double_gate(double dist);
int substate_white_line(double dist);
/********************************************
 * Motion control
 */

typedef struct
{  // input
    int cmd;
    int curcmd;
    double speedcmd;
    double dist;
    double angle;
    double left_pos, right_pos;
    // parameters
    double w;
    // output
    double motorspeed_l, motorspeed_r;
    int finished;
    // internal variables
    double startpos;
    // follow line offset to follow left or right
    double follow_line_diff;
} motiontype;

enum {
    mot_stop = 1,
    mot_move,
    mot_rev,
    mot_follow_line,
    mot_turn
};

void update_motcon(motiontype *p);

void segfaulthandler(int sig) {
    //    perror(NULL);
    printf("Seg-error\n");
    exit(1);
}

void brokenpipehandler(int sig) {
    printf("mrc: broken pipe \n");
    // savelog("log");
    exit(1);
}

void ctrlchandler(int sig) {
    printf("mrc: ctrl-c \n");
    running = 0;
}

typedef struct
{
    int state, oldstate, substate;
    int time_;
} smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr,
    *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;
