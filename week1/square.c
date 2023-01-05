/*
 * An example SMR program.
 *
 */
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
struct {
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
        if (strcmp(tab[i].name, sym_name) == 0) return &tab[i];
    return 0;
}

symTableElement *getoutputref(const char *sym_name, symTableElement *tab) {
    int i;
    for (i = 0; i < getSymbolTableSize('w'); i++)
        if (strcmp(tab[i].name, sym_name) == 0) return &tab[i];
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
#define K 0.1

typedef struct {              // input signals
    int left_enc, right_enc;  // encoderticks
    // parameters
    double w;       // wheel separation
    double cr, cl;  // meters per encodertick
                    // output signals
    double right_pos, left_pos;
    double x, y, theta, theta_b;                      // tilført 3.2
    double Delta_theta, Delta_U, delta_Ur, delta_Ul;  // tilført 3.2
    int len;                                          // Tilført 5.using zoneobst with square
    double delta_v;                                   // Tilført 7.1
    // internal variables
    int left_enc_old, right_enc_old;
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);
void writeToFile();
void sm_saveArray();

/********************************************
 * Motion control
 */

typedef struct {  // input
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
} motiontype;

enum { mot_stop = 1,
       mot_move,
       mot_turn };

void update_motcon(motiontype *p);

int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);

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

typedef struct {
    int state, oldstate;
    int time;
} smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr,
    *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum { ms_init,
       ms_fwd,
       ms_turn,
       ms_end };

int main(int argc, char **argv) {
    int n = 0, arg, time = 0, opt, calibration;
    double dist = 0, angle = 0;
    // install sighandlers
    if (1) {
        if (signal(SIGSEGV, segfaulthandler) == SIG_ERR) {
            perror("signal");
            exit(1);
        }
    }
    if (signal(SIGPIPE, brokenpipehandler) == SIG_ERR) {
        perror("signal");
        exit(1);
    }
    if (signal(SIGINT, ctrlchandler) == SIG_ERR) {
        perror("signal");
        exit(1);
    }
    robot_port = DEFAULT_ROBOTPORT;
    while (EOF != (opt = getopt(argc, argv, "ct:v:l:s:h:u"))) {
        switch (opt) {
            case 'c':
                calibration = 1;
                break;

            case 's':
                if (optarg) {
                    int port;
                    port = atoi(optarg);
                    if (port != 0) robot_port = port;
                } else
                    exit(1);
                break;

            default:;
        }
    }

    /* Establish connection to robot sensors and actuators.
     */
    if (rhdConnect('w', "localhost", robot_port) != 'w') {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }

    printf("connected to robot \n");
    if ((inputtable = getSymbolTable('r')) == NULL) {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }
    if ((outputtable = getSymbolTable('w')) == NULL) {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }
    // connect to robot I/O variables
    lenc = getinputref("encl", inputtable);
    renc = getinputref("encr", inputtable);
    linesensor = getinputref("linesensor", inputtable);
    irsensor = getinputref("irsensor", inputtable);

    speedl = getoutputref("speedl", outputtable);
    speedr = getoutputref("speedr", outputtable);
    resetmotorr = getoutputref("resetmotorr", outputtable);
    resetmotorl = getoutputref("resetmotorl", outputtable);

    // **************************************************
    //  Camera server code initialization
    //

    /* Create endpoint */
    lmssrv.port = 24919;
    strcpy(lmssrv.host, "127.0.0.1");
    strcpy(lmssrv.name, "laserserver");
    lmssrv.status = 1;
    camsrv.port = 24920;
    strcpy(camsrv.host, "127.0.0.1");
    camsrv.config = 1;
    strcpy(camsrv.name, "cameraserver");
    camsrv.status = 1;

    if (camsrv.config) {
        int errno = 0;
        camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (camsrv.sockfd < 0) {
            perror(strerror(errno));
            fprintf(stderr, " Can not make  socket\n");
            exit(errno);
        }

        serverconnect(&camsrv);

        xmldata = xml_in_init(4096, 32);
        printf(" camera server xml initialized \n");
    }

    // **************************************************
    //  LMS server code initialization
    //

    /* Create endpoint */
    lmssrv.config = 1;
    if (lmssrv.config) {
        char buf[256];
        int errno = 0, len;
        lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (lmssrv.sockfd < 0) {
            perror(strerror(errno));
            fprintf(stderr, " Can not make  socket\n");
            exit(errno);
        }

        serverconnect(&lmssrv);
        if (lmssrv.connected) {
            xmllaser = xml_in_init(4096, 32);
            printf(" laserserver xml initialized \n");
            len = sprintf(buf, "scanpush cmd='zoneobst'\n");
            odo.len = len;
            send(lmssrv.sockfd, buf, len, 0);
        }
    }

    /* Read sensors and zero our position.
     */
    rhdSync();

    odo.w = 0.256;
    odo.cr = DELTA_M;
    odo.cl = odo.cr;
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];
    reset_odo(&odo);
    printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
    mot.w = odo.w;
    running = 1;
    mission.state = ms_init;
    mission.oldstate = -1;
    while (running) {
        if (lmssrv.config && lmssrv.status && lmssrv.connected) {
            while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
                xml_proca(xmllaser);
        }

        if (camsrv.config && camsrv.status && camsrv.connected) {
            while ((xml_in_fd(xmldata, camsrv.sockfd) > 0)) xml_proc(xmldata);
        }

        rhdSync();
        odo.left_enc = lenc->data[0];
        odo.right_enc = renc->data[0];
        update_odo(&odo);

        /****************************************
        / mission statemachine
        */
        sm_update(&mission);
        switch (mission.state) {
            case ms_init:
                n = 4;
                dist = 1;
                angle = -90.0 / 180 * M_PI;
                mission.state = ms_fwd;
                break;

            case ms_fwd:
                if (fwd(dist, 0.3, mission.time)) mission.state = ms_turn;

                // 3.3)
                // if (fwd(2,0.2,mission.time))  mission.state=ms_end;
                // if (fwd(2,0.4,mission.time))  mission.state=ms_end;
                // if (fwd(2,0.6,mission.time))  mission.state=ms_end;

                break;

            case ms_turn:
                if (mission.time == 0) odo.theta_b = (angle + odo.theta);
                if (turn(angle, 0.3, mission.time)) {
                    n = n - 1;
                    if (n == 0)
                        mission.state = ms_end;
                    else
                        mission.state = ms_fwd;
                }
                break;

            case ms_end:
                mot.cmd = mot_stop;
                running = 0;
                break;
        }
        /*  end of mission  */

        mot.left_pos = odo.left_pos;
        mot.right_pos = odo.right_pos;
        update_motcon(&mot);
        speedl->data[0] = 100 * mot.motorspeed_l;
        speedl->updated = 1;
        speedr->data[0] = 100 * mot.motorspeed_r;
        speedr->updated = 1;
        if (time % 100 == 0)
            //    printf(" laser %f \n",laserpar[3]);
            time++;
        /* stop if keyboard is activated
         *
         */
        ioctl(0, FIONREAD, &arg);
        if (arg != 0) running = 0;

    } /* end of main control loop */
    speedl->data[0] = 0;
    speedl->updated = 1;
    speedr->data[0] = 0;
    speedr->updated = 1;
    rhdSync();
    rhdDisconnect();

    /* Write the results */
    writeToFile();
    exit(0);
}

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

void reset_odo(odotype *p) {
    p->right_pos = p->left_pos = 0.0;
    p->right_enc_old = p->right_enc;
    p->left_enc_old = p->left_enc;
    p->x = p->y = p->theta = 0.0;  // 3.2
}

void update_odo(odotype *p) {
    int delta;

    delta = p->right_enc - p->right_enc_old;
    if (delta > 0x8000)
        delta -= 0x10000;
    else if (delta < -0x8000)
        delta += 0x10000;
    p->right_enc_old = p->right_enc;
    p->delta_Ur = delta * p->cr;
    p->right_pos += p->delta_Ur;

    delta = p->left_enc - p->left_enc_old;
    if (delta > 0x8000)
        delta -= 0x10000;
    else if (delta < -0x8000)
        delta += 0x10000;
    p->left_enc_old = p->left_enc;
    p->delta_Ul = delta * p->cl;
    p->left_pos += p->delta_Ul;

    // tilført
    p->Delta_U = (p->delta_Ur + p->delta_Ul) / 2;
    p->Delta_theta = (p->delta_Ur - p->delta_Ul) / WHEEL_SEPARATION;
    p->theta += p->Delta_theta;
    p->x += p->Delta_U * cos(p->theta);
    p->y += p->Delta_U * sin(p->theta);
    // printf("%f    %f     %f\n",p->x,p->y,p->theta);
    //  tilført
}

void update_motcon(motiontype *p) {
    sm_saveArray(); /*ADDED*/

    if (p->cmd != 0) {
        p->finished = 0;
        switch (p->cmd) {
            case mot_stop:
                p->curcmd = mot_stop;
                break;
            case mot_move:
                p->startpos = (p->left_pos + p->right_pos) / 2;
                p->curcmd = mot_move;
                break;

            case mot_turn:
                if (p->angle > 0)
                    p->startpos = p->right_pos;
                else
                    p->startpos = p->left_pos;
                p->curcmd = mot_turn;
                break;
        }

        p->cmd = 0;
    }

    double d;
    double d_turn;
    odo.delta_v = K * (odo.theta_b-odo.theta)*0.1;
    switch (p->curcmd) {
        case mot_stop:
            p->motorspeed_l = 0;
            p->motorspeed_r = 0;
            break;
        case mot_move:
            // 7.1 we change the motors to stay on course

            p->motorspeed_l = p->motorspeed_l-odo.delta_v;
            p->motorspeed_r = p->motorspeed_l+odo.delta_v;
            // 3.5)
            d = p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos);
            if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) {
                p->finished = 1;
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
            } else if (p->motorspeed_l > sqrt(2 * ACCELLERATION * d)) {  // same speed for each motor due to fwd
                p->motorspeed_l = p->motorspeed_l - TICK_ACCELLERATION;
                p->motorspeed_r = p->motorspeed_r - TICK_ACCELLERATION;
            } else {
                // 3.4.)
                if (p->motorspeed_l < p->speedcmd) {
                    p->motorspeed_l = p->motorspeed_l + TICK_ACCELLERATION;
                } else {
                    p->motorspeed_l = p->speedcmd;
                }

                if (p->motorspeed_r < p->speedcmd) {
                    p->motorspeed_r = p->motorspeed_r + TICK_ACCELLERATION;
                } else {
                    p->motorspeed_r = p->speedcmd;
                }
            }
            break;

        case mot_turn:
            d_turn = ((odo.theta_b - odo.theta) * (odo.w / 2));
            
            if (p->angle > 0) {
                
                if (p->motorspeed_r > sqrt(2 * ACCELLERATION * d_turn)) {
                    p->motorspeed_r = p->motorspeed_r - TICK_ACCELLERATION;
                    p->motorspeed_l = p->motorspeed_l + TICK_ACCELLERATION;
                } else if (odo.theta * p->w < odo.theta_b * p->w) {  // p->right_pos-p->startpos
                    p->motorspeed_l = p->motorspeed_l - TICK_ACCELLERATION;
                    p->motorspeed_r = p->motorspeed_r + TICK_ACCELLERATION;

                    if (p->motorspeed_r > odo.delta_v) {
                        p->motorspeed_r = p->speedcmd / 2;
                        p->motorspeed_l = -p->speedcmd / 2;
                    }
                } else {
                    p->motorspeed_r = 0;
                    p->motorspeed_l = 0;
                    p->finished = 1;
                }
            } else {
                if (p->motorspeed_l > sqrt(2 * ACCELLERATION * fabs(d_turn))) {  // same speed for each motor due to fwd
                    p->motorspeed_l = p->motorspeed_l - TICK_ACCELLERATION;
                    p->motorspeed_r = p->motorspeed_r + TICK_ACCELLERATION;
                } else if (odo.theta * p->w > (odo.theta_b * p->w)) {
                    p->motorspeed_r = p->motorspeed_r - TICK_ACCELLERATION;
                    p->motorspeed_l = p->motorspeed_l + TICK_ACCELLERATION;

                    if (p->motorspeed_l > odo.delta_v) {
                        p->motorspeed_r = -p->speedcmd / 2;
                        p->motorspeed_l = p->speedcmd / 2;
                    }
                }

                else {
                    p->motorspeed_l = 0;
                    p->motorspeed_r = 0;
                    p->finished = 1;
                }
            } 

            break;
    }
}

int fwd(double dist, double speed, int time) {
    if (time == 0) {
        mot.cmd = mot_move;
        mot.speedcmd = speed;
        mot.dist = dist;
        return 0;
    } else
        return mot.finished;
}

int turn(double angle, double speed, int time) {
    if (time == 0) {
        mot.cmd = mot_turn;
        mot.speedcmd = speed;
        mot.angle = angle;
        return 0;
    } else
        return mot.finished;
}

void sm_update(smtype *p) {
    if (p->state != p->oldstate) {
        p->time = 0;
        p->oldstate = p->state;
    } else {
        p->time++;
    }
}

int arrayCounter = 0;
float array[16][10000];
void sm_saveArray() {
    array[0][arrayCounter] = mission.time;
    array[1][arrayCounter] = mot.motorspeed_l;
    array[2][arrayCounter] = mot.motorspeed_r;
    array[3][arrayCounter] = odo.x;
    array[4][arrayCounter] = odo.y;
    array[5][arrayCounter] = odo.theta;
    for (int i = 0; i < 9; i++) {
        array[6 + i][arrayCounter] = laserpar[i];
    }

    arrayCounter++;
}

void writeToFile() {
    FILE *f1;
    FILE *f2;
    f1 = fopen("/home/smr/offline/square/log.dat", "w");
    f2 = fopen("/home/smr/offline/square/laserlog.dat", "w");
    for (int i = 0; i < arrayCounter; i++) {
        fprintf(f1, "%.5d ,%.3f, %.3f, %.3f, %.3f, %.3f \n", (int)array[0][i],
                array[1][i], array[2][i], array[3][i], array[4][i],
                array[5][i]);
        fprintf(f2,
                "%.5d ,%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f \n",
                (int)array[0][i], array[6][i], array[7][i], array[8][i],
                array[9][i], array[10][i], array[11][i], array[12][i],
                array[13][i], array[14][i]);
    }

    fclose(f1);
}