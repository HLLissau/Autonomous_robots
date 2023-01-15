/*
 * An example SMR program.
 *
 */
#include "square.h"
#define TEST 1
#define BLACKLEVEL 0.28

enum {
    ms_init,  // initial state
    ms_end,
    ms_box,
    ms_gate,
    ms_double_gate,
    ms_white_line,
    ms_garage,
    ms_box_measure_distance,
    ms_box_fwd,
    ms_box_follow_line_left,
    ms_box_follow_line,
    ms_box_push,
    ms_box_reverse,
    ms_box_turn,
    ms_box_forward_untill_line,
    ms_box_fwd2,
    ms_box_follow_line_left2,
    ms_box_fwd3,
    ms_box_turn_towards_gates,
    ms_box_follow_line2,
    ms_box_fwd4,
    ms_box_follow_line3,
    ms_follow_line_right,
    ms_gate_fwd1,
    ms_gate_fwd2,
    ms_gate_turn,
    ms_double_gate_fwd1,
    ms_double_gate_turn1,
    ms_double_gate_fwd2,
    ms_double_gate_fwd3,
    ms_double_gate_turn2,
    ms_double_gate_fwd4,
    ms_double_gate_turn3,
    ms_double_gate_drive_to_line,
    ms_double_gate_past_line,
    ms_double_gate_turn4,
    ms_double_gate_rev,
    ms_double_gate_follow_line,
    ms_white_line_fwd1,
    ms_white_line_follow_line1,
    ms_white_line_fwd2,
    ms_white_line_turn,
    ms_white_line_follow_line2,
    ms_garage_fwd1,
    ms_garage_turn1,
    ms_garage_fwd2,
    ms_garage_fwd3,
    ms_garage_turn2,
    ms_garage_fwd4,
    ms_garage_turn3,
    ms_garage_fwd5,
    ms_garage_turn4,
    ms_garage_fwd6,
    ms_garage_turn5,
    ms_garage_fwd7,
    ms_garage_fwd8,
    ms_garage_turn6,
    ms_garage_fwd9,
};

int main(int argc, char **argv) {
    int arg, time_ = 0, opt, calibration;
    double dist = 0, angle = 0;
                    start = time(NULL);
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
            case 'v':
                speed = atoi(optarg) * 0.001;
                break;
            case 's':
                if (optarg) {
                    int port;
                    port = atoi(optarg);
                    if (port != 0)
                        robot_port = port;
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
    mission.substate = ms_init;
    mission.oldstate = -1;
    while (running) {
        if (lmssrv.config && lmssrv.status && lmssrv.connected) {
            while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
                xml_proca(xmllaser);
        }

        if (camsrv.config && camsrv.status && camsrv.connected) {
            while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
                xml_proc(xmldata);
        }

        rhdSync();
        odo.left_enc = lenc->data[0];
        odo.right_enc = renc->data[0];
        update_odo(&odo);

        /****************************************
        / main mission statemachine
        / - Each main case leads to a substate
        ****************************************/
        sm_update(&mission);
        switch (mission.state) {
            case ms_init:

                mission.state = ms_box;
                break;
            case ms_box:
                if (substate_box(dist))
                    mission.state = ms_gate;
                break;
            case ms_gate:
                if (substate_gate(dist))
                    mission.state = ms_double_gate;
                break;
            case ms_double_gate:
                if (substate_double_gate(dist))
                    mission.state = ms_white_line;
                break;
            case ms_white_line:
                if (substate_white_line(dist))
                    mission.state = ms_garage;
                break;
            case ms_garage:
                if (substate_garage(dist))
                    mission.state = ms_end;
                break;
            case ms_end:
                mot.cmd = mot_stop;
                running = 0;
                stop = time(NULL);
                printf("It took %ld seconds to complete the track\n", stop - start);
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
        if (time_ % 100 == 0)
            //    printf(" laser %f \n",laserpar[3]);
            time_++;
        /* stop if keyboard is activated
         *
         */
        ioctl(0, FIONREAD, &arg);
        if (arg != 0)
            running = 0;

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
    sm_saveArray();         /*ADDED*/
    read_linesensor();      // added 7.2
    calibrateLinesensor();  // added 7.2 normaliserer linesensor og finder den mindste værdis placering.
    crossdetection(jarray);
    odo.COM = center_of_mass2(jarray);  // 7.3

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
            case mot_rev:
                p->startpos = (p->left_pos + p->right_pos) / 2;
                p->curcmd = mot_rev;
                break;

            case mot_turn:
                if (p->angle > 0)
                    p->startpos = p->right_pos;
                else
                    p->startpos = p->left_pos;
                p->curcmd = mot_turn;
                break;
            case mot_follow_line:
                p->startpos = (p->left_pos + p->right_pos) / 2;
                p->curcmd = mot_follow_line;
                break;
        }

        p->cmd = 0;
    }

    double d = p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos);
    double d_turn;

    switch (p->curcmd) {
        case mot_stop:
            p->motorspeed_l = 0;
            p->motorspeed_r = 0;
            break;
        case mot_move:
            // 7.1 we change the motors to stay on course
            odo.delta_v = (K * (odo.theta_ref - odo.theta)) / 2;
            p->motorspeed_l = p->motorspeed_l - odo.delta_v;
            p->motorspeed_r = p->motorspeed_r + odo.delta_v;
            // if (p->motorspeed_l<0) p->motorspeed_l=0;
            // if (p->motorspeed_r<0) p->motorspeed_r=0;
            //  3.5)

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
                    p->motorspeed_l = p->speedcmd - odo.delta_v;
                }

                if (p->motorspeed_r < p->speedcmd) {
                    p->motorspeed_r = p->motorspeed_r + TICK_ACCELLERATION;
                } else {
                    p->motorspeed_r = p->speedcmd + odo.delta_v;
                }
            }
            break;
        case mot_rev:
            // 7.1 we change the motors to stay on course
            //  3.5)

            d = p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos);

            if ((p->right_pos + p->left_pos) / 2 - p->startpos < p->dist) {
                p->finished = 1;
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
            } else if (abs(p->motorspeed_l) > sqrt(2 * ACCELLERATION * abs(d))) {  // same speed for each motor due to fwd
                p->motorspeed_l = p->motorspeed_l + TICK_ACCELLERATION;
                p->motorspeed_r = p->motorspeed_r + TICK_ACCELLERATION;
            } else {
                // 3.4.)

                if (p->motorspeed_l > p->speedcmd) {
                    p->motorspeed_l = p->motorspeed_l - TICK_ACCELLERATION;
                } else {
                    p->motorspeed_l = p->speedcmd;
                }

                if (p->motorspeed_r > p->speedcmd) {
                    p->motorspeed_r = p->motorspeed_r - TICK_ACCELLERATION;
                } else {
                    p->motorspeed_r = p->speedcmd + odo.delta_v;
                }
            }
            break;
        case mot_follow_line:;
            double ls = odo.COM + mot.follow_line_diff;
            odo.theta_ls = atan(ls / 0.25);
            odo.delta_v = (K2 * odo.theta_ls);
            // printf("Angle: %.8f \ndel_v: %.8f \nCOM: %.8f\nLS: %.8f\n", odo.theta_ls, odo.delta_v, odo.COM, ls);
            if (odo.delta_v < 0) {
                p->motorspeed_l = p->motorspeed_l;
                p->motorspeed_r = p->motorspeed_l - odo.delta_v;
            } else {
                p->motorspeed_r = p->motorspeed_r;
                p->motorspeed_l = p->motorspeed_r + odo.delta_v;
            }
            // if (p->motorspeed_l<0) p->motorspeed_l=0;
            // if (p->motorspeed_r<0) p->motorspeed_r=0;
            //  3.5)

            if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) {
                p->finished = 1;
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
           /* } else if (p->motorspeed_l > sqrt(2 * ACCELLERATION * d)) {  // same speed for each motor due to fwd
                p->motorspeed_l = p->motorspeed_l - TICK_ACCELLERATION;
                p->motorspeed_r = p->motorspeed_r - TICK_ACCELLERATION;
            */}
           else
           {
               // 3.4.)
               if (p->motorspeed_l < p->speedcmd) {
                   p->motorspeed_l = p->motorspeed_l + TICK_ACCELLERATION;
               } else {
                   p->motorspeed_l = p->speedcmd - odo.delta_v;
               }

               if (p->motorspeed_r < p->speedcmd) {
                   p->motorspeed_r = p->motorspeed_r + TICK_ACCELLERATION;
               } else {
                   p->motorspeed_r = p->speedcmd + odo.delta_v;
               }
           }
           break;

           break;
        case mot_turn:
           d_turn = ((odo.theta_ref - odo.theta) * (odo.w / 2));

           if (p->angle > 0) {
               if (p->motorspeed_r > sqrt(2 * ACCELLERATION * d_turn)) {
                   p->motorspeed_r = p->motorspeed_r - TICK_ACCELLERATION;
                   p->motorspeed_l = p->motorspeed_l + TICK_ACCELLERATION;
               } else if (odo.theta * p->w < odo.theta_ref * p->w) {  // p->right_pos-p->startpos
                   p->motorspeed_l = p->motorspeed_l - TICK_ACCELLERATION;
                   p->motorspeed_r = p->motorspeed_r + TICK_ACCELLERATION;

                   if (p->motorspeed_r > p->speedcmd / 2) {
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
               } else if (odo.theta * p->w > (odo.theta_ref * p->w)) {
                   p->motorspeed_r = p->motorspeed_r - TICK_ACCELLERATION;
                   p->motorspeed_l = p->motorspeed_l + TICK_ACCELLERATION;

                   if (p->motorspeed_l > p->speedcmd / 2) {
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

int fwd(double dist, double speed, int time_, int detect_line, int wall_detection, int wall_end_detection) {
    if (time_ == 0) {
        mot.cmd = mot_move;
        mot.speedcmd = speed;
        mot.dist = dist;
        odo.theta_ref = odo.theta;
        return 0;
    } else {
        if (detect_line && !mot.finished) {
           mot.finished = linedetection(jarray);
        }
        if (wall_detection && !mot.finished) {
           mot.finished = detect_wall();
        }
        if (wall_end_detection && !mot.finished) {
           mot.finished = detect_wall_end();
        }
        return mot.finished;
    }
}
int rev(double dist, double speed, int time_) {
    if (time_ == 0) {
        mot.cmd = mot_rev;
        mot.speedcmd = speed;
        mot.dist = dist;
        return 0;
    } else
        return mot.finished;
}
int turn(double angle, double speed, int time_) {
    if (time_ == 0) {
        mot.cmd = mot_turn;
        mot.speedcmd = speed;
        mot.angle = angle;
        return 0;
    } else
        return mot.finished;
}
int follow_line(double dist, double speed, int time_, int stop_at_cross, int gate_on_the_loose) {
    if (time_ == 0) {
        mot.cmd = mot_follow_line;
        mot.speedcmd = speed;
        mot.dist = dist;
        mot.follow_line_diff = 0;
        return 0;
    } else {
        if (stop_at_cross && !mot.finished) {
           mot.finished = crossdetection(jarray);
        }
        if (gate_on_the_loose && !mot.finished) {
           mot.finished = detect_gate_on_the_loose();
        }
        return mot.finished;
    }
}
int follow_line_left(double dist, double speed, int time_, int stop_at_cross) {
    if (time_ == 0) {
        mot.cmd = mot_follow_line;
        mot.speedcmd = speed;
        mot.dist = dist;
        mot.follow_line_diff = LINESENSORDIST / 5;
        return 0;
    } else {
        if (stop_at_cross && !mot.finished) {
           mot.finished = crossdetection(jarray);
        }
        return mot.finished;
    }
}
int follow_line_right(double dist, double speed, int time_, int stop_at_cross) {
    if (time_ == 0) {
        mot.cmd = mot_follow_line;
        mot.speedcmd = speed;
        mot.dist = dist;
        mot.follow_line_diff = -LINESENSORDIST / 5;
        return 0;
    } else {
        if (stop_at_cross && !mot.finished) {
           mot.finished = crossdetection(jarray);
        }
        return mot.finished;
    }
}
void sm_update(smtype *p) {
    if (p->substate != p->oldstate) {
        p->time_ = 0;
        p->oldstate = p->substate;
        #if TEST 
            stop = time(NULL);
            int time_used = stop-start;
            int min = time_used /60;
            int sec =(int) time_used % 60;
            printf("State %d, substate %d, time used : %d min, %d sec.\n",mission.state, mission.substate,min,sec);
        #endif
    } else {
        p->time_++;
    }
}

void read_linesensor() {
    for (int count = 0; count < LINE_SENSOR_DATA_LENGTH; count++) {
        line_array[count] = linesensor->data[count];
    }
}

void calibrateLinesensor() {
    for (int i = 0; i < LINE_SENSOR_DATA_LENGTH; i++) {
        jarray[i] = (line_array[i] / 255);  // setting higher mass for black
        #if TEST
            //printf("%i: %f.   ",i,jarray[i]);
        #endif
    }
    //printf("\n");
}

float minIntensity(double *intensity_array) {
    int loc = 0;
    for (int c = 1; c < LINE_SENSOR_DATA_LENGTH; c++) {
        if (intensity_array[c] < intensity_array[loc]) {
           loc = c;
        }
    }
    return (loc + 1);
}
// finds a crossline on the current route.
int crossdetection(double *intensity_array) {
    int amount = 0;
    for (int i = 0; i < LINE_SENSOR_DATA_LENGTH; i++) {
        if (intensity_array[i] == 0)
           amount++;
    }
    return (amount > 5);
}
// finds a line while driving.
int linedetection(double *intensity_array) {
    int amount = 0;
    for (int i = 0; i < LINE_SENSOR_DATA_LENGTH; i++) {
        if (intensity_array[i] == 0)
           amount++;
    }
    return (amount > 0);
}

double find_laser_min() {
    double min = laserpar[0];
    for (int i = 1; i < 9; i++) {
        if (laserpar[i] < min) {
           min = laserpar[i];
        }
    }
    return min;
}

int detect_gate_on_the_loose() {
    // As we wan't to be perpendicular to the gate entrance, we only use the left most sensor
    if (laserpar[0] < 1 && laserpar[0] > 0.001) {
#if TEST
        printf("Detected gate on the loose at distance: %f \n", laserpar[0]);
#endif
        return 1;
    } else {
        return 0;
    }
}

int detect_wall() {
    // We only need to detect right in front of us
    if (laserpar[4] < 0.2 && laserpar[4] > 0.001) {
#if TEST
        printf("Detected wall at distance: %f \n", laserpar[4]);
#endif
        return 1;
    } else {
        return 0;
    }
}

int detect_wall_end() {
    // We only need to detect right in front of us
    if (laserpar[8] > 1) {
#if TEST
        printf("Detected that wall ended \n.");
#endif
        return 1;
    } else {
        return 0;
    }
}

// com = center_of_mass(array_with_intensities);
// calculate where the line is in comparison to the line sensors 1-8, where 4.555 is the middle value
float center_of_mass(double *intensity_array) {
    float num = 0;
    float den = 0;

    for (int i = 0; i < LINE_SENSOR_DATA_LENGTH; i++) {
        if (intensity_array[i] > BLACKLEVEL) {
           num += ((i - 3.5) * intensity_array[i] * LINESENSORDIST);
           den += intensity_array[i];
        } else {  // if line is black, we exchange i with i-1
           num += ((i - 3.5) * (1 - intensity_array[i]) * LINESENSORDIST);
           den += (1 - intensity_array[i]);
        }
    }
    float res = num / den;
    float error = 0;  // 0.001035; // An small numerical error measured through simulation
    res = res - error;
    printf("%f \n",res);
    
    return (res);
}

float center_of_mass2(double *intensity_array) {
    float num = 0;
    float den = 0;

    for (int i = 0; i < LINE_SENSOR_DATA_LENGTH; i++) {
           num += ((i - 3.5) * (1-intensity_array[i]) * LINESENSORDIST);
           den += intensity_array[i];
        
    }
    float res = num / den;
    float error = 0;  // 0.001035; // An small numerical error measured through simulation
    res = res - error;
    printf("%f \n",res);
    return (res);
}

int arrayCounter = 0;
float array[25][3 * 60 * 100];
void sm_saveArray() {
    array[0][arrayCounter] = mission.time_;
    array[1][arrayCounter] = mot.motorspeed_l;
    array[2][arrayCounter] = mot.motorspeed_r;
    array[3][arrayCounter] = odo.x;
    array[4][arrayCounter] = odo.y;
    array[5][arrayCounter] = odo.theta;
    for (int i = 0; i < 9; i++) {
        array[6 + i][arrayCounter] = laserpar[i];
    }
    for (int i = 0; i < 8; i++) {
        array[15 + i][arrayCounter] = jarray[i];
    }
    array[23][arrayCounter] = odo.location_line_sensor;
    array[24][arrayCounter] = odo.COM;

    arrayCounter++;
}

void writeToFile() {
    FILE *f1;
    FILE *f2;
    FILE *f3;
    f1 = fopen("/home/smr/offline/square/log.dat", "w");
    f2 = fopen("/home/smr/offline/square/laserlog.dat", "w");
    f3 = fopen("/home/smr/offline/square/linelog.dat", "w");

    for (int i = 0; i < arrayCounter; i++) {
        fprintf(f1, "%.5d  %.3f  %.3f  %.3f  %.3f  %.3f \n", (int)array[0][i],
                array[1][i], array[2][i], array[3][i], array[4][i],
                array[5][i]);
        fprintf(f2,
                "%.5d  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f \n",
                (int)array[0][i], array[6][i], array[7][i], array[8][i],
                array[9][i], array[10][i], array[11][i], array[12][i],
                array[13][i], array[14][i]);
        fprintf(f3,
                "%.5d  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f  %.5d %.3f \n",
                (int)array[0][i], array[15][i], array[16][i], array[17][i],
                array[18][i], array[19][i], array[20][i], array[21][i],
                array[22][i], (int)array[23][i], array[24][i]);
    }

    fclose(f1);
}

int substate_box(double dist) {
    int finished = 0;
    switch (mission.substate) {
        case ms_init:
           mission.substate = ms_box_fwd;
        case ms_box_fwd:
           if (fwd(.3, 0.4, mission.time_, 0, 0, 0))
               mission.substate = ms_box_measure_distance;

           break;
        case ms_box_measure_distance:;
           if (mission.time_ < 20) {
               mission.substate = ms_box_measure_distance;
           } else {
               double min = find_laser_min();
               printf("min laser distance: %f \n", min);
               mission.substate = ms_box_follow_line_left;
           }
           break;
        case ms_box_follow_line_left:
           // 7.3
           if (mission.time_ == 0) {
               odo.theta_ls = 0;
               dist = 1.8;
           }
           // if (mission.time % 25 == 24) odo.theta_ls = odo.theta_ls + 0.1;
           if (follow_line_left(dist, 0.2, mission.time_, 0))

               mission.substate = ms_box_follow_line;

           break;
        case ms_box_follow_line:
           // 7.3
           if (mission.time_ == 0) {
               odo.theta_ls = 0;
               dist = 3.2;
           }
           // if (mission.time % 25 == 24) odo.theta_ls = odo.theta_ls + 0.1;
           if (follow_line(dist, speed, mission.time_, 1, 0))

               mission.substate = ms_box_push;

           break;
        case ms_box_push:
           if (mission.time_ == 0) {
               odo.theta_ref = odo.theta;
               odo.theta_ls = 0;
               speed = 0.3;
               dist = 0.2;
           }
           if (fwd(dist, speed, mission.time_, 0, 0, 0))
               mission.substate = ms_box_reverse;
           break;

        case ms_box_reverse:
           if (mission.time_ == 0) {
               odo.theta_ref = odo.theta;
               odo.theta_ls = 0;

               speed = -0.6;
               dist = -1.2;
           }
           if (rev(dist, speed, mission.time_))
               mission.substate = ms_box_turn;
           break;
        case ms_box_turn:
           if (mission.time_ == 0) {
               odo.theta_ref = (-M_PI / 2.5 + odo.theta);
           }
           if (turn(-M_PI / 2.5, 0.3, mission.time_))
               mission.substate = ms_box_forward_untill_line;

           break;
        case ms_box_forward_untill_line:
           if (fwd(2, 0.2, mission.time_, 1, 0, 0))
               mission.substate = ms_box_fwd2;

           break;
        case ms_box_fwd2:
           if (fwd(0.1, 0.2, mission.time_, 0, 0, 0))
               mission.substate = ms_box_follow_line_left2;
           break;

        case ms_box_follow_line_left2:
           // 7.3
           if (mission.time_ == 0) {
               odo.theta_ls = odo.theta_ref;
               speed = 0.1;
               dist = 1;
           }
           // if (mission.time_ % 25 == 24) odo.theta_ls = odo.theta_ls + 0.1;
           if (follow_line(dist, speed, mission.time_, 1, 0))
               mission.substate = ms_box_fwd3;

           break;

        case ms_box_fwd3:
           if (fwd(0.25, 0.3, mission.time_, 0, 0, 0))
               mission.substate = ms_box_turn_towards_gates;
           break;

        case ms_box_turn_towards_gates:
           if (mission.time_ == 0) {
               odo.theta_ref = (M_PI_2 + odo.theta);
           }
           if (turn(M_PI_2, 0.3, mission.time_))
               mission.substate = ms_box_follow_line2;

           break;
        case ms_box_follow_line2:
           // 7.3
           if (mission.time_ == 0) {
               odo.theta_ls = 0;
               dist = 2;
           }

           // if (mission.time_ % 25 == 24) odo.theta_ls = odo.theta_ls + 0.1;
           if (follow_line(dist, speed, mission.time_, 1, 0))
               mission.substate = ms_box_fwd4;

           break;
        case ms_box_fwd4:
           if (fwd(0.2, 0.3, mission.time_, 0, 0, 0))
               mission.substate = ms_box_follow_line3;
           break;
        case ms_box_follow_line3:
           // 7.3
           if (mission.time_ == 0) {
               odo.theta_ls = 0;
               dist = 2;
           }

           // if (mission.time_ % 25 == 24) odo.theta_ls = odo.theta_ls + 0.1;
           if (follow_line(dist, speed, mission.time_, 1, 0))
               mission.substate = ms_end;

           break;
        case ms_follow_line_right:
           // 7.3
           if (mission.time_ == 0)
               odo.theta_ls = 0;
           // if (mission.time_ % 25 == 24) odo.theta_ls = odo.theta_ls + 0.1;
           if (follow_line_right(dist, speed, mission.time_, 1))
               mission.substate = ms_end;
           break;
        case ms_end:
           finished = 1;
           mission.substate = ms_init;
    }
    return finished;
}

int substate_gate(double dist) {
    int finished = 0;
    switch (mission.substate) {
        case ms_init:
           mission.substate = ms_gate_fwd1;
           break;
        case ms_gate_fwd1:
           if (follow_line(2, 0.05, mission.time_, 0, 1))
               mission.substate = ms_gate_fwd2;
           break;
        case ms_gate_fwd2:
           // Unfortunately the sensor covers a zone of 20 degrees. Therefore we need as small corection of distance.
           if (follow_line(0.65, 0.2, mission.time_, 0, 0))
               mission.substate = ms_gate_turn;
           break;
        case ms_gate_turn:
           if (mission.time_ == 0) {
               odo.theta_ref = (M_PI_2 + odo.theta);
           }
           if (turn(M_PI_2, 0.3, mission.time_))
               mission.substate = ms_end;
           break;
        case ms_end:
           finished = 1;
           mission.substate = ms_init;
           break;
    }
    return finished;
}

int substate_double_gate(double dist) {
    int finished = 0;
    switch (mission.substate) {
        case ms_init:
           mission.substate = ms_double_gate_fwd1;
           break;
        case ms_double_gate_fwd1:
           if (fwd(2, 0.3, mission.time_, 0, 1, 0))  // TODO: Bruge fejlen i den aflæste LS hvis den gør en forskel
               mission.substate = ms_double_gate_turn1;
           break;
        case ms_double_gate_turn1:
           if (mission.time_ == 0) {
               odo.theta_ref = (M_PI_2 + odo.theta);
           }
           if (turn(M_PI_2, 0.3, mission.time_))
               mission.substate = ms_double_gate_fwd2;
           break;
        case ms_double_gate_fwd2:
           if (fwd(1, 0.3, mission.time_, 0, 0, 1))  // TODO: Bruge fejlen i den aflæste LS hvis den gør en forskel
               mission.substate = ms_double_gate_fwd3;
           break;
        case ms_double_gate_fwd3:
           if (fwd(0.45, 0.2, mission.time_, 0, 0, 0))  // TODO: Bruge fejlen i den aflæste LS hvis den gør en forskel
               mission.substate = ms_double_gate_turn2;
           break;
        case ms_double_gate_turn2:
           if (mission.time_ == 0) {
               odo.theta_ref = (-M_PI_2 + odo.theta);
           }
           if (turn(-M_PI_2, 0.3, mission.time_))
               mission.substate = ms_double_gate_fwd4;
           break;
        case ms_double_gate_fwd4:
           if (fwd(0.7, 0.5, mission.time_, 0, 0, 0))  // TODO: Bruge fejlen i den aflæste LS hvis den gør en forskel
               mission.substate = ms_double_gate_turn3;
           break;
        case ms_double_gate_turn3:
           if (mission.time_ == 0) {
               odo.theta_ref = (-M_PI_2 + odo.theta);
           }
           if (turn(-M_PI_2, 0.3, mission.time_))
               mission.substate = ms_double_gate_drive_to_line;
           break;
        case ms_double_gate_drive_to_line:
           if (fwd(2, 0.2, mission.time_, 1, 0, 0))
               mission.substate = ms_double_gate_past_line;
           break;
        case ms_double_gate_past_line:
           if (fwd(0.25, 0.3, mission.time_, 0, 0, 0))
               mission.substate = ms_double_gate_turn4;
           break;
        case ms_double_gate_turn4:
           if (mission.time_ == 0) {
               odo.theta_ref = (M_PI_2 + odo.theta);
           }
           if (turn(M_PI_2, 0.3, mission.time_))
               mission.substate = ms_double_gate_rev;
           break;
        case ms_double_gate_rev:
           if (rev(-0.7, -0.6, mission.time_))
               mission.substate = ms_double_gate_follow_line;
           break;
        case ms_double_gate_follow_line:
           if (follow_line(1.5, 0.3, mission.time_, 1, 0))
               mission.substate = ms_end;
           break;
        case ms_end:
           finished = 1;
           mission.substate = ms_init;
           break;
    }
    return finished;
}

int substate_white_line(double dist) {
    int finished = 0;
    switch (mission.substate) {
        case ms_init:
           mission.substate = ms_white_line_fwd1;
           break;
        case ms_white_line_fwd1:
           if (fwd(0.1, 0.3, mission.time_, 0, 0, 0))
               mission.substate = ms_white_line_follow_line1;
           break;
        case ms_white_line_follow_line1:
           if (follow_line(4.5, 0.3, mission.time_, 1, 0))
               mission.substate = ms_white_line_fwd2;
           break;
        case ms_white_line_fwd2:
           if (fwd(0.25, 0.3, mission.time_, 0, 0, 0))
               mission.substate = ms_white_line_turn;
           break;
        case ms_white_line_turn:
           if (mission.time_ == 0) {
               odo.theta_ref = (-M_PI_2 + odo.theta);
           }
           if (turn(-M_PI_2, 0.3, mission.time_))
               mission.substate = ms_white_line_follow_line2;
           break;
        case ms_white_line_follow_line2:
           if (follow_line(2, 0.3, mission.time_, 1, 0))
               mission.substate = ms_end;
           break;
        case ms_end:
           finished = 1;
           mission.substate = ms_init;
           break;
    }
    return finished;
}

int substate_garage(double dist) {
    int finished = 0;
    switch (mission.substate) {
        case ms_init:
           mission.substate = ms_garage_fwd1;
           break;
        case ms_garage_fwd1:
           if (fwd(0.5, 0.3, mission.time_, 0, 1, 0))
               mission.substate = ms_garage_turn1;
           break;
        case ms_garage_turn1:
           if (mission.time_ == 0) {
               odo.theta_ref = (M_PI_2 + odo.theta);
           }
           if (turn(M_PI_2, 0.3, mission.time_))
               mission.substate = ms_garage_fwd2;
           break;
        case ms_garage_fwd2:
           if (fwd(1.5, 0.3, mission.time_, 0, 0, 1))
               mission.substate = ms_garage_fwd3;
           break;
        case ms_garage_fwd3:
           if (fwd(0.5, 0.5, mission.time_, 0, 0, 0))
               mission.substate = ms_garage_turn2;
           break;
        case ms_garage_turn2:
           if (mission.time_ == 0) {
               odo.theta_ref = (-M_PI_2 + odo.theta);
           }
           if (turn(-M_PI_2, 0.3, mission.time_))
               mission.substate = ms_garage_fwd4;
           break;
        case ms_garage_fwd4:
           if (fwd(1.3, 0.5, mission.time_, 0, 0, 0))
               mission.substate = ms_garage_turn3;
           break;
        case ms_garage_turn3:
           if (mission.time_ == 0) {
               odo.theta_ref = (-M_PI_2 + odo.theta);
           }
           if (turn(-M_PI_2, 0.3, mission.time_))
               mission.substate = ms_garage_fwd5;
           break;
        case ms_garage_fwd5:
           if (fwd(0.11, 0.5, mission.time_, 0, 0, 0))
               mission.substate = ms_garage_turn4;
           break;
        case ms_garage_turn4:
           if (mission.time_ == 0) {
               odo.theta_ref = (-M_PI_2 + odo.theta);
           }
           if (turn(-M_PI_2, 0.3, mission.time_))
               mission.substate = ms_garage_fwd6;
           break;
        case ms_garage_fwd6:
           if (fwd(1.6, 0.5, mission.time_, 0, 0, 0))
               mission.substate = ms_garage_turn5;
           break;
        // move inside garage
        case ms_garage_turn5:
           if (mission.time_ == 0) {
               odo.theta_ref = (M_PI_2 + odo.theta);
           }
           if (turn(M_PI_2, 0.3, mission.time_))
               mission.substate = ms_garage_fwd7;
           break;
        case ms_garage_fwd7:
           if (fwd(0.5, 0.5, mission.time_, 1, 0, 0))
               mission.substate = ms_garage_fwd8;
           break;
        case ms_garage_fwd8:
           if (fwd(0.16, 0.5, mission.time_, 0, 0, 0))
               mission.substate = ms_garage_turn6;
           break;
        case ms_garage_turn6:
           if (mission.time_ == 0) {
               odo.theta_ref = (M_PI_2 + odo.theta);
           }
           if (turn(M_PI_2, 0.3, mission.time_))
               mission.substate = ms_garage_fwd9;
           break;
        case ms_garage_fwd9:
           if (fwd(0.5, 0.5, mission.time_, 0, 1, 0))
               mission.substate = ms_end;
           break;
        case ms_end:
           finished = 1;
           mission.substate = ms_init;
           break;
    }
    return finished;
}