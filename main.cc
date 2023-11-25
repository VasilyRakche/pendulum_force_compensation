

#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>
#include <sstream>
#include <Eigen/Dense>

#include <mujoco/mujoco.h>
#include <mujoco/mjtnum.h>
#include <map>
#include "GLFW/glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define ndof 2
#define SPECIAL_GROUP 777
#define IS_SPECIAL_INSTANCE(model, instance) \
    (model->actuator_group[instance] == SPECIAL_GROUP)

//related to writing data to a file
int loop_index = 0;
const int data_frequency = 50; //frequency at which data is written to a file
static mjtNum K_act = 15;
static mjtNum q_b = 0.01;

char xmlpath[] = "./tripplependulum.xml";

typedef Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXn_t;
typedef Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> VectorXn_t;

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

mjtNum custom_gain(const mjModel *m, const mjData *d, int instance) {
    // In forward computation gain is used as follows: gain*act[end]
    // For activation dimensions bigger than 1,
    // the last element is used to generate force.
    // scalar_force = gain_term * (act or ctrl) + bias_term
    return 0;  // question if needed for this kind of dynamics
}

mjtNum custom_bias(const mjModel *m, const mjData *d, int instance) {
    // In forward computation bias is used as follows: gain*act[end] + bias
    return 0;
}

//**************************
void mycontroller(const mjModel* m, mjData* d)
{
  // Write control here

  // Not implemented to show the custom dynamics
  // on its own

  if ( loop_index%data_frequency==0) {
    std::printf("q1:%f\tq2:%f\tq3:%f\n",d->qpos[0],d->qpos[1],d->qpos[2]);
    //   save_data(m,d);
    }
  loop_index = loop_index + 1;
}

mjtNum custom_dynamics(const mjModel *m, const mjData *d, int instance) {

    if (instance == m->njnt - 1) {
        // only in last actuator instance update all actuators
        Eigen::Map<VectorXn_t> qfrc_bias(d->qfrc_bias, m->nv);
        Eigen::Map<VectorXn_t> qfrc_constraint(d->qfrc_constraint, m->nv);
        Eigen::Map<VectorXn_t> qfrc_ctrl(d->ctrl, m->nv);
        // VectorXn_t qfrc_ctrl = VectorXn_t::Zero(m->nv);    
        VectorXn_t qfrc_damp = VectorXn_t::Zero(m->nv);    

        // Compute control forces
        for (int i = 0; i < m->nv; i++) {
            mjtNum *act = d->act + m->actuator_actadr[i];

            qfrc_ctrl[i] = K_act * act[1]; // Ctrl is acting through double integrator
            qfrc_damp[i] = q_b * d->qvel[i]; // Custom dumping
        }

        // Mass matrix
        mjtNum mjt_qM_dense[m->nv * m->nv];  // array to hold qM
        mj_fullM(m, mjt_qM_dense, d->qM);    // qM originally provided as sparse
        Eigen::Map<MatrixXn_t> B(mjt_qM_dense, m->nv,
                                        m->nv);  // create eigen matrix
        MatrixXn_t B_inv = B.inverse();

        // Selector matrix to chose joint for which to compensate all external forces
        MatrixXn_t c_sel(1, m->nv);
        c_sel << 1, 0, 0;

        // Forces to compensate all external forces
        MatrixXn_t M_c = c_sel * B_inv * c_sel.transpose();
        VectorXn_t qfrc_all = qfrc_bias + qfrc_damp - qfrc_constraint - qfrc_ctrl;
        VectorXn_t Lambda = M_c.inverse() * c_sel * B_inv * qfrc_all;
        VectorXn_t qfrc_compensation_forces = c_sel.transpose() * Lambda;

        // Apply control and compensation forces
        for (int i = 0; i < m->nv; i++) {
            mjtNum *act = d->act + m->actuator_actadr[i];			
            mjtNum *act_dot = d->act_dot + m->actuator_actadr[i];			

            act_dot[0] = d->ctrl[i];  					                                    // ctrl first integrator
            act_dot[1] = act[0];  					                                        // ctrl second integrator

            d->qfrc_applied[i] = qfrc_compensation_forces[i] - qfrc_damp[i] + qfrc_ctrl[i]; // + constraints - bias (addded by mujoco) // qforces
        }
    }

    // ddq will be calculated by MuJuCo after applying the input
    return -1;  // ignored by caller in the case of nact>1
}

//************************
// main function
int main(int argc, const char** argv)
{

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(xmlpath, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 2.000000};
    double arr_view[] = {-154, -20, 3.5, 0.0, 0.0, 1.};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    // Install control callback
    mjcb_control = mycontroller;

    // Global functions
    mjcb_act_dyn = custom_dynamics;
    mjcb_act_gain = custom_gain;
    mjcb_act_bias = custom_bias;

    // set initial position from xml model file
    mju_copy(d->qpos, m->key_qpos, m->nq); 
    // read time from keyframe (assume that it corresponds to end-time)
    mjtNum simend = *m->key_time;
    
    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

        if (d->time>=simend)
        {
           break;
         }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
