/*
Compile with:
g++ -O1 -g -Wall -o DRASER_comm DRASER_comm.cpp -I/usr/include/opencv4 -I/usr/include/eigen3 -lstdc++ -lpthread -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc -l:dsl.so -std=gnu++17
g++ -std=gnu++17 -O1 -g -Wall -o  DRASER_comm DRASER_comm.cpp -I/usr/include/eigen3 -I/usr/include/opencv4  -lstdc++ -lpthread -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc -l:dsl.so
*/

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
//#include<opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstdio>
#include "dsl.h"
//#include<cuda_runtime.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <string>

#include <eigen3/Eigen/Dense>

//#include <eigen3/Eigen/Eigen> //include of the entire Eigen library

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sstream>
#include <vector>
#include <fcntl.h>
#include <thread>
//#include <sys/mman.h>
//#include "gpionano.h"

// for signal handling
#include <signal.h>
#include <JetsonGPIO.h>

#include <numeric>

//ZK
#include <cmath>

#include <random>


//------------------PROMJENE
/*
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif
*/

using namespace std;
using namespace std::chrono;
using namespace cv;
//using namespace Eigen;

// ---- Vec3 ----
struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
    Vec3() = default;
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator-() const               { return {-x, -y, -z}; }
    Vec3 operator*(double s) const       { return {x*s, y*s, z*s}; }
    friend Vec3 operator*(double s, const Vec3& v) { return v * s; }
};

// ---- inline vector helpers ----
inline double dot(const Vec3& a, const Vec3& b)   { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3   cross(const Vec3& a, const Vec3& b) { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
inline double norm(const Vec3& v)                  { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) {
    double n = norm(v);
    if (n < 1e-12) throw std::runtime_error("Vektor ima gotovo nultu normu.");
    return {v.x/n, v.y/n, v.z/n};
}

// ---- Mat3 (kept here for parity with the standalone file) ----
struct Mat3 {
    Vec3 c0, c1, c2;
    Mat3() = default;
    Mat3(const Vec3& a, const Vec3& b, const Vec3& c) : c0(a), c1(b), c2(c) {}
    double at(int i, int j) const {
        const Vec3& c = (j == 0) ? c0 : (j == 1) ? c1 : c2;
        return (i == 0) ? c.x : (i == 1) ? c.y : c.z;
    }
};


struct EllipseFit3D {
    Eigen::Vector3d center        = Eigen::Vector3d::Zero();
    double          a             = 0.0;   // poluvelika os
    double          b             = 0.0;   // polumala os
    Eigen::Vector3d axis_major    = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis_minor    = Eigen::Vector3d::Zero();
    Eigen::Vector3d plane_normal  = Eigen::Vector3d::Zero();
    double          rms_off_plane  = 0.0;
    double          rms_to_ellipse = 0.0;
    int             n_points       = 0;
    bool            valid          = false;
};

//----------------------------------------------Function declaration
std::array<double, 6UL> find_weld(Mat depth_profile, Mat x_mm, std::array<double, 6UL> robot_pose);
std::array<double, 6UL> find_weld_cijev(Mat depth_profile, Mat x_mm, std::array<double, 6UL> robot_pose);

//std::array<double, 6UL> fit_elipse(Mat y_mm_depth, Mat x_mm, std::array<double, 6UL> robot_pose, double pipe_radius = 158);
//std::array<double, 9UL> kineta_face(Mat y_mm_depth, Mat x_mm, std::array<double, 6UL> robot_pose, double pipe_radius = 158, int option = 1);
std::array<double, 6UL> find_midpoint(Mat depth_profile, std::array<double, 6UL> robot_pose);
//std::array<double, 6UL> find_ribs(Mat y_mm_depth, Mat x_mm, std::array<double, 6UL> robot_pose);
Eigen::Matrix4d Fanuc_tool_2_mat(std::array<double, 6UL> tool_pose);
std::array<double, 6UL> mat_2_euler(Eigen::Matrix4d input_mat);
Eigen::Matrix4d euler_2_mat(std::array<double, 6UL> input);
void  orthonormalize_basis(Vec3& u, Vec3& v);
void  rotation_matrix_to_fanuc_wpr(const Mat3& R, double& W_deg, double& P_deg, double& R_deg);



std::vector<std::array<double, 6>> generate_fanuc_buffer_from_ellipse(
    const Vec3& C, double a, double b,
    const Vec3& u, const Vec3& v, const Vec3& n_plane_unit,
    double start_deg, double end_deg, double step_deg);

std::vector<std::array<double, 6>> fanuc_buffer_from_fit(
    const EllipseFit3D& ef,
    double start_deg,
    double end_deg,
    double step_deg);

void shutdown_seq();



template <class Vector3d>
std::pair<Vector3d, Vector3d> best_line_from_points(const std::vector<Vector3d> &c)
{
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix<Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic> centers(num_atoms, 3);
    for (size_t i = 0; i < num_atoms; ++i)
        centers.row(i) = c[i];

    Vector3d origin = centers.colwise().mean();
    Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
    Eigen::MatrixXd cov = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    Vector3d axis = eig.eigenvectors().col(2).normalized();

    return std::make_pair(origin, axis);
}



//----------------------------------------------Global variables
bool vision_bool = false;
bool cam_capture = false; //true;
bool bool_imshow = false;
bool cam_test = false;
float center_y = 0;
int keycode;
int slika = 0;
int Z;
int k=1;
int l;

bool first_write = true;
bool first_write_korigirani_online = true;
bool flag_nema_zavara = false;
double last_x;

// ── weld detection data structures ─────────────PROMJENE─────────────
// ── data structures ─────────────────────────────────────────
struct WeldLine   { double a = 0, b = 0; }; // y = a*x + b
struct WeldPoint  { double x = 0, y = 0; };

struct algorithm2_result {
    WeldLine leftSlope,  rightSlope;
    WeldLine leftHoriz,  rightHoriz;
    double   leftSlopeAngle  = 0, rightSlopeAngle  = 0;
    double   leftHorizAngle  = 0, rightHorizAngle  = 0;
    int apexCol = 0, apexRow = 0;

    WeldPoint leftToe,  rightToe;
    bool      leftToeValid  = false, rightToeValid  = false;
    WeldPoint centreOld;
    bool      centreOldValid = false;
    WeldPoint centreNew;
    bool      centreNewValid = false;

    double avgBaselineK = 0, avgBaselineB = 0;
    double weldWidthPx  = 0;
    int averageX = 0, maxY = 0, minY = 0, dubina = 0;
};

struct algorithm1_result {
    int minY    = 0;   // donji rub welda (max y u ROI)
    int maxY    = 0;   // korijen zavara (min y u ROI)
    int dubina  = 0;   // minY - maxY
    int averageX = 0;  // x-centroid u korijenu zavara
};

// ── Config ──────────────────────────────────────────────────
struct algorithm2_parameters {
    double residualThreshold = 2.0;   // RANSAC inlier distance (px)
    double flatFraction      = 0.25;  // fraction of profile for baseline
    double flankThreshold    = 0.10;  // fraction of amplitude for flank
    int    smoothWindow      = 11;    // Savitzky-Golay window (odd)
    int    guardPx           = 2;     // exclusion zone around apex (px)
    int    thresholdVal      = 10;    // brightness threshold
};



EllipseFit3D fit_ellipse_3d(
    const std::vector<std::array<double, 6>>& points,
    const Eigen::Vector3d& u_ref = Eigen::Vector3d(0.0, 0.3875, -0.9219),
    const Eigen::Vector3d& v_ref = Eigen::Vector3d(1.0, 0.0, 0.0));
// ─────────────────────────────────────────────────────────────
                

Point cam_center(948, 288);



float rcv_data[7];
float send_data[6];
std::vector<std::array<double, 6>> position_buffer;
std::vector<std::array<double, 1>> scanner_buffer;

std::vector<std::array<double, 6>> korigirani_buffer;
std::vector<std::array<double, 6>> milling_buffer;

std::vector<std::array<double, 6>> minimalne_tocke; //ZK
std::vector<std::array<double, 6>> FANUC_buffer;

Mat glob_profile, glob_xmm, glob_ymm;
vector<Mat> vec_profile, vec_xmm, vec_ymm;

//Eigen::Vector4D aaaaa;
Eigen::Matrix4d T_F_L = Eigen::Matrix4d::Identity(); // calibrated tool transformation

std::array<double, 6> Fanuc_tool = {279.9, 1.9, 155.2, -90, 0, 0}; //ZYX --> Fanuc XYZ

//-----------------------------------------------
//for the calibration exposure was exposuretimerange=\"600000 600000, this was increased to remove errors with 1000-400 kineta base
std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc exposuretimerange=\"750000 750000\" gainrange=\"1.2 1.2\" ispdigitalgainrange=\"2 2\" ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)GRAY8 ! videoconvert ! video/x-raw, format=(string)GRAY8 ! appsink max-buffers=1 drop=True";
}

//calib od 05_2022 - origigi pram
/*
std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc exposuretimerange=\"600000 600000\" gainrange=\"1 1\" ispdigitalgainrange=\"2 2\" ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)GRAY8 ! videoconvert ! video/x-raw, format=(string)GRAY8 ! appsink max-buffers=1 drop=True";
}*/

int capture_width = 1920;
int capture_height = 1080;
int display_width = 1920;
int display_height = 1080;
int framerate = 66;
int flip_method = 0;

string pipeline = gstreamer_pipeline(capture_width,
                                     capture_height,
                                     display_width,
                                     display_height,
                                     framerate,
                                     flip_method);
//cout << "Using pipeline: \n\t" << pipeline << "\n";

VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

//-----laser global----------------------------------------------------------------------------------
// Pin Definitions
int output_pin = 18; // BOARD pin 12, BCM pin 18
int curr_value = GPIO::HIGH;


double perpendicular_distance(const std::array<double, 6>& pt,const std::array<double, 6>& start,const std::array<double, 6>& end)
{
double dx = end[0] - start[0];
double dy = end[1] - start[1];
double dz = end[2] - start[2];
double line_len = std::sqrt(dx * dx + dy * dy + dz * dz);

if (line_len == 0.0)
{
double ex = pt[0] - start[0];
double ey = pt[1] - start[1];
double ez = pt[2] - start[2];
return std::sqrt(ex * ex + ey * ey + ez * ez);
}

double t = ((pt[0] - start[0]) * dx +
(pt[1] - start[1]) * dy +
(pt[2] - start[2]) * dz) / (line_len * line_len);

t = std::max(0.0, std::min(1.0, t));

double proj_x = start[0] + t * dx;
double proj_y = start[1] + t * dy;
double proj_z = start[2] + t * dz;

double ex = pt[0] - proj_x;
double ey = pt[1] - proj_y;
double ez = pt[2] - proj_z;

return std::sqrt(ex * ex + ey * ey + ez * ez);
}

void rdp_array(const std::vector<std::array<double, 6>>& points,int start,int end,double epsilon,std::vector<bool>& keep)
{
if (end - start < 2)
return;

double dmax = 0.0;
int index = start;

for (int i = start + 1; i < end; ++i)
{
double d = perpendicular_distance(points[i], points[start], points[end]);

if (d > dmax)
{
dmax = d;
index = i;
}
}

if (dmax > epsilon)
{
keep[index] = true;
rdp_array(points, start, index, epsilon, keep);
rdp_array(points, index, end, epsilon, keep);
}
}



//------socket communication--------------------------------------------------------------------------------------
class MyRobot
{
public:
    MyRobot()
    {
        for (int i = 0; i < 6; i++)
        {
            cmd[i] = 0;
            pos[i] = 0;
        }
    }

    virtual ~MyRobot()
    {
    }

    void error(const char *msg)
    {
        perror(msg);
        exit(1);
    }
    
    
    void read_comm()
    {

        n = ::read(newsockfd, buffer, 255);
        if (n == 0)
        {
            cout << "Klijent odspojen???" << endl;
            return;
        }
        string poruka(buffer, n);
        stringstream ss(poruka.c_str());
        cout << poruka;
        // parsiranje
        vector<string> result;
        double x1, x2, x3, x4, x5, x6, x7;

        int vector_size = 0;

        while (ss.good())
        {
            string substr;
            getline(ss, substr, '\''); // uzmi prvi string odvojen zarezom i staviti u substr
            result.push_back(substr);
            vector_size++;
        }
        // n=vector_size;
        if (vector_size >= 6)
        {
            stringstream ss_x1(result.at(1).c_str());
            stringstream ss_x2(result.at(3).c_str());
            stringstream ss_x3(result.at(5).c_str());
            stringstream ss_x4(result.at(7).c_str());
            stringstream ss_x5(result.at(9).c_str());
            stringstream ss_x6(result.at(11).c_str());
            stringstream ss_x7(result.at(13).c_str());

            ss_x1 >> x1;    //konverziju stringstream → double
            ss_x2 >> x2;
            ss_x3 >> x3;
            ss_x4 >> x4;
            ss_x5 >> x5;
            ss_x6 >> x6;
            ss_x7 >> x7;
        }
        else
        {
            x1 = 0;
            x2 = 0;
            x3 = 0;
            x4 = 0;
            x5 = 0;
            x6 = 0;
            x7 = 0;
        }

        pos[0] = x1;
        pos[1] = x2;
        pos[2] = x3;
        pos[3] = x4;
        pos[4] = x5;
        pos[5] = x6;
        pos[6] = x7;

        rcv_data[0] = pos[0];
        rcv_data[1] = pos[1];
        rcv_data[2] = pos[2];
        rcv_data[3] = pos[3];
        rcv_data[4] = pos[4];
        rcv_data[5] = pos[5];
        rcv_data[6] = pos[6];
    }
    

    void write_comm()
    {

        cmd[0] = send_data[0];
        cmd[1] = send_data[1];
        cmd[2] = send_data[2];
        cmd[3] = send_data[3];
        cmd[4] = send_data[4];
        cmd[5] = send_data[5];

        stringstream sss_x1, sss_x2, sss_x3, sss_x4, sss_x5, sss_x6;
        sss_x1 << cmd[0];
        sss_x2 << cmd[1];
        sss_x3 << cmd[2];
        sss_x4 << cmd[3];
        sss_x5 << cmd[4];
        sss_x6 << cmd[5];

        string sx1 = sss_x1.str();
        string sx2 = sss_x2.str();
        string sx3 = sss_x3.str();
        string sx4 = sss_x4.str();
        string sx5 = sss_x5.str();
        string sx6 = sss_x6.str();

        string ss_send = "'" + sx1 + "''" + sx2 + "''" + sx3 + "''" + sx4 + "''" + sx5 + "''" + sx6 + "'";

        sendbuf = ss_send.c_str();

        n = ::write(newsockfd, sendbuf, (int)strlen(sendbuf));
        if (n == 0)
        {
            cout << "Klijent odspojen???" << endl;
            return;
        }
        cout << "Poslana poruka: " << ss_send << endl;
        return;
    }

    void slusanje_novog_klijenta()
    {
        if (newsockfd > 0)
            return;

        ::listen(sockfd, 1);
        clilen = sizeof(cli_addr);

        newsockfd = ::accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);    // funkcija accept() blokira dok ne primi dolaznu vezu, a nakon uspostavljanja veze vraća novi socket file descriptor (newsockfd) koji se koristi za komunikaciju s klijentom, dok originalni sockfd ostaje otvoren i sluša za nove veze 
        if (newsockfd < 0)
        {
            //printf("Nema socket-a ... "); //ispisuje se ako nema dolaznih veza 
            return;
        }

        bzero(buffer, 256);

        printf("Novi klijent %d \n", newsockfd);
    }

    void comm_setup()
    {
        //kreiranje socket-a
        sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0)
            printf("ERROR pri otvaranju socket-a");
        fcntl(sockfd, F_SETFL, O_NONBLOCK); //postavljanje socket-a u non-blocking mod, tako da se funkcija accept neće blokirati ako nema dolaznih veza, već će se odmah vratiti s greškom EWOULDBLOCK ili EAGAIN, što omogućava programu da nastavi s izvršavanjem i redovito provjerava ima li novih veza bez zaustavljanja na čekanju 

        bzero((char *)&serv_addr, sizeof(serv_addr));
        portno = atoi("3142");
        serv_addr.sin_family = AF_INET;         //koristi se IPv4 protokol
        serv_addr.sin_addr.s_addr = INADDR_ANY; //server prihvaća veze na svim mrežnim sučeljima računala
        serv_addr.sin_port = htons(portno);     //port (u ovom slučaju 3142) pretvara se u mrežni format
        if (::bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
            printf("ERROR prilikom bindanja socket-a");
    }

    void CLoseSocket()
    {
        close(sockfd);
        sockfd = 0;
        newsockfd = 0;
        portno = 0;
        clilen = 0;
    }

    int dohvat_n()
    {
        return n;
    }

    void ispis_read()
    {
        std::cout << "Trenutna pozicija:"
                  << "  " << pos[0] << " " << pos[1] << "  " << pos[2] << "  " << pos[3] << " " << pos[4] << "  " << pos[5]
                  << "  " << pos[6] << std::endl;
    }


private:
    double cmd[6];
    double pos[7];

public:
    int sockfd, newsockfd, portno;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;
    const char *sendbuf;
};

//------thread function-----------------------------------------------------------------------------

struct thread_data
{
    int thread_id;
    bool scrun;
    string message;
};

void *RobotThread(void *threadarg)
{
    struct thread_data *my_data;
    my_data = (struct thread_data *)threadarg;
    cout << "Thread ID : " << my_data->thread_id;
    cout << " Message : " << my_data->message << endl;
    uint i_nb = 0;

    MyRobot robot;

    robot.comm_setup(); // pokretanje servera
    printf("\n Pokretanje veze ...\n");

    try
    {
        while (my_data->scrun)
        {
            // cout<<my_data->x<<", "<<my_data->y<<endl;
        ponovi_slusanje:
            if (robot.newsockfd < 1)
            {
                //printf("\nCekanje za nadolazecu vezu ...\n");
                robot.slusanje_novog_klijenta();
            }
            else
            {
                // Collect current position of ROBOT;
                robot.read_comm();
                if (robot.dohvat_n() == 0)
                {
                    printf("Klijent odspojen\n");
                    // robot.CLoseSocket();
                    robot.newsockfd = 0;
                    goto ponovi_slusanje;
                    // break; //--> prekine nit
                }
                else
                {
                    robot.ispis_read();
                }

                if (rcv_data[0] == 55555) // basic comm alive check
                {
                    //std::ofstream csv_file_positions("position_data.csv");
                    l=1;
                    if (!cam_capture)
                    {
                        cam_capture = true;
                        usleep(100000);
                    } //cam and laser on
                    // ackn to robot
                    int i;
                    for (i = 0; i < 6; i++)
                    {
                        send_data[i] = 55555;
                    }
                    
                    Z=0;
                    first_write = true;
                    first_write_korigirani_online = true;
                    flag_nema_zavara = false;
                    last_x = 1.0;

                    i_nb=0;//zk dodao 21.3.2026. ne znam zasto, ali bez ovog se javljaju problemi sa slanjem korekcija nakon ponovnog pokretanja trajektorije, jer se i_nb ne resetira na 0 
                    milling_buffer.clear();
                    position_buffer.clear();
                    scanner_buffer.clear();
                    korigirani_buffer.clear();
                    vec_xmm.clear();
                    vec_ymm.clear();
                    vec_profile.clear();

                    robot.write_comm();
                }
                else if (rcv_data[0] == 33333) // cam and laser on
                {
                    if (!cam_capture)
                    {
                        cam_capture = true;
                        usleep(100000);
                    } //cam and laser on
                    // ackn to robot
                    int i;
                    for (i = 0; i < 6; i++)
                    {
                        send_data[i] = 33333;
                    }

                    robot.write_comm();
                }
                else if (rcv_data[0] == 44444) // basic comm alive check with turn laser off
                {
                    if(cam_capture){cam_capture = false;} //cam and laser off
                    usleep(500000);

                    // ackn to robot
                    int i;
                    for (i = 0; i < 6; i++)
                    {
                        send_data[i] = 44444;
                    }

                    milling_buffer.clear();
                    position_buffer.clear();
                    scanner_buffer.clear();
                    korigirani_buffer.clear();
                    vec_xmm.clear();
                    vec_ymm.clear();
                    vec_profile.clear();

                    robot.write_comm();


                
                }
                else if (rcv_data[0] == 11111) // basic get save data request from robot
                {
                    position_buffer.push_back({{rcv_data[1], rcv_data[2], rcv_data[3], rcv_data[4], rcv_data[5], rcv_data[6]}});

                    center_y = 0;
                    //captured_new_cam =false;
                    while (center_y == 0 /*!captured_new_cam*/)
                    {
                        usleep(100000);
                        // dodati error checking ili sl. da ovo ne čeka beskonačno ako laser ništa ne vidi
                    }

                    vec_profile.push_back(glob_profile);
                    vec_xmm.push_back(glob_xmm);
                    vec_ymm.push_back(glob_ymm);

                    
                    

                    std::ofstream csv_file_positions;

                    if (first_write)
                    {
                        csv_file_positions.open("position_data.csv", std::ios::trunc); // prvi put briši
                        first_write = false;
                    }
                    else
                    {
                        csv_file_positions.open("position_data.csv", std::ios::app); // dalje dodaj
                    }

                    if (csv_file_positions.is_open())
                    {
                        const auto& row = position_buffer.back();

                        for (size_t i = 0; i < row.size(); i++)
                        {
                            csv_file_positions << row[i];
                            if (i < row.size() - 1) csv_file_positions << ",";
                        }
                        csv_file_positions << "\n";
                    }

                    Z=Z+1;
                    cout << "Z = " << Z << endl;
                    
                    // ackn to robot
                    int i;
                    for (i = 0; i < 6; i++)
                    {
                        send_data[i] = 11111;
                    }

                    robot.write_comm();

                    //cam_capture = false;
                }
                else if (rcv_data[0] == 21000) //request correction of the fillet weld poses
                {
                    std::ofstream csv_file_korigirani_offline("korigirani_data.csv");
                    std::ofstream csv_file_korigirani_minimalne("minimalne_data.csv");
                    //if (position_buffer.size() > =1) bilo za PROJEKT ZK dodal ali trebat će biti ovako >1 jer se ovaj hend shake poziva nakon kretnje robota, a ne nakon svake pojedinačne pozicije, tako da se u bufferu očekuje više pozicija koje se onda obrađuju u petlji, a ne samo jedna pozicija

                    if (position_buffer.size() > 1)

                    {
                        int i = 0;
                        while (i < position_buffer.size())
                        {
                            korigirani_buffer.push_back(find_weld(vec_ymm.at(i), vec_xmm.at(i), position_buffer.at(i))); //find weld and save corrected pose
                            i++;
                        }
                        if (csv_file_korigirani_offline.is_open())
                        {
                            // header (opcionalno)
                            //csv_file_korigirani_offline << "X,Y,Z,W,P,R\n";
                            cout << "Upisujem u csv!" << endl;

                            for (const auto& row : korigirani_buffer)
                            {
                                csv_file_korigirani_offline   << row[0] << ","
                                                << row[1] << ","
                                                << row[2] << ","
                                                << row[3] << ","
                                                << row[4] << ","
                                                << row[5] << "\n";
                            }

                            csv_file_korigirani_offline.close();
                        }
                        else
                        {
                            cout << "Ne mogu otvoriti CSV file za korigirane pozicije!" << endl;
                        }

                    }
                   //ZK
                    //cout << "Buffer size "<< position_buffer.size() << endl;
                    //korigirani_buffer.push_back({{rcv_data[1], rcv_data[2], rcv_data[3] - center_y, rcv_data[4], rcv_data[5], rcv_data[6]}});

                    const double epsilon = rcv_data[6];
                    //const double epsilon = 0.9;
                    cout << "Epsilon za RDP: " << epsilon << endl;
            
                    if (korigirani_buffer.size() > 1)
                    {
                        std::vector<bool> keep(korigirani_buffer.size(), false);
                        keep.front() = true;
                        keep.back() = true;
                
                        rdp_array(korigirani_buffer, 0,
                                static_cast<int>(korigirani_buffer.size()) - 1,
                                epsilon, keep);
                
                        minimalne_tocke.clear();
                
                        for (size_t i = 0; i < korigirani_buffer.size(); ++i)
                        {
                            if (keep[i])
                            {
                                minimalne_tocke.push_back(korigirani_buffer[i]);
                            }
                        }
                        if (csv_file_korigirani_minimalne.is_open())
                        {
                            // header (opcionalno)
                            //csv_file_korigirani_minimalne << "X,Y,Z,W,P,R\n";
                            cout << "Upisujem u csv!" << endl;

                            for (const auto& row : minimalne_tocke)
                            {
                                csv_file_korigirani_minimalne   << row[0] << ","
                                                << row[1] << ","
                                                << row[2] << ","
                                                << row[3] << ","
                                                << row[4] << ","
                                                << row[5] << "\n";
                            }

                            csv_file_korigirani_minimalne.close();
                        }
                        else
                        {
                            cout << "Ne mogu otvoriti CSV file za minimalne pozicije!" << endl;
                        }
                
                        std::cout << "Ulaznih tocaka: " << korigirani_buffer.size() << std::endl;
                        std::cout << "Minimalnih tocaka: " << minimalne_tocke.size() << std::endl;
                    }


                    int i;
                    for (i = 0; i < 6; i++)
                    {
                        send_data[i] = 21000;
                    }

                    robot.write_comm();
                }

                //ZK-offline cijev
                else if (rcv_data[0] == 21200) //request correction of the fillet weld poses
                {
                    std::ofstream csv_file_korigirani_offline("korigirani_data.csv");
                    //if (position_buffer.size() > =1) bilo za PROJEKT ZK dodal ali trebat će biti ovako >1 jer se ovaj hend shake poziva nakon kretnje robota, a ne nakon svake pojedinačne pozicije, tako da se u bufferu očekuje više pozicija koje se onda obrađuju u petlji, a ne samo jedna pozicija

                    if (position_buffer.size() > 1)

                    {
                        int i = 0;
                        while (i < position_buffer.size())
                        {
                            korigirani_buffer.push_back(find_weld_cijev(vec_ymm.at(i), vec_xmm.at(i), position_buffer.at(i))); //find weld and save corrected pose
                            i++;
                        }
                        if (csv_file_korigirani_offline.is_open())
                        {
                            // header (opcionalno)
                            //csv_file_korigirani_offline << "X,Y,Z,W,P,R\n";
                            cout << "Upisujem u csv!" << endl;

                            for (const auto& row : korigirani_buffer)
                            {
                                csv_file_korigirani_offline   << row[0] << ","
                                                << row[1] << ","
                                                << row[2] << ","
                                                << row[3] << ","
                                                << row[4] << ","
                                                << row[5] << "\n";
                            }

                            csv_file_korigirani_offline.close();
                        }
                        else
                        {
                            cout << "Ne mogu otvoriti CSV file za korigirane pozicije!" << endl;
                        }
                        // 3) Fit ellipse + generate FANUC trajectory
                        EllipseFit3D ef = fit_ellipse_3d(korigirani_buffer);
                        if (ef.valid)
                        {
                            cout << "\n=== Fit elipse 3D ===" << endl;
                            cout << "Broj tocaka:    " << ef.n_points << endl;
                            cout << "Centar:         (" << ef.center(0) << ", " << ef.center(1) << ", " << ef.center(2) << ")" << endl;
                            cout << "Poluvelika a:   " << ef.a << endl;
                            cout << "Polumala b:     " << ef.b << endl;
                            cout << "Ekscentricitet: " << std::sqrt(1.0 - (ef.b/ef.a)*(ef.b/ef.a)) << endl;
                            cout << "Velika os:      (" << ef.axis_major(0) << ", " << ef.axis_major(1) << ", " << ef.axis_major(2) << ")" << endl;
                            cout << "Mala os:        (" << ef.axis_minor(0) << ", " << ef.axis_minor(1) << ", " << ef.axis_minor(2) << ")" << endl;
                            cout << "Normala ravnine:(" << ef.plane_normal(0) << ", " << ef.plane_normal(1) << ", " << ef.plane_normal(2) << ")" << endl;
                            cout << "RMS van ravnine: " << ef.rms_off_plane << endl;
                            cout << "RMS do elipse:   " << ef.rms_to_ellipse << endl;

                            // Save fit parameters to CSV
                            std::ofstream csv_ellipse("ellipse_fit.csv");
                            if (csv_ellipse.is_open()) {
                                csv_ellipse << "Cx,Cy,Cz,a,b,axMx,axMy,axMz,axmx,axmy,axmz,nx,ny,nz\n";
                                csv_ellipse << ef.center(0) << "," << ef.center(1) << "," << ef.center(2) << ","
                                            << ef.a << "," << ef.b << ","
                                            << ef.axis_major(0) << "," << ef.axis_major(1) << "," << ef.axis_major(2) << ","
                                            << ef.axis_minor(0) << "," << ef.axis_minor(1) << "," << ef.axis_minor(2) << ","
                                            << ef.plane_normal(0) << "," << ef.plane_normal(1) << "," << ef.plane_normal(2) << "\n";
                                csv_ellipse.close();
                            }

                            // Generate FANUC trajectory along the fitted ellipse  (ONCE)
                            const double start_deg = 265.0;
                            const double end_deg   = 95.0;
                            const double step_deg  = -(start_deg - end_deg) / 100;

                            try {
                                FANUC_buffer = fanuc_buffer_from_fit(ef, start_deg, end_deg, step_deg);
                            } catch (const std::exception& e) {
                                cout << "[FANUC] Greska: " << e.what() << endl;
                            }
                            cout << "Generirano " << FANUC_buffer.size() << " XYZWPR tocaka." << endl;

                            // Save trajectory CSVs (ONCE)
                            std::ofstream f1("ellipse_xyzwpr.csv", std::ios::trunc);
                            std::ofstream f2("ellipse_xyzwpr_FANUC.csv", std::ios::trunc);
                            if (f1.is_open() && f2.is_open()) {
                                f1 << "deg,X,Y,Z,W,P,R\n";
                                for (size_t i = 0; i < FANUC_buffer.size(); ++i) {
                                    double t_deg = start_deg + i * step_deg;
                                    const auto& r = FANUC_buffer[i];
                                    f1 << t_deg << "," << r[0] << "," << r[1] << "," << r[2] << ","
                                                    << r[3] << "," << r[4] << "," << r[5] << "\n";
                                    f2 << r[0] << "," << r[1] << "," << r[2] << ","
                                    << r[3] << "," << r[4] << "," << r[5] << "\n";
                                }
                                f1.close(); f2.close();
                                cout << "Spremljeno: ellipse_xyzwpr.csv, ellipse_xyzwpr_FANUC.csv" << endl;
                            }
                        }
                        else
                        {
                            cout << "Fit elipse nije moguc (premalo tocaka ili lose raspodjeljene tocke)." << endl;
                        }

                    }
                                   
                    int i;
                    for (i = 0; i < 6; i++)
                    {
                        send_data[i] = 21200;
                    }

                    robot.write_comm();
                }

                //ZK-online
                else if (rcv_data[0] == 21100) //request correction of the fillet weld poses
                {
                    if (position_buffer.size() == 1)

                    {
                        int i = 0;
                        while (i < position_buffer.size())
                        {
                            korigirani_buffer.clear();
                            korigirani_buffer.push_back(find_weld(vec_ymm.at(i), vec_xmm.at(i), position_buffer.at(i)));
                            i++;

                    }
                    // Write latest position_buffer entry to CSV
                   
                    std::ofstream csv_file_korigirani_online;

                    if (first_write_korigirani_online)
                    {
                        csv_file_korigirani_online.open("korigirani_data.csv", std::ios::trunc); // prvi put briši
                        first_write_korigirani_online = false;
                    }
                    else
                    {
                        csv_file_korigirani_online.open("korigirani_data.csv", std::ios::app); // dalje dodaj
                    }

                    if (csv_file_korigirani_online.is_open())
                        {
                            const auto& row = korigirani_buffer.back(); // always the latest (only) row
                            for (size_t i = 0; i < row.size(); i++)
                            {
                                csv_file_korigirani_online << row[i];
                                if (i < row.size() - 1) csv_file_korigirani_online << ",";
                            }
                            csv_file_korigirani_online << "\n";
                        }
                        else
                        {
                            cout << "Ne mogu otvoriti CSV file za korigirane pozicije!" << endl;
                        }


                    int z;
                    for (z = 0; z < 6; z++)
                    {
                        send_data[z] = 21100;
                    }

                    robot.write_comm();
                    }
                }
                //ZK
                else if (rcv_data[0] == 40000) // request reduction of corrected poses
                {
                    const double epsilon = 0.8;
                
                    if (korigirani_buffer.size() > 1)
                    {
                        std::vector<bool> keep(korigirani_buffer.size(), false);
                        keep.front() = true;
                        keep.back() = true;
                
                        rdp_array(korigirani_buffer, 0,
                                static_cast<int>(korigirani_buffer.size()) - 1,
                                epsilon, keep);
                
                        minimalne_tocke.clear();
                
                        for (size_t i = 0; i < korigirani_buffer.size(); ++i)
                        {
                            if (keep[i])
                            {
                                minimalne_tocke.push_back(korigirani_buffer[i]);
                            }
                        }
                
                        std::cout << "Ulaznih tocaka: " << korigirani_buffer.size() << std::endl;
                        std::cout << "Minimalnih tocaka: " << minimalne_tocke.size() << std::endl;
                    }
                
                    for (int i = 0; i < 6; i++)
                    {
                        send_data[i] = 40000;
                    }
                
                    robot.write_comm();
                }

                else if (rcv_data[0] == 24000) //request testing socket communication
                {
                    usleep(100000); // 100 ms
                    
                    int i;
                    for (i = 0; i < 6; i++)
                    {
                        send_data[i] = 24000;
                    }
                    robot.write_comm();
                }

                else if (rcv_data[0] == 31000 ) //send corrected data and clear() global variables when done
                {                                                                                                      // pracenje konture

                    /*// OBRADI PODATKE IZ position buffer-a   --- saljemo apsolutno sve tocke iz korigirani buffer-a, bez obzira na to koliko ih 
                    for (uint ii = 0; ii < korigirani_buffer.size(); ii++)
                    {
                        cout << "korigirani br. " << ii << " = " << korigirani_buffer.at(ii)[0] << "," << korigirani_buffer.at(ii)[1] << "," << korigirani_buffer.at(ii)[2] << "," << korigirani_buffer.at(ii)[3] << "," << korigirani_buffer.at(ii)[4] << "," << korigirani_buffer.at(ii)[5] << endl;
                    }

                    if (i_nb < korigirani_buffer.size())// jer se korigigirani buffer puni s push_back-om, a ne s indeksiranjem, tako da se ne zna unaprijed koliko će elemenata imati, zato se koristi veličina vektora kao uvjet u petlji 
                    {
                        //ovo tu je samo za projekt  jer korekciju radim odmah nakon sto napravim inkrement kretnje
                        //int slanje = korigirani_buffer.size()-1;
                        //send_data[0] = korigirani_buffer[slanje][0];
                        //send_data[1] = korigirani_buffer[slanje][1];
                        //send_data[2] = korigirani_buffer[slanje][2];
                        //send_data[3] = korigirani_buffer[slanje][3];
                        //send_data[4] = korigirani_buffer[slanje][4];
                       // send_data[5] = korigirani_buffer[slanje][5];

                        send_data[0] = korigirani_buffer[i_nb][0];
                        send_data[1] = korigirani_buffer[i_nb][1];
                        send_data[2] = korigirani_buffer[i_nb][2];
                        send_data[3] = korigirani_buffer[i_nb][3];
                        send_data[4] = korigirani_buffer[i_nb][4];
                        send_data[5] = korigirani_buffer[i_nb][5];

                        robot.write_comm();

                        i_nb++;
                    }*/
                    //ZK-- šaljemo minimalne točke umjesto svih korigiranih točaka, jer se očekuje da robot može raditi s manjim brojem točaka koje su međusobno udaljenije
                    /*
                    for (uint ii = 0; ii < minimalne_tocke.size(); ii++)
                    {
                        cout << "minimalne br. " << ii << " = "
                            << minimalne_tocke.at(ii)[0] << ","
                            << minimalne_tocke.at(ii)[1] << ","
                            << minimalne_tocke.at(ii)[2] << ","
                            << minimalne_tocke.at(ii)[3] << ","
                            << minimalne_tocke.at(ii)[4] << ","
                            << minimalne_tocke.at(ii)[5] << endl;
                    }*/
                    if (i_nb < minimalne_tocke.size())
                    {
                        send_data[0] = minimalne_tocke[i_nb][0];
                        send_data[1] = minimalne_tocke[i_nb][1];
                        send_data[2] = minimalne_tocke[i_nb][2];
                        send_data[3] = minimalne_tocke[i_nb][3];
                        send_data[4] = minimalne_tocke[i_nb][4];
                        send_data[5] = minimalne_tocke[i_nb][5];

                        robot.write_comm();

                        i_nb++;
                    }




                    else
                    {
                        if (rcv_data[0] != 34000)
                        {
                            cam_capture = false;
                        } //laser is turned off by 44444 after communication 34000
                        int i;
                        for (i = 0; i < 6; i++)
                        {
                            send_data[i] = 99999;
                        }

                        robot.write_comm();

                        cout << "kraj trajektorije" << endl;

                        i_nb = 0;

                        cout << "position_buffer size =" << position_buffer.size() << endl;
                        cout << "korigirani_buffer size =" << korigirani_buffer.size() << endl;
                        std::cout << "Minimalnih tocaka: " << minimalne_tocke.size() << std::endl;

                        position_buffer.clear();
                        scanner_buffer.clear();
                        korigirani_buffer.clear();
                        vec_xmm.clear();
                        vec_ymm.clear();
                        vec_profile.clear();
                    }
                }

                //ZK-offline cijev
                else if (rcv_data[0] == 31200 )
                {                                                                                                      


                    if (i_nb < FANUC_buffer.size()) 
                    {
                        
                        send_data[0] = FANUC_buffer[i_nb][0];
                        send_data[1] = FANUC_buffer[i_nb][1];
                        send_data[2] = FANUC_buffer[i_nb][2];
                        send_data[3] = FANUC_buffer[i_nb][3];
                        send_data[4] = FANUC_buffer[i_nb][4];
                        send_data[5] = FANUC_buffer[i_nb][5];

                        robot.write_comm();

                        i_nb++;
                    }
                    
                    else
                    {
                        if (rcv_data[0] != 34000)
                        {
                            cam_capture = false;
                        } //laser is turned off by 44444 after communication 34000
                        int i;
                        for (i = 0; i < 6; i++)
                        {
                            send_data[i] = 99999;
                        }

                        robot.write_comm();

                        cout << "kraj trajektorije" << endl;

                        i_nb = 0;

                        cout << "position_buffer size =" << position_buffer.size() << endl;
                        cout << "korigirani_buffer size =" << korigirani_buffer.size() << endl;
                        cout << "FANUC_buffer size =" << FANUC_buffer.size() << endl;

                        position_buffer.clear();
                        scanner_buffer.clear();
                        korigirani_buffer.clear();
                        FANUC_buffer.clear();
                        vec_xmm.clear();
                        vec_ymm.clear();
                        vec_profile.clear();
                    }
                }

                //zk-online
                else if (rcv_data[0] == 31100) //send corrected data and clear() global variables when done
                {    
                    i_nb=0;                                                                                                 // pracenje konture
                    if (i_nb < korigirani_buffer.size())
                        {
                        send_data[0] = korigirani_buffer[0][0];
                        send_data[1] = korigirani_buffer[0][1];
                        send_data[2] = korigirani_buffer[0][2];
                        send_data[3] = korigirani_buffer[0][3];
                        send_data[4] = korigirani_buffer[0][4];
                        send_data[5] = korigirani_buffer[0][5];
                        
                        //ZK-online proba
                        cout << "position_buffer size =" << position_buffer.size() << endl;
                        cout << "korigirani_buffer size =" << korigirani_buffer.size() << endl;
                        position_buffer.clear();
                        korigirani_buffer.clear();
                        vec_xmm.clear();
                        vec_ymm.clear();
                        vec_profile.clear();

                        robot.write_comm();
                        i_nb++;
                        }
                    else
                        {
                            /*if (rcv_data[0] != 34000)
                            {
                                cam_capture = false;
                            } //laser is turned off by 44444 after communication 34000
                            int i;
                            for (i = 0; i < 6; i++)
                            {
                                send_data[i] = 99999;
                            }

                            robot.write_comm();

                            cout << "kraj trajektorije" << endl;*/

                            i_nb = 0;

                            cout << "position_buffer size =" << position_buffer.size() << endl;
                            cout << "korigirani_buffer size =" << korigirani_buffer.size() << endl;
                            position_buffer.clear();
                            korigirani_buffer.clear();
                            vec_xmm.clear();
                            vec_ymm.clear();
                            vec_profile.clear();
                        }
                }
                

                if (robot.dohvat_n() == 0)
                {
                    printf("Klijent odspojen\n");
                    // robot.CLoseSocket();
                    robot.newsockfd = 0;
                    goto ponovi_slusanje;
                    // break; //--> prekine nit
                }

            } // else

            //this_thread::sleep_for(milliseconds(250));//original Josip
            this_thread::sleep_for(milliseconds(50));

            if (keycode == 27)
                break;
        }

    } //try
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        std::cout << "Error occured --- closing app" << endl;
        curr_value = GPIO::LOW;
        cout << "Outputting (end)" << curr_value << " to pin ";
        cout << output_pin << endl;
        GPIO::output(output_pin, curr_value);
        GPIO::cleanup();

        // Stop the thread
        //  td.scrun=0; //close sc thread
        // Clean and close camera
        DSLCLOSE();
        cap.release();
        cv::destroyAllWindows();

        robot.CLoseSocket();
        cout << "Socket communication thread is closing!" << endl;
        pthread_exit(NULL);

        return 0;
    }

    robot.CLoseSocket();
    cout << "Socket communication thread is closing!" << endl;
    pthread_exit(NULL);
}
//-----------------------------------------------------------------------------------------------

int vision_system()
{
    //-----laser on----------------------------------------------------------------------------------
    // Pin Definitions
    //int output_pin = 18; // BOARD pin 12, BCM pin 18
    GPIO::setmode(GPIO::BCM);
    // set pin as an output pin with optional initial state of HIGH
    GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);
    curr_value = GPIO::HIGH;
    //---------------------------


    namedWindow("Output");
    cout << "Using pipeline: \n\t" << pipeline << "\n";

    if (!cap.isOpened())
    {
        cout << "Failed to open camera." << std::endl;
        shutdown_seq();
        return (-1);
    }

    const int rows = display_height;
    const int cols = display_width;

    Mat input, inputA;
    bool preklop = false;

    // Create output image
    Mat output(rows, cols, CV_8UC1);
    // Mat profil(rows,1,CV_32S);
    Mat profil = Mat::zeros(cols, 1, CV_32S);   // r,c
    Mat ieprofil = Mat::zeros(cols, 1, CV_32S); // r,c
    Mat xmm = Mat::zeros(cols, 1, CV_64F);
    Mat ymm = Mat::zeros(cols, 1, CV_64F);

    int i, poz, saveprofil = 0, ifn = 0;
    //int keycode;
    string fn, info;
    //   Image center is get through calibration
    //Point p1(0, 287), p2(cols - 1, 287);
    //Point p3(955, 0), p4(955, rows - 1);
    Point p1(0, cam_center.y), p2(cols - 1, cam_center.y);
    Point p3(cam_center.x, 0), p4(cam_center.x, rows - 1);

    

    /*   float time, fps;
   auto start = high_resolution_clock::now();
   auto end = high_resolution_clock::now();
   auto duration = duration_cast<milliseconds>(end - start);
*/

    READCALIBRATION("Calibration.dat");
    DSLINIT(rows, cols, 1);

    try
    {

        // 2) ZAJEM IN OBDELAVA
        while (vision_bool == true)
        {
            
            if (cam_test && !cam_capture)
            {
                if (!cap.read(input) || input.empty())
                {
                    cout << "Camtest - Capture read error or image empty!" << endl;
                    
                    break;
                }
                cout << "Camtest - ok" << endl;
                cam_test=false;
            }
            else if (!cam_capture)
            {
                //Laser off while not capturing images
                curr_value = GPIO::LOW;
                GPIO::output(output_pin, curr_value);
                //destroyAllWindows();
                usleep(1000);
            }
            else
            {
                if (curr_value == GPIO::LOW)
                {
                    curr_value = GPIO::HIGH;
                    GPIO::output(output_pin, curr_value);
                }

                if (!cap.read(input) || input.empty())
                {
                    cout << "Capture read error or image empty!" << endl;
                    
                    break;
                }

                cvtColor(input, output, COLOR_GRAY2BGR); //this was added so everything would work with color

                rectangle(input, Rect(Point2i(0, 550), Point2i(cols - 1, rows - 1)), cv::Scalar(0), -1, LINE_8, 0);        //horizontal
                rectangle(input, Rect(Point2i(0, 0), Point2i(cols - 1, 80)), cv::Scalar(0), -1, LINE_8, 0);                //horizontal
                rectangle(input, Rect(Point2i(0, 0), Point2i(100, rows - 1)), cv::Scalar(0), -1, LINE_8, 0);               //vertical
                rectangle(input, Rect(Point2i(cols - 100, 0), Point2i(cols - 1, rows - 1)), cv::Scalar(0), -1, LINE_8, 0); //vertical

                //imshow("Original cam image", input);
                DSLLOAD(input); //gpu
                //DSLDELTA(input); //gpu
                DSLGETPROFIL(profil);
                //***HERE YOU CAN COPY PROFILE IN PIXELS TO SOME DEDICATED MEMORY

                //crtanje profila na sliku
                for (i = 0; i < cols; i++)
                {
                    poz = profil.at<int>(i);
                    //output.at<unsigned char>(poz,i) = 255-output.at<unsigned char>(poz,i);
                    input.at<unsigned char>(poz, i) = 255 - output.at<unsigned char>(poz, i);
                    //Vec3b &color = input.at<Vec3b>(poz, i);
                    //if (color[2] < 5)
                    //    profil.at<int>(i) = 0;
                }

                //***CLEAN PROFILE
                SELECTBLOBS(profil, ieprofil, cols, 5, 10); // cpu
                //***APPLY CALIBRATION
                UMERITEV(ieprofil, xmm, ymm, cols); // cpu
                                                    //***xmm, ymm are im [mm] BITNO

                //if (ymm.at<double>(cam_center.x, 0) == 0)
                //{
                // //center_y=0;
                // double bb = 0;
                // double avg_center = 0;
                // for (int i = -5; i < 6; i++)
                // {
                //   if (ymm.at<double>((cols / 2 + i), 0) != 0)
                //   {
                //     avg_center += ymm.at<double>((cols / 2 + i), 0);
                //     bb++;
                //   }
                // }
                // center_y = float(avg_center / bb); //average of nonzero center values
                //}
                //else
                //{
                //save measurement values to glob variables
                center_y = float(ymm.at<double>(cam_center.x, 0));
                if (center_y == 0)
                {
                    for (int i = 0; i < 12; i++)
                    {
                        //cout<<"For i ="<<i<<endl;
                        if (float(ymm.at<double>(cam_center.x + i, 0)) != 0)
                        {
                            center_y = float(ymm.at<double>(cam_center.x + 1, 0));
                            cout << "Found depth = " << center_y << endl;
                            break;
                        }
                        else if (float(ymm.at<double>(cam_center.x - i, 0)) != 0)
                        {
                            center_y = float(ymm.at<double>(cam_center.x - i, 0));
                            cout << "Found depth = " << center_y << endl;
                            break;
                        }
                    }
                }

                glob_profile = ieprofil.clone();
                glob_xmm = xmm.clone();
                glob_ymm = ymm.clone();
                //captured_new_cam=true;
                //}

                if (bool_imshow == true)
                {
                    //izris profila na sliko
                    for (i = 0; i < cols; i++)
                    {
                        poz = ieprofil.at<int>(i);
                        output.at<Vec3b>(poz, i) = Vec3b(0, 0, 255);
                    }
                    info = std::to_string(ymm.at<double>(cam_center.x, 0));
                    cv::putText(output, info, Point(p3.x + 20, p1.y + 20), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);

                    //coord 100,0
                    info = std::to_string(ymm.at<double>(1364, 0));
                    cv::putText(output, info, Point(1364 + 20, p1.y + 20), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
                    //cross y-line
                    cv::line(output, Point (1364,p1.y-5),  Point (1364,p1.y+5), Scalar(255, 255, 255), 1, LINE_4);

                    //coord 100+x,0
                    info = std::to_string(ymm.at<double>(1564, 0));
                    cv::putText(output, info, Point(1564 + 20, p1.y + 20), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
                    //cross y-line
                    cv::line(output, Point (1564,p1.y-5),  Point (1564,p1.y+5), Scalar(255, 255, 255), 1, LINE_4);

                    //coord -100,0
                    info = std::to_string(ymm.at<double>(534, 0));
                    cv::putText(output, info, Point(534 + 20, p1.y + 20), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
                    //cross y-line
                    cv::line(output, Point (534,p1.y-5),  Point (534,p1.y+5), Scalar(255, 255, 255), 1, LINE_4);

                   //coord -100-x,0
                    info = std::to_string(ymm.at<double>(334, 0));
                    cv::putText(output, info, Point(334 + 20, p1.y + 20), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
                    //cross y-line
                    cv::line(output, Point (334,p1.y-5),  Point (334,p1.y+5), Scalar(255, 255, 255), 1, LINE_4);


                    //cross x-line
                    cv::line(output, p1, p2, Scalar(255, 255, 255), 1, LINE_4);
                    //cross y-line
                    cv::line(output, p3, p4, Scalar(255, 255, 255), 1, LINE_4);
                    //        imshow("Output",output);
                    //work area
                    cv::line(output, Point(0, 80), Point(output.cols, 80), Scalar(0, 255, 0), 1, LINE_4);
                    //work area
                    cv::line(output, Point(0, 500), Point(output.cols, 500), Scalar(0, 255, 0), 1, LINE_4);
                    
                    
                    //ZK
                    //work area
                    cv::line(output, Point(0, 500), Point(output.cols, 500), Scalar(0, 255, 0), 1, LINE_4);

                    // Draw weld ROI rectangle 
                    int x = 948;
                    int y = 288;
                    int frame_size = 300; // search frame size from find_weld
                    //int frame_size = 100; // search frame size from find_weld_cijev
                    cv::Rect weld_rect(x - frame_size / 2, y - frame_size / 2, frame_size, frame_size);
                    cv::rectangle(output, weld_rect, Scalar(0, 0, 255), 2);

                    imshow("Output", output);
                }
                
            } // if cam_capture==true

            

            keycode = waitKey(20); // & 0xff ;
            if (keycode == 27)
                break;
            if (keycode == 104)
            {
                cout << "p - Save profil" << endl
                     << "r - Reset file counter" << endl
                     << "ESC - exit" << endl;
                ifn = 0;
            };
            if (keycode == 112)
            {
                cout << "p" << endl;
                std::ostringstream filename;
                filename << "output_screen" << slika << ".jpg";
                cv::imwrite(filename.str(), output);
                slika ++;


                ifn++;
                saveprofil = 1;
            };
            if (keycode == 114)
            {
                cout << "r - Reset camcapture and imshow" << endl;
                cam_capture = !cam_capture;
                ifn = 0;
                // td.scrun=0; //close sc thread
            };
            if (keycode == 105) // i
            {
                bool_imshow = !bool_imshow;
                cout << "i - show camera input = " << bool_imshow << endl;
                

            };

            input.release();
        } // while
    }
    catch (const std::exception &e)
    {
        std::cout << "Error occured --- closing app" << endl;
        curr_value = GPIO::LOW;
        cout << "Outputting (end)" << curr_value << " to pin ";
        cout << output_pin << endl;
        GPIO::output(output_pin, curr_value);
        GPIO::cleanup();

        // Stop the thread
        //  td.scrun=0; //close sc thread
        // Clean and close camera
        DSLCLOSE();
        cap.release();
        cv::destroyAllWindows();
        system("sudo systemctl restart nvargus-daemon.service"); //this was added to ensure proper restart of the function
        keycode=27;
        return 0;
    }

    cout<<"Vision while loop exited."<<endl;

    //***THIS SHOULD HAPPEN*** DO NOT CLOSE THE PROGRAM WITH CTRL-C***
    //----------------new laser off------
    /*curr_value = GPIO::LOW;
    cout << "Outputting (end)" << curr_value << " to pin ";
    cout << output_pin << endl;
    GPIO::output(output_pin, curr_value);
    GPIO::cleanup();*/

    // Stop the thread
    //  td.scrun=0; //close sc thread
    // Clean and close camera

    DSLCLOSE();
    cap.release();
    cv::destroyAllWindows();
    system("sudo systemctl restart nvargus-daemon.service"); //this was added to ensure proper restart of the function

    // pthread_exit(NULL); //close thread
    keycode=27;
    return 0;
}

int main()
{
    
    //T_F_L = Fanuc_tool_2_mat(Fanuc_tool);
    //cout << T_F_L;
    sleep(1);

    //-----start socket communication thread --------------------------------------------------------
    int rc;
    pthread_t threadsc;
    thread_data td;
    cout << "Creating socket communication thread ... " << endl;
    td.thread_id = 1;
    td.message = "Socket communication thread is running!";
    td.scrun = 1;
    rc = pthread_create(&threadsc, NULL, RobotThread, (void *)&td);

    if (rc)
    {
        cout << "Error:unable to create thread," << rc << endl;
        exit(-1);
    }

    //-----start camera -----------------------------------------------------------------------------
    cout << "Starting camera capture..." << endl;
    vision_bool = true;
    //cam_capture = true;
    thread th1(vision_system);
    // vision_system();
    cout << "Vision thread actived..." << endl;
    int brojac = 0;

    while (true /*rcv_data[0] != 999*/)
    {

        brojac++;
        cout << "Main thread counter =" << brojac << endl;
        if (!cam_capture) {cout<<"Inititate camera capture test"<<endl; cam_test=true;};
        usleep(50000000);
        if (brojac == 9999999)
        {
            brojac = 0;
        }

        if (keycode == 27) break;

    }

    cout << "Initiating vision shutdown..." << endl;
    vision_bool = false;

    // Wait for th1 to finish cleanup
    th1.join();
    cout << "Ending all thread..." << endl;

    return 0;
}

//-------------------------------------------PROMJENE
// ── 1. extract profile (column-wise weighted centroid) ──────
static void wdet_extractProfile(const cv::Mat& binary,
    std::vector<int>& cols,
    std::vector<int>& rows)
{
cols.clear(); rows.clear();
for (int x = 0; x < binary.cols; ++x) {
double colMax = 0;
for (int y = 0; y < binary.rows; ++y)
colMax = std::max(colMax, (double)binary.at<uchar>(y, x));
if (colMax < 10) continue;

double thresh = colMax * 0.3;
double wSum = 0, wRow = 0;
for (int y = 0; y < binary.rows; ++y) {
double v = binary.at<uchar>(y, x);
if (v >= thresh) { wRow += v * y; wSum += v; }
}
if (wSum == 0) continue;
cols.push_back(x);
rows.push_back((int)std::round(wRow / wSum));
}
}

// ── 2. linear regression ────────────────────────────────────
static WeldLine wdet_linreg(const std::vector<double>& x,
const std::vector<double>& y)
{
double n = x.size(), sx=0, sy=0, sxx=0, sxy=0;
for (size_t i=0;i<x.size();++i){sx+=x[i];sy+=y[i];sxx+=x[i]*x[i];sxy+=x[i]*y[i];}
double d = n*sxx - sx*sx;
if (std::abs(d) < 1e-12) return {0.0, sy/n};
WeldLine l; l.a=(n*sxy-sx*sy)/d; l.b=(sy-l.a*sx)/n; return l;
}

// ── 3. detrend ──────────────────────────────────────────────
static std::vector<double> wdet_detrend(const std::vector<int>& cols,
            const std::vector<int>& rows,
            double flatFrac)
{
int n=(int)cols.size(), k=std::max(4,(int)(n*flatFrac));
std::vector<double> bx,by;
for(int i=0;i<k;++i){bx.push_back(cols[i]);by.push_back(rows[i]);}
for(int i=n-k;i<n;++i){bx.push_back(cols[i]);by.push_back(rows[i]);}
WeldLine bl = wdet_linreg(bx,by);
std::vector<double> dt(n);
for(int i=0;i<n;++i) dt[i]=rows[i]-(bl.a*cols[i]+bl.b);
return dt;
}

// ── 4. Savitzky-Golay smooth ─────────────────────────────────
static std::vector<double> wdet_savgol(const std::vector<double>& y, int win)
{
if(win%2==0) win++;
if(win<3) win=3;
int m=win/2, n=(int)y.size();
int deg=std::min(3,win-1);
std::vector<double> out(n);
for(int i=0;i<n;++i){
int lo=std::max(0,i-m), hi=std::min(n-1,i+m), wl=hi-lo+1;
int D=std::min(deg,wl-1)+1;
std::vector<std::vector<double>> A(wl,std::vector<double>(D,1.0));
for(int j=0;j<wl;++j){double xj=(lo+j)-i;for(int d=1;d<D;++d)A[j][d]=A[j][d-1]*xj;}
std::vector<double> AtA(D*D,0),Aty(D,0);
for(int j=0;j<wl;++j)for(int r=0;r<D;++r){Aty[r]+=A[j][r]*y[lo+j];for(int c=0;c<D;++c)AtA[r*D+c]+=A[j][r]*A[j][c];}
std::vector<std::vector<double>> M(D,std::vector<double>(D+1));
for(int r=0;r<D;++r){for(int c=0;c<D;++c)M[r][c]=AtA[r*D+c];M[r][D]=Aty[r];}
for(int col=0;col<D;++col){
int pv=col; for(int r=col+1;r<D;++r)if(std::abs(M[r][col])>std::abs(M[pv][col]))pv=r;
std::swap(M[col],M[pv]);
if(std::abs(M[col][col])<1e-12)continue;
for(int r=0;r<D;++r){if(r==col)continue;double f=M[r][col]/M[col][col];for(int c=col;c<=D;++c)M[r][c]-=f*M[col][c];}
}
out[i]=(std::abs(M[0][0])>1e-12)?M[0][D]/M[0][0]:y[i];
}
return out;
}

// ── 5. find peaks with prominence ────────────────────────────
struct WdetPeak { int idx; double prom; };
static std::vector<WdetPeak> wdet_findPeaks(const std::vector<double>& y,
                 double minProm=0.3)
{
std::vector<WdetPeak> pk;
int n=(int)y.size();
for(int i=1;i<n-1;++i){
if(y[i]>y[i-1]&&y[i]>y[i+1]){
double lb=y[i]; for(int j=i-1;j>=0;--j){lb=std::min(lb,y[j]);if(y[j]>y[i])break;}
double rb=y[i]; for(int j=i+1;j<n;++j){rb=std::min(rb,y[j]);if(y[j]>y[i])break;}
double p=y[i]-std::max(lb,rb);
if(p>=minProm) pk.push_back({i,p});
}
}
return pk;
}

// ── 6. find apex ─────────────────────────────────────────────
static int wdet_findApex(const std::vector<double>& dt,
std::vector<double>& smooth, int win)
{
int n=(int)dt.size();
win=std::min(win|1,(n/4)*2+1); if(win<3)win=3;
smooth=wdet_savgol(dt,win);
auto peaks=wdet_findPeaks(smooth,0.3);
std::vector<double> neg(smooth.size()); for(size_t i=0;i<smooth.size();++i)neg[i]=-smooth[i];
auto vals=wdet_findPeaks(neg,0.3);
int best=-1; double bscore=0;
for(auto& p:peaks) if(p.prom>bscore){bscore=p.prom;best=p.idx;}
for(auto& v:vals)  if(v.prom>bscore){bscore=v.prom;best=v.idx;}
if(best==-1){
auto itMn=std::min_element(smooth.begin(),smooth.end());
auto itMx=std::max_element(smooth.begin(),smooth.end());
best=(int)(std::abs(*itMn)>=std::abs(*itMx)
?std::distance(smooth.begin(),itMn)
:std::distance(smooth.begin(),itMx));
}
return best;
}

// ── 7. flank segments ────────────────────────────────────────
static void wdet_flanks(const std::vector<double>& sm,
int apexIdx, int guard, double thr,
std::vector<bool>& lm, std::vector<bool>& rm)
{
int n=(int)sm.size();
double amp=std::abs(sm[apexIdx]);
if(amp<0.5){double mx=0;for(auto v:sm)mx=std::max(mx,std::abs(v));amp=std::max(mx,1.0);}
double t=thr*amp;
int le=std::max(0,apexIdx-guard);
for(int i=apexIdx-guard;i>=0;--i){if(std::abs(sm[i])<t){le=i;break;}}
int rs=std::min(n-1,apexIdx+guard);
for(int i=apexIdx+guard;i<n;++i){if(std::abs(sm[i])<t){rs=i;break;}}
lm.assign(n,false); rm.assign(n,false);
for(int i=le;i<apexIdx-guard;++i)lm[i]=true;
for(int i=apexIdx+guard+1;i<=rs;++i)rm[i]=true;
int lc=0,rc=0; for(bool b:lm)lc+=b; for(bool b:rm)rc+=b;
if(lc<4)for(int i=0;i<apexIdx-guard;++i)lm[i]=true;
if(rc<4)for(int i=apexIdx+guard+1;i<n;++i)rm[i]=true;
}

// ── 8. RANSAC line fit ───────────────────────────────────────
static WeldLine wdet_ransac(const std::vector<double>& x,
 const std::vector<double>& y,
 double rThr,
 std::vector<bool>& inliers,
 int maxT=2000)
{
int n=(int)x.size();
if(n<4){inliers.assign(n,true);return wdet_linreg(x,y);}
int ms=std::max(4,(int)(0.2*n));
std::mt19937 rng(42);
WeldLine best; int bc=0; inliers.assign(n,false);
for(int t=0;t<maxT;++t){
std::vector<int> idx(n); std::iota(idx.begin(),idx.end(),0);
std::shuffle(idx.begin(),idx.end(),rng); idx.resize(ms);
std::vector<double> sx,sy;
for(int i:idx){sx.push_back(x[i]);sy.push_back(y[i]);}
WeldLine c=wdet_linreg(sx,sy);
int cnt=0; for(int i=0;i<n;++i)if(std::abs(y[i]-(c.a*x[i]+c.b))<=rThr)++cnt;
if(cnt>bc){bc=cnt;best=c;
std::vector<double> ix,iy;
for(int i=0;i<n;++i)if(std::abs(y[i]-(best.a*x[i]+best.b))<=rThr){ix.push_back(x[i]);iy.push_back(y[i]);}
if((int)ix.size()>=ms)best=wdet_linreg(ix,iy);
}
}
for(int i=0;i<n;++i)inliers[i]=std::abs(y[i]-(best.a*x[i]+best.b))<=rThr;
return best;
}

static WeldLine wdet_fitMasked(const std::vector<int>& cols,
    const std::vector<int>& rows,
    const std::vector<bool>& mask,
    double rThr, std::vector<bool>& inliers)
{
std::vector<double> x,y;
for(size_t i=0;i<mask.size();++i)if(mask[i]){x.push_back(cols[i]);y.push_back(rows[i]);}
return wdet_ransac(x,y,rThr,inliers);
}

static double wdet_angle(double a){return std::atan(a)*180.0/M_PI;}

static bool wdet_intersect(const WeldLine& l1, const WeldLine& l2,
    WeldPoint& out)
{
double d = l1.a - l2.a;
if(std::abs(d) < 1e-12) return false;
out.x = (l2.b - l1.b) / d;
out.y = l1.a * out.x + l1.b;
return true;
}   

// ── 9. compute averageX (exact C++ replica) ──────────────────
static void wdet_averageX(const cv::Mat& binary, int thr,
int& avgX, int& maxY, int& minY)
{
maxY=binary.rows; minY=-1;
for(int i=0;i<binary.rows;++i)
for(int j=0;j<binary.cols;++j)
if((int)binary.at<uchar>(i,j)>thr){
if(i<maxY)maxY=i;
if(i>minY)minY=i;
}
std::vector<int> cx;
for(int i=0;i<binary.rows;++i)
for(int j=0;j<binary.cols;++j)
if((int)binary.at<uchar>(i,j)>thr && i<=maxY+1)
cx.push_back(j);
if(cx.empty()){avgX=0;return;}
double s=0; for(int v:cx)s+=v;
avgX=(int)std::round(s/cx.size());
}

// ── 10. MAIN DETECTION (accepts cv::Mat directly) ────────────
static algorithm2_result algorithm2_characteristic_point(const cv::Mat& weld_ROI,
                 const algorithm2_parameters& cfg = algorithm2_parameters{})
{
algorithm2_result res;

// profile
std::vector<int> cols, rows;
wdet_extractProfile(weld_ROI, cols, rows);
if((int)cols.size()<10){
std::cerr<<"[weld_det] Too few bright pixels in weld_ROI\n"; return res;}

// detrend
auto dt = wdet_detrend(cols, rows, cfg.flatFraction);

// apex
std::vector<double> smooth;
int apexIdx = wdet_findApex(dt, smooth, cfg.smoothWindow);
res.apexCol = cols[apexIdx];
res.apexRow = rows[apexIdx];

// flanks
std::vector<bool> lm, rm;
wdet_flanks(smooth, apexIdx, cfg.guardPx, cfg.flankThreshold, lm, rm);

// slope fits
std::vector<bool> lIn, rIn;
res.leftSlope  = wdet_fitMasked(cols, rows, lm, cfg.residualThreshold, lIn);
res.rightSlope = wdet_fitMasked(cols, rows, rm, cfg.residualThreshold, rIn);
res.leftSlopeAngle  = wdet_angle(res.leftSlope.a);
res.rightSlopeAngle = wdet_angle(res.rightSlope.a);

// horizontal baselines
int lfStart=res.apexCol, rfEnd=res.apexCol;
for(int i=0;i<(int)lm.size();++i)if(lm[i]){lfStart=cols[i];break;}
for(int i=(int)rm.size()-1;i>=0;--i)if(rm[i]){rfEnd=cols[i];break;}

std::vector<bool> lhm(cols.size(),false), rhm(cols.size(),false);
for(size_t i=0;i<cols.size();++i){
if(cols[i]<lfStart) lhm[i]=true;
if(cols[i]>rfEnd)   rhm[i]=true;
}
int lhc=0,rhc=0; for(bool b:lhm)lhc+=b; for(bool b:rhm)rhc+=b;
if(lhc<4)for(size_t i=0;i<cols.size();++i)lhm[i]=cols[i]<res.apexCol-cfg.guardPx;
if(rhc<4)for(size_t i=0;i<cols.size();++i)rhm[i]=cols[i]>res.apexCol+cfg.guardPx;

std::vector<bool> lhIn, rhIn;
res.leftHoriz  = wdet_fitMasked(cols, rows, lhm, cfg.residualThreshold, lhIn);
res.rightHoriz = wdet_fitMasked(cols, rows, rhm, cfg.residualThreshold, rhIn);
res.leftHorizAngle  = wdet_angle(res.leftHoriz.a);
res.rightHorizAngle = wdet_angle(res.rightHoriz.a);


// intersections (toe points)
res.leftToeValid  = wdet_intersect(res.leftSlope,  res.leftHoriz,  res.leftToe);
res.rightToeValid = wdet_intersect(res.rightSlope, res.rightHoriz, res.rightToe);
// old centre (midpoint of toes)
if(res.leftToeValid && res.rightToeValid){
    double mx = (res.leftToe.x + res.rightToe.x) / 2.0;
    double my = (res.leftToe.y + res.rightToe.y) / 2.0;
    res.centreOld.x   = mx; res.centreOld.y = my;
    res.centreOldValid = true;
    res.weldWidthPx   = std::abs(res.rightToe.x - res.leftToe.x);
    // new geometric centre
    double nx         = (mx + res.apexCol) / 2.0;
    res.avgBaselineK  = (res.leftHoriz.a + res.rightHoriz.a) / 2.0;
    res.avgBaselineB  = (res.leftHoriz.b + res.rightHoriz.b) / 2.0;
    res.centreNew.x   = nx;
    res.centreNew.y   = res.avgBaselineK * nx + res.avgBaselineB;
    res.centreNewValid = true;
}



// averageX / maxY / dubina  (exact C++ replica)
wdet_averageX(weld_ROI, cfg.thresholdVal,
res.averageX, res.maxY, res.minY);
res.dubina = res.minY - res.maxY;

/*
// print results
std::cout << "\n── WELD DETECTION RESULTS ──────────────────────────\n";
std::cout << "  Apex         : col=" << res.apexCol
<< "  row=" << res.apexRow << "\n";
std::cout << "  Left  horiz  : y = " << res.leftHoriz.a  << "*x + " << res.leftHoriz.b
<< "   angle=" << res.leftHorizAngle  << "deg\n";
std::cout << "  Left  slope  : y = " << res.leftSlope.a  << "*x + " << res.leftSlope.b
<< "   angle=" << res.leftSlopeAngle  << "deg\n";
std::cout << "  Right slope  : y = " << res.rightSlope.a << "*x + " << res.rightSlope.b
<< "   angle=" << res.rightSlopeAngle << "deg\n";
std::cout << "  Right horiz  : y = " << res.rightHoriz.a << "*x + " << res.rightHoriz.b
<< "   angle=" << res.rightHorizAngle << "deg\n";
if(res.leftToeValid)
    std::cout<<"  Left  toe    : x="<<res.leftToe.x<<"  y="<<res.leftToe.y<<"\n";
if(res.rightToeValid)
    std::cout<<"  Right toe    : x="<<res.rightToe.x<<"  y="<<res.rightToe.y<<"\n";
if(res.centreOldValid)
    std::cout<<"  Centre OLD   : x="<<res.centreOld.x<<"  y="<<res.centreOld.y<<"\n";
if(res.centreNewValid)
    std::cout<<"  Centre NEW   : x="<<res.centreNew.x<<"  y="<<res.centreNew.y<<"\n";
std::cout<<"  Weld width   : "<<res.weldWidthPx<<" px\n";
std::cout<<"  averageX     : "<<res.averageX
<<"  maxY="<<res.maxY
<<"  dubina="<<res.dubina<<" px\n";
std::cout<<"  Incl. angle  : "
<<std::abs(res.leftSlopeAngle-res.rightSlopeAngle)<<"deg\n";
std::cout<<"────────────────────────────────────────────────────\n\n";*/
return res;
}

static algorithm1_result algorithm1_characteristic_point(const cv::Mat& weld_ROI,
    int initialMaxY,
    int threshold = 10)
{
algorithm1_result res;
res.minY = 0;
res.maxY = initialMaxY;

// donji rub welda - najveca y koordinata
for (int i = 0; i < weld_ROI.rows; i++)
{
for (int j = 0; j < weld_ROI.cols; j++)
{
if ((int)weld_ROI.at<uchar>(i, j) > threshold && res.minY < i)
{
res.minY = i;
}
}
}

// gornji rub welda (korijen zavara) - najmanja y koordinata
for (int i = 0; i < weld_ROI.rows; i++)
{
for (int j = 0; j < weld_ROI.cols; j++)
{
if ((int)weld_ROI.at<uchar>(i, j) > threshold && res.maxY > i)
{
res.maxY = i;
}
}
}

res.dubina = res.minY - res.maxY;

// x-centroid u korijenu zavara (koristi i susjedni red iznad)
std::vector<int> centroidX;
for (int i = 0; i < weld_ROI.rows; i++)
{
for (int j = 0; j < weld_ROI.cols; j++)
{
if ((int)weld_ROI.at<uchar>(i, j) > threshold && res.maxY >= i - 1)
{
centroidX.push_back(j);
}
}
}

if (!centroidX.empty())
{
double sum = 0.0;
for (int v : centroidX) sum += v;
res.averageX = (int)std::round(sum / centroidX.size());
}
else
{
res.averageX = 0;
}

return res;
}


//-----PROMJENE---------------------------------------

//-----------------------------------------vision functions------------------------------
std::array<double, 6UL> find_weld(Mat depth_profile, Mat x_mm, std::array<double, 6UL> robot_pose)
{
    // u x_mm se nalaze x koordinate svakog piksela u mm udaljene od središnje linije vertikalne
    // Print the values of x_mm
    /*for (int i = 0; i < x_mm.rows; ++i) {
        printf("x_mm[%d] = %f\n", i, x_mm.at<double>(i, 0));
    }*/

    // input is Mat ymm row-vector of depth values from the laser and current robot pose
    // output is th corrected robot pose, where only positional values are changed based on the located weld position
    
    Mat result = Mat::zeros(1000, depth_profile.rows, CV_8UC1); 
    for (int i = 0; i < result.cols; i++)
    {
        // u depth_profile se nalazi udaljenost svakog pikesla od središnje linije u mm, a u result se crta profil, gdje je svaki piksel s vrijednosti 255 onaj koji je na dubini koja odgovara toj vrijednosti u depth_profile, a ostali su crni (0)
        //cout << depth_profile.at<double>(i, 0) << ", " << i << endl; 
        if (depth_profile.at<double>(i, 0) != 0) 
        {
            //cout << depth_profile.at<double>(i,0)<< ", " << i << endl;
            // cout << (int)round(depth_profile.at<uchar>(0,i)) + 500 << ", " << i << endl;
            result.at<uchar>((int)round(depth_profile.at<double>(i, 0)) + 288, i) = 255; 
        }
    }

    //int x = result.cols / 2;
    int x = 948;
    //ZK
    //int y = 293;  Point cam_center(948, 288);
    int y = 288;
    //ZK stavil na 150 a bilo je 200
    int frame_size = 300; // search frame size frame za trazenje zavara, odnosno dimenzija kvadrata unutar kojeg se trazi zavara, a koji je centriran u koordinati (x,y) koja predstavlja ocekivanu poziciju sredine welda
    cv::Rect weld_rect(x - frame_size / 2, y - frame_size / 2, frame_size, frame_size);
    Mat weld_ROI = result(weld_rect).clone(); //uzima iz result samo onaj dio koji je unutar pravokutnika weld_rect, a zatim se radi analiza samo nad tim dijelom slike 

    // ────────── birna analiza slike ────────────────────────────
    //algoritam1
    algorithm1_result alg1 = algorithm1_characteristic_point(weld_ROI, y);
    int   minY     = alg1.minY;
    int   maxY     = alg1.maxY;
    int   dubina   = alg1.dubina;
    float averageX = alg1.averageX;

    int newCentreX = averageX;
    double newCentreY = minY;

    // ───────────── geometrijska analiza slike ────────────────────
    //algoritam2
    //algorithm2_parameters alg2_cfg;
    //algorithm2_result alg2 = algorithm2_characteristic_point(weld_ROI, alg2_cfg);
    
    //int newCentreX = alg2.centreNewValid ? (int)alg2.centreNew.x : averageX;
    //double newCentreY = alg2.centreNewValid ? alg2.centreNew.y : maxY;
    // ─────────────────────────────────────────────────────────────
    cout << "New Centre X: " << newCentreX << endl;
    cout << "New Centre y: " << newCentreY << endl;
   

              
                //ZK  cranje 
                    // Create a new window for displaying the weld ROI
                    /*namedWindow("Weld ROI", WINDOW_NORMAL);
                    imshow("Weld ROI", weld_ROI);
                    waitKey(1);
                    // Create a new window for displaying the weld ROI
                    namedWindow("result", WINDOW_NORMAL);
                    imshow("result", result);
                        // Save the weld ROI image to the build directory
                        string weld_roi_filename = "weld_ROI"<<l<<".jpg";
                        if (!imwrite(weld_roi_filename, weld_ROI))
                        {
                            cerr << "Failed to save weld ROI image to " << weld_roi_filename << endl;
                        }
                        else
                        {
                            cout << "Weld ROI image saved to " << weld_roi_filename << endl;
                        }
                    waitKey(1);*/

                    /*
                    namedWindow("Weld ROI", WINDOW_NORMAL);
                    //imshow("Weld ROI", weld_ROI);
                    //waitKey(1);

                    // generiranje imena
                    string weld_roi_filename = "save_images/weld_ROI" + std::to_string(l++) + ".jpg";

                    // spremanje slike
                    if (!imwrite(weld_roi_filename, weld_ROI))
                    {
                        cerr << "Failed to save weld ROI image to " << weld_roi_filename << endl;
                    }
                    else
                    {
                        cout << "Weld ROI image saved to " << weld_roi_filename << endl;
                    }

                    waitKey(1);*/

    // cout << "Weld center coordinates" << maxY << "," << average << endl;
    // circle(weld_ROI, Point(round(average), maxY), 3, Scalar(255), 1, LINE_8);
    // circle(result, Point(round(average) + weld_rect.x, maxY + weld_rect.y), 3, Scalar(255), 1, LINE_8);
    // imshow("Weld center", weld_ROI);
    // imshow("Profile xy", result);
    // waitKey(0);
    // return int(round(average) + weld_rect.x);

    //test sa robotom X-Y-Z
    std::array<double, 6UL> robot_temp = robot_pose;
    robot_temp[3] = robot_pose[3] * (M_PI / 180);
    robot_temp[4] = robot_pose[4] * (M_PI / 180);
    robot_temp[5] = robot_pose[5] * (M_PI / 180);

    // position of the weld center is at the coordinate (averageX+weld_rect.x, maxY + weld_rect.y) of the laser system frame  ŠULIGOJ
    //--------------transform from laser frame to the robot world coordiante frame-------------------------------------------------
    //Eigen::Vector4d weld_cent(-(x_mm.at<double>(averageX + weld_rect.x, 0)), maxY + weld_rect.y - 500, 0, 1); //x coordinate is returned with reversed direction!
    
    //ZK---ispod MOJA PRVA IDEJA
    //Eigen::Vector4d weld_cent((x_mm.at<double>(averageX + weld_rect.x, 0)), -(maxY + weld_rect.y - 288), 0, 1); //x coordinate is returned with reversed direction!
    //cout << "Weld center coordinates " << (x_mm.at<double>(averageX + weld_rect.x, 0)) << endl;   //ZK


    Eigen::Vector4d weld_cent((x_mm.at<double>(newCentreX + weld_rect.x, 0)),-(newCentreY + weld_rect.y - 288),0, 1);

    Eigen::Vector4d corrected_xyz;
    Eigen::Matrix4d robot_pose_mat = Eigen::Matrix4d::Identity();
    //----------Transform x,y,z,eul1,eul2,eul3 --> to Matrix4D
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(robot_temp[5], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(robot_temp[4], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(robot_temp[3], Eigen::Vector3d::UnitX()); // angle1,angle2,angle3 and notation (z-y-x for Fanuc)
    // check if angles or radians are delivered from the robot!

    robot_pose_mat.block<3, 3>(0, 0) = R;
    robot_pose_mat.block<3, 1>(0, 3) = Eigen::Vector3d(robot_pose[0], robot_pose[1], robot_pose[2]);

    //cout << "robot_pose_mat:" << endl << robot_pose_mat << endl;          //ZK

    //corrected_xyz = robot_pose_mat * T_F_L * weld_cent; // TEST vector multiplication...both vectors should be column-vectors
    
    //algoritam2
    /*
    if (alg2.weldWidthPx>20 && alg2.weldWidthPx<110) //ako je sirina zavara veća od 5px i manja od 100px, onda se korigira pozicija robota 
    {
        corrected_xyz = robot_pose_mat * weld_cent; // poznat alat na Fanucu korigirane kordinate prema robotu
        flag_nema_zavara = true; 
        last_x = corrected_xyz.x();
        cout << "Širina: " <<alg2.weldWidthPx << endl;
    }
    else
    {
        if (flag_nema_zavara == false)
        {
            weld_cent = Eigen::Vector4d(1, 1, 0, 1); //ako je dubina zavara manja od 4mm, onda se ne korigira pozicija robota, nego se koristi trenutna pozicija
            corrected_xyz = robot_pose_mat * weld_cent;
        }
        else
        {
            weld_cent = Eigen::Vector4d(1, 1, 0, 1); //ako je dubina zavara manja od 4mm, onda se ne korigira pozicija robota, nego se koristi trenutna pozicija
            corrected_xyz = robot_pose_mat * weld_cent;
            corrected_xyz.x() = last_x; //ako nema zavara, onda se koristi zadnja poznata x koordinata
            cout << "Nema zavara, koristim zadnju poznatu x koordinatu: " << last_x << endl;
        }
    }*/
    
    //algoritam1
    corrected_xyz = robot_pose_mat * weld_cent;


    cout << "Dubina = " << dubina << endl;
    
    //cout << "Robot Pose Matrix:" << endl << robot_pose_mat << endl;       //ZK
    //cout << "Weld Center:" << endl << weld_cent.transpose() << endl;      //ZK

    robot_pose[0] = corrected_xyz.x();
    robot_pose[1] = corrected_xyz.y();
    robot_pose[2] = corrected_xyz.z();

    return robot_pose;
}



std::array<double, 6UL> find_weld_cijev(Mat depth_profile, Mat x_mm, std::array<double, 6UL> robot_pose)
{
    
    Mat result = Mat::zeros(1000, depth_profile.rows, CV_8UC1); 
    for (int i = 0; i < result.cols; i++)
    {
        if (depth_profile.at<double>(i, 0) != 0) 
        {
            result.at<uchar>((int)round(depth_profile.at<double>(i, 0)) + 288, i) = 255; 
        }
    }

    int x = 948;
    int y = 288;
    int frame_size = 100;
    cv::Rect weld_rect(x - frame_size / 2, y - frame_size / 2, frame_size, frame_size); //(948-150=798, 288-100=188, 300, 150)
    Mat weld_ROI = result(weld_rect).clone();

    
    // ────────── birna analiza slike ────────────────────────────
    //algoritam1
    algorithm1_result alg1 = algorithm1_characteristic_point(weld_ROI, y);
    int   minY     = alg1.minY;
    int   maxY     = alg1.maxY;
    int   dubina   = alg1.dubina;
    float averageX = alg1.averageX;

    // ───────────── geometrijska analiza slike ────────────────────
    //algoritam2
    algorithm2_parameters alg2_cfg;
    algorithm2_result alg2 = algorithm2_characteristic_point(weld_ROI, alg2_cfg);
    // ─────────────────────────────────────────────────────────────
    int newCentreX = alg2.centreNewValid ? alg2.apexCol : averageX;
    //double newCentreY = alg2.centreNewValid ? alg2.centreNew.y : maxY;
    int newCentreY =  maxY;
    cout << "New Centre x: " << alg2.apexCol << endl;
    cout << "New maxY: " << maxY << endl;
    cout << "New alg2.centreNew.y: " << alg2.centreNew.y << endl;

    //test sa robotom X-Y-Z
    std::array<double, 6UL> robot_temp = robot_pose;
    robot_temp[3] = robot_pose[3] * (M_PI / 180.0);
    robot_temp[4] = robot_pose[4] * (M_PI / 180.0);
    robot_temp[5] = robot_pose[5] * (M_PI / 180.0);

    // position of the weld center is at the coordinate (averageX+weld_rect.x, maxY + weld_rect.y) of the laser system frame  ŠULIGOJ
    //--------------transform from laser frame to the robot world coordiante frame-------------------------------------------------
    //Eigen::Vector4d weld_cent(-(x_mm.at<double>(averageX + weld_rect.x, 0)), maxY + weld_rect.y - 500, 0, 1); //x coordinate is returned with reversed direction!
    
    //ZK---ispod MOJA PRVA IDEJA
    //Eigen::Vector4d weld_cent((x_mm.at<double>(averageX + weld_rect.x, 0)), -(maxY + weld_rect.y - 288), 0, 1); //x coordinate is returned with reversed direction!
    //cout << "Weld center coordinates " << (x_mm.at<double>(averageX + weld_rect.x, 0)) << endl;   //ZK
   


    Eigen::Vector4d weld_cent((x_mm.at<double>(newCentreX + weld_rect.x, 0)),-(newCentreY + weld_rect.y - 288)-3,0, 1); //-2 je da nije direktno u korijenu zavara nego u zraku
    cout << "Weld center coordinates (x_mm): " << x_mm.at<double>(newCentreX + weld_rect.x, 0) << endl;   //ZK

    Eigen::Vector4d corrected_xyz;
    Eigen::Matrix4d robot_pose_mat = Eigen::Matrix4d::Identity();
    //----------Transform x,y,z,eul1,eul2,eul3 --> to Matrix4D
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(robot_temp[5], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(robot_temp[4], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(robot_temp[3], Eigen::Vector3d::UnitX()); // angle1,angle2,angle3 and notation (z-y-x for Fanuc)
    robot_pose_mat.block<3, 3>(0, 0) = R;
    robot_pose_mat.block<3, 1>(0, 3) = Eigen::Vector3d(robot_pose[0], robot_pose[1], robot_pose[2]);

    //cout << "Robot Pose Matrix:" << endl << robot_pose_mat << endl;       //ZK
    //cout << "Weld Center:" << endl << weld_cent.transpose() << endl;      //ZK

    corrected_xyz = robot_pose_mat * weld_cent;
    robot_pose[0] = corrected_xyz.x();
    robot_pose[1] = corrected_xyz.y();
    robot_pose[2] = corrected_xyz.z();
    
    //ovo ispod je Tool offset 
    /*robot_pose[0] = x_mm.at<double>(newCentreX + weld_rect.x, 0);
    robot_pose[1] = -(newCentreY + weld_rect.y - 288);
    robot_pose[2] = 0;
    robot_pose[3] = 0;
    robot_pose[4] = 0;
    robot_pose[5] = 0;*/

    return robot_pose;
}



//---------------------------------std::array<double, 6UL> fit_elipse(Mat y_mm_depth, Mat x_mm, std::array<double, 6UL> robot_pose, double pipe_radius)

//----------------------------------std::array<double, 6UL> find_ribs(Mat y_mm_depth, Mat x_mm, std::array<double, 6UL> robot_pose)

//----------------------------------std::array<double, 9UL> kineta_face(Mat y_mm_depth, Mat x_mm, std::array<double, 6UL> robot_pose, double pipe_radius, int option)


std::array<double, 6UL> find_midpoint(Mat depth_profile, std::array<double, 6UL> robot_pose)
{

    // input is Mat ymm row-vector of depth values from the laser and current robot pose
    // output is th corrected robot pose, where only positional values are changed based on the located weld position
    double center_depth_ymm = 0;
    //error checking if the point is visible
    for (int i = 0; i < 12; i++)
    {
        cout << "For i =" << i << endl;
        if (depth_profile.at<double>(cam_center.x + i, 0) != 0)
        {
            center_depth_ymm = depth_profile.at<double>(cam_center.x + i, 0);
            cout << "Found depth = " << center_depth_ymm << endl;
            break;
        }
        else if (depth_profile.at<double>(cam_center.x - i, 0) != 0)
        {
            center_depth_ymm = depth_profile.at<double>(cam_center.x - i, 0);
            cout << "Found depth = " << center_depth_ymm << endl;
            break;
        }
    }
    if (center_depth_ymm == 0)
    {
        return {66666, 66666, 66666, 66666, 66666, 66666};
    }

    Eigen::Matrix4d robot_pose_mat;
    robot_pose_mat = euler_2_mat(robot_pose);
    // position of the weld center is at the coordinate (averageX+weld_rect.x, maxY + weld_rect.y) of the laser system frame
    //--------------transform from laser frame to the robot world coordiante frame-------------------------------------------------
    Eigen::Vector4d cent(0, -1 * center_depth_ymm, 0, 1);
    Eigen::Vector4d corrected_xyz;

    //corrected_xyz = robot_pose_mat * T_F_L * cent; // TEST vector multiplication...both vectors should be column-vectors
    //new_robot_pose = corrected_xyz * inv(cent) * inv(T_F_L)
    corrected_xyz = robot_pose_mat * cent;
    robot_pose[0] = corrected_xyz.x();
    robot_pose[1] = corrected_xyz.y();
    robot_pose[2] = corrected_xyz.z();

    return robot_pose;
}

Eigen::Matrix4d Fanuc_tool_2_mat(std::array<double, 6UL> tool_pose)
{
    std::array<double, 6UL> &tool_temp = tool_pose;

    tool_pose[3] = tool_temp[3] * (M_PI / 180);
    tool_pose[4] = tool_temp[4] * (M_PI / 180);
    tool_pose[5] = tool_temp[5] * (M_PI / 180);

    Eigen::Matrix4d tool_pose_mat = Eigen::Matrix4d::Identity();
    //----------Transform x,y,z,eul1,eul2,eul3 --> to Matrix4D
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(tool_pose[3], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(tool_pose[4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(tool_pose[5], Eigen::Vector3d::UnitX()); // angle1,angle2,angle3 and notation (z-y-x for Fanuc)
    // check if angles or radians are delivered from the robot!

    tool_pose_mat.block<3, 3>(0, 0) = R;
    tool_pose_mat.block<3, 1>(0, 3) = Eigen::Vector3d(tool_pose[0], tool_pose[1], tool_pose[2]);

    return tool_pose_mat;
}

std::array<double, 6UL> mat_2_euler(Eigen::Matrix4d input_mat)
{
    //cout << "Mat: " << endl
    //<< input_mat << endl;
    Eigen::Vector3d ea = input_mat.block<3, 3>(0, 0).eulerAngles(2, 1, 0); //3x3 rotation to x-y-z Euler angles in radians
    //cout << "to Euler angles:" << endl;
    cout << ea << endl
         << endl;

    std::array<double, 6UL> output;

    output[0] = input_mat(0, 3);
    output[1] = input_mat(1, 3);
    output[2] = input_mat(2, 3);
    output[3] = ea(2) / M_PI * 180;
    output[4] = ea(1) / M_PI * 180;
    output[5] = ea(0) / M_PI * 180;

    return output;
}

Eigen::Matrix4d euler_2_mat(std::array<double, 6UL> input)
{
    std::array<double, 6UL> tool_temp = input;

    input[3] = tool_temp[3] * (M_PI / 180);
    input[4] = tool_temp[4] * (M_PI / 180);
    input[5] = tool_temp[5] * (M_PI / 180);

    Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
    //----------Transform x,y,z,eul1,eul2,eul3 --> to Matrix4D
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(input[5], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(input[4], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(input[3], Eigen::Vector3d::UnitX()); // angle1,angle2,angle3 and notation (x-z-y for Fanuc)
    // check if angles or radians are delivered from the robot!
    output.block<3, 3>(0, 0) = R;
    output.block<3, 1>(0, 3) = Eigen::Vector3d(input[0], input[1], input[2]);

    return output;
}



void shutdown_seq()
{
    std::cout << "Error occured --- closing app" << endl;
    curr_value = GPIO::LOW;
    cout << "Outputting (end)" << curr_value << " to pin ";
    cout << output_pin << endl;
    GPIO::output(output_pin, curr_value);
    GPIO::cleanup();

    DSLCLOSE();
    cap.release();
    cv::destroyAllWindows();

    pthread_exit(NULL);
}




// ---- fit_ellipse_3d ----
EllipseFit3D fit_ellipse_3d(
    const std::vector<std::array<double, 6>>& points,
    const Eigen::Vector3d& u_ref,    
    const Eigen::Vector3d& v_ref)    
{
    EllipseFit3D out;
    const int N = static_cast<int>(points.size());
    out.n_points = N;
    if (N < 5) {
        std::cout << "[fit_ellipse_3d] potrebno >= 5 tocaka, primljeno " << N << std::endl;
        return out;
    }

    Eigen::MatrixXd P(N, 3);
    for (int i = 0; i < N; ++i) {
        P(i, 0) = points[i][0]; P(i, 1) = points[i][1]; P(i, 2) = points[i][2];
    }
    Eigen::Vector3d centroid = P.colwise().mean();
    Eigen::MatrixXd Pc = P.rowwise() - centroid.transpose();

    Eigen::Matrix3d cov = Pc.transpose() * Pc;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covSolver(cov);
    if (covSolver.info() != Eigen::Success) return out;
    Eigen::Vector3d e1 = covSolver.eigenvectors().col(2);
    Eigen::Vector3d e2 = covSolver.eigenvectors().col(1);
    Eigen::Vector3d nrm = covSolver.eigenvectors().col(0);
    out.plane_normal = nrm;
    out.rms_off_plane = std::sqrt((Pc * nrm).squaredNorm() / static_cast<double>(N));

    Eigen::VectorXd u = Pc * e1;
    Eigen::VectorXd v = Pc * e2;
    double mx = u.mean(), my = v.mean();
    double sx = std::sqrt((u.array() - mx).square().sum() / N);
    double sy = std::sqrt((v.array() - my).square().sum() / N);
    double s  = std::max(sx, sy);
    if (s < 1e-12) return out;

    Eigen::VectorXd un = (u.array() - mx) / s;
    Eigen::VectorXd vn = (v.array() - my) / s;
    Eigen::MatrixXd D1(N, 3), D2(N, 3);
    for (int i = 0; i < N; ++i) {
        D1(i,0)=un(i)*un(i); D1(i,1)=un(i)*vn(i); D1(i,2)=vn(i)*vn(i);
        D2(i,0)=un(i);       D2(i,1)=vn(i);       D2(i,2)=1.0;
    }
    Eigen::Matrix3d S1 = D1.transpose() * D1;
    Eigen::Matrix3d S2 = D1.transpose() * D2;
    Eigen::Matrix3d S3 = D2.transpose() * D2;
    Eigen::Matrix3d C1; C1 << 0,0,2, 0,-1,0, 2,0,0;
    Eigen::Matrix3d M = C1.inverse() * (S1 - S2 * S3.inverse() * S2.transpose());

    Eigen::EigenSolver<Eigen::Matrix3d> es(M);
    if (es.info() != Eigen::Success) return out;
    int best = -1;
    double bestC = -std::numeric_limits<double>::infinity();
    Eigen::Matrix3cd V = es.eigenvectors();
    for (int k = 0; k < 3; ++k) {
        Eigen::Vector3d vk(V(0,k).real(), V(1,k).real(), V(2,k).real());
        double c = 4.0 * vk(0) * vk(2) - vk(1) * vk(1);
        if (c > bestC) { bestC = c; best = k; }
    }
    if (best < 0 || bestC <= 0) return out;
    Eigen::Vector3d a1(V(0,best).real(), V(1,best).real(), V(2,best).real());
    Eigen::Vector3d a2 = -S3.inverse() * S2.transpose() * a1;

    double A_ = a1(0), B_ = a1(1), C_ = a1(2);
    double D_ = a2(0), E_ = a2(1), F_ = a2(2);
    double A = A_/(s*s), B = B_/(s*s), C = C_/(s*s);
    double D = D_/s - 2*A_*mx/(s*s) - B_*my/(s*s);
    double E = E_/s - 2*C_*my/(s*s) - B_*mx/(s*s);
    double F = (A_*mx*mx + B_*mx*my + C_*my*my)/(s*s) - (D_*mx + E_*my)/s + F_;

    Eigen::Matrix2d M0; M0 << 2*A,B, B,2*C;
    Eigen::Vector2d uv_c = M0.colPivHouseholderQr().solve(Eigen::Vector2d(-D, -E));
    double uc = uv_c(0), vc = uv_c(1);
    double F0 = A*uc*uc + B*uc*vc + C*vc*vc + D*uc + E*vc + F;

    Eigen::Matrix2d Q; Q << A,B/2, B/2,C;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> qSolver(Q);
    double a2_sq = -F0 / qSolver.eigenvalues()(0);
    double b2_sq = -F0 / qSolver.eigenvalues()(1);
    if (a2_sq <= 0 || b2_sq <= 0) return out;
    double aa = std::sqrt(a2_sq);
    double bb = std::sqrt(b2_sq);
    Eigen::Vector2d v_major = qSolver.eigenvectors().col(0);
    double theta = std::atan2(v_major(1), v_major(0));

    out.center     = centroid + uc * e1 + vc * e2;
    out.a          = aa; out.b = bb;
    out.axis_major =  std::cos(theta) * e1 + std::sin(theta) * e2;
    out.axis_minor = -std::sin(theta) * e1 + std::cos(theta) * e2;

    // ---- Sign convention + axis assignment -----------------------------
    // Eigensolver vraća osi sortirane po duljini, ali kad su a i b bliske,
    // to ne odgovara nužno tome koja os fizički ide u smjeru u_ref / v_ref.
    // Provjerimo i prebacimo ako treba.
    {
        // Koja je os bliža u_ref smjeru (po apsolutnoj vrijednosti dot-producta)?
        double major_along_u = std::abs(out.axis_major.dot(u_ref));
        double minor_along_u = std::abs(out.axis_minor.dot(u_ref));

        if (minor_along_u > major_along_u) {
            // Eigensolver je dao "krive" oznake — zamijeni osi i pripadne duljine
            std::swap(out.axis_major, out.axis_minor);
            std::swap(out.a, out.b);
        }

        // Sada lokalno predznake na ono što odgovara u_ref/v_ref
        if (out.axis_major.dot(u_ref) < 0) out.axis_major = -out.axis_major;
        if (out.axis_minor.dot(v_ref) < 0) out.axis_minor = -out.axis_minor;

        // Plane normal iz cross-producta da bude desnoruka
        out.plane_normal = out.axis_major.cross(out.axis_minor).normalized();
    }
    // ---- DEBUG ----
    std::cout << "==== fit_ellipse_3d DEBUG ====" << std::endl;
    std::cout << "  u_ref received: (" << u_ref(0) << ", " << u_ref(1) << ", " << u_ref(2) << ")" << std::endl;
    std::cout << "  v_ref received: (" << v_ref(0) << ", " << v_ref(1) << ", " << v_ref(2) << ")" << std::endl;
    std::cout << "  axis_major:     (" << out.axis_major(0) << ", " << out.axis_major(1) << ", " << out.axis_major(2) << ")" << std::endl;
    std::cout << "  axis_minor:     (" << out.axis_minor(0) << ", " << out.axis_minor(1) << ", " << out.axis_minor(2) << ")" << std::endl;
    std::cout << "  axis_major . u_ref = " << out.axis_major.dot(u_ref) << "  (mora biti > 0)" << std::endl;
    std::cout << "  axis_minor . v_ref = " << out.axis_minor.dot(v_ref) << "  (mora biti > 0)" << std::endl;
    std::cout << "==============================" << std::endl;

    const int M_samples = 2000;
    std::vector<Eigen::Vector3d> dense(M_samples);
    for (int i = 0; i < M_samples; ++i) {
        double t = 2.0 * M_PI * i / M_samples;
        dense[i] = out.center + (aa * std::cos(t)) * out.axis_major
                              + (bb * std::sin(t)) * out.axis_minor;
    }
    double sum_d2 = 0.0;
    for (int i = 0; i < N; ++i) {
        Eigen::Vector3d p = P.row(i).transpose();
        double mind2 = std::numeric_limits<double>::infinity();
        for (const auto& q : dense) {
            double d2 = (p - q).squaredNorm();
            if (d2 < mind2) mind2 = d2;
        }
        sum_d2 += mind2;
    }
    out.rms_to_ellipse = std::sqrt(sum_d2 / N);
    out.valid = true;
    return out;
}

// ---- orthonormalize_basis ----
void orthonormalize_basis(Vec3& u, Vec3& v)
{
    u = normalize(u);
    v = v - dot(v, u) * u;
    v = normalize(v);
}

// ---- rotation_matrix_to_fanuc_wpr ----
// FANUC WPR iz rotacijske matrice. Konvencija: R = Rz(R)*Ry(P)*Rx(W)
void rotation_matrix_to_fanuc_wpr(const Mat3& R,
                                  double& W_deg, double& P_deg, double& R_deg)
{
    double sy = std::sqrt(R.at(0,0)*R.at(0,0) + R.at(1,0)*R.at(1,0));
    bool singular = sy < 1e-9;
    double W, P, Rz;
    if (!singular) {
        W  = std::atan2( R.at(2,1), R.at(2,2));   // oko X
        P  = std::atan2(-R.at(2,0), sy);           // oko Y
        Rz = std::atan2( R.at(1,0), R.at(0,0));    // oko Z
    } else {
        W  = std::atan2(-R.at(1,2), R.at(1,1));
        P  = std::atan2(-R.at(2,0), sy);
        Rz = 0.0;
    }
    constexpr double rad2deg = 180.0 / M_PI;
    W_deg = W * rad2deg;
    P_deg = P * rad2deg;
    R_deg = Rz * rad2deg;
}

// ---- generate_fanuc_buffer_from_ellipse ----
// Generira XYZWPR tocke duz 3D elipse i sprema u std::vector
// (svaki red: {X, Y, Z, W, P, R}).
std::vector<std::array<double, 6>> generate_fanuc_buffer_from_ellipse(
    const Vec3& C, double a, double b,
    const Vec3& u, const Vec3& v, const Vec3& n_plane_unit,
    double start_deg, double end_deg, double step_deg)
{
    
    if (std::abs(step_deg) < 1e-12)
        throw std::runtime_error("step_deg ne smije biti nula.");
    if ((end_deg > start_deg && step_deg <= 0.0) ||
        (end_deg < start_deg && step_deg >= 0.0))
        throw std::runtime_error("Predznak step_deg nije dobar za zadani start_deg i end_deg.");

    std::vector<std::array<double, 6>> FANUC_buffer;
    int n_steps = static_cast<int>(std::round((end_deg - start_deg) / step_deg)) + 1;
    if (n_steps < 1) n_steps = 0;
    FANUC_buffer.reserve(n_steps);

    for (int i = 0; i < n_steps; ++i) {
        double t_deg = start_deg + i * step_deg;
        double t_rad = t_deg * M_PI / 180.0;
        double ct = std::cos(t_rad);
        double st = std::sin(t_rad);



        // Tocka, tangenta i normala u ravnini elipse
        Vec3 point          = C + (a * ct) * u + (b * st) * v;
        Vec3 T_unit         = normalize((-a * st) * u + (b * ct) * v);
        Vec3 n_ellipse_unit = normalize((ct / a) * u + (st / b) * v);

        // Relativni okvir alata (kao u tvom Python kodu)
        Vec3 x_rel =  normalize(n_plane_unit);
        Vec3 y_rel = -n_ellipse_unit;
        Vec3 z_rel = -T_unit;
        Mat3 R_mat(x_rel, y_rel, z_rel);

        double W_deg, P_deg, R_deg;
        rotation_matrix_to_fanuc_wpr(R_mat, W_deg, P_deg, R_deg);

        FANUC_buffer.push_back({point.x, point.y, point.z, W_deg, P_deg, R_deg});
    }
    return FANUC_buffer;
}



// ---- fanuc_buffer_from_fit ----
// Adapter: prima EllipseFit3D iz fit_ellipse_3d, vraca FANUC buffer.
std::vector<std::array<double, 6>> fanuc_buffer_from_fit(
    const EllipseFit3D& ef,
    double start_deg, double end_deg, double step_deg)
{
    if (!ef.valid) {
        std::cout << "[fanuc_buffer_from_fit] EllipseFit3D nije valjan." << std::endl;
        return {};
    }
    Vec3 C(ef.center.x(),       ef.center.y(),       ef.center.z());
    Vec3 u(ef.axis_major.x(),   ef.axis_major.y(),   ef.axis_major.z());  // direction multiplied by `a`
    Vec3 v(ef.axis_minor.x(),   ef.axis_minor.y(),   ef.axis_minor.z());  // direction multiplied by `b`
    Vec3 n(ef.plane_normal.x(), ef.plane_normal.y(), ef.plane_normal.z());

    // axis_major and axis_minor are already orthonormal from the fit;
    // call orthonormalize_basis as a safety belt anyway.
    orthonormalize_basis(u, v);
    n = normalize(n);

    return generate_fanuc_buffer_from_ellipse(
        C, ef.a, ef.b, u, v, n,
        start_deg, end_deg, step_deg);
}

// za projekt je bilo tako da sam se krenao 10 po 10 milimetara posalo poziciju i nakon toga posao da mi izračuna korekciju točno u svakom pomaku računa korekciju što stvori jako veliki korigirani buffer 

// sljedeći korak i korak koji treba naoraviti je da ide krenja robota koji snima cijelo vrijeme dok se kreće kroz zavar i šalje pozicije preko handshake 11111 nakon što prođe kroz zavar ima puni postion buffer
// tada mu šaljemo kod 21000 koji za sve pozicije u position bufferu računa korekciju i sprema ih u korigirani buffer, a zatim šaljemo koordinate iz korigiranog buffera natrag robotu  preko handshake 31000 te ih spremamo u pozicijske registre te se nakon toga gibamo kroz zavar s korigiranom putanjom

// ono što je glavni cilj je automatsko snimanje i korekcija putanje te gibanje kroz zavar s korigiranom putanjom, a ne da se svaki put šalju pojedinačne pozicije i da se čeka na korekciju, jer to stvara preveliki buffer i preduga čekanja kamera treba gledati par koraka ispred alata za zavarivanje korisiti će se FIFO buffer koji će se puniti kontinuirano dok se robot giba kroz zavar, a kamera snima, i onda će se nakon zavara poslati signal da se izračunaju korekcije za sve pozicije u FIFO bufferu i da se pošalju natrag robotu, a zatim će se robot kretati kroz zavar s korigiranom putanjom

