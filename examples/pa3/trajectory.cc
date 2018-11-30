#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "../data/trajectory.txt";
string es_trajectory_file= "../data/estimated.txt";
string gt_trajectory_file = "../data/groundtruth.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

  if(argc == 2)
  {
    trajectory_file = std::string(argv[1]);
    std::cout << "Traj.: " << trajectory_file << std::endl;
  }
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_estimate;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_gt;

    std::ifstream file_es(es_trajectory_file);
    std::ifstream file_gt(gt_trajectory_file);
    double timestamp, tx, ty, tz, qx, qy, qz, qw;
    std::string line;
    /// implement pose reading code
    // start your code here (5~10 lines)
    while(!file_es.eof())
    {
      std::getline(file_es, line);
      std::stringstream sin(line);
      sin >> timestamp;
      sin >> tx;
      sin >> ty;
      sin >> tz;
      sin >> qx;
      sin >> qy;
      sin >> qz;
      sin >> qw;
      Eigen::Quaterniond q(qw, qx, qy, qz);
      Eigen::Vector3d t(tx, ty, tz);
      Sophus::SE3 se3_qt(q, t);
      poses_estimate.push_back(se3_qt);
      //std::cout << "time: " << timestamp
      //          << "\ttx: " << tx
      //          << "\tty: " << ty
      //          << "\ttz: " << tz
      //          << "\nqx: " << qx
      //          << "\tqy: " << qy
      //          << "\tqz: " << qz
      //          << "\tqw: " << qw
      //          << std::endl;
    }
    // end your code here
    while(!file_gt.eof())
    {
      std::getline(file_gt, line);
      std::stringstream sin(line);
      sin >> timestamp;
      sin >> tx;
      sin >> ty;
      sin >> tz;
      sin >> qx;
      sin >> qy;
      sin >> qz;
      sin >> qw;
      Eigen::Quaterniond q(qw, qx, qy, qz);
      Eigen::Vector3d t(tx, ty, tz);
      Sophus::SE3 se3_qt(q, t);
      poses_gt.push_back(se3_qt);
      //std::cout << "time: " << timestamp
      //          << "\ttx: " << tx
      //          << "\tty: " << ty
      //          << "\ttz: " << tz
      //          << "\nqx: " << qx
      //          << "\tqy: " << qy
      //          << "\tqz: " << qz
      //          << "\tqw: " << qw
      //          << std::endl;
    }

    // draw trajectory in pangolin
    DrawTrajectory(poses_estimate);
    DrawTrajectory(poses_gt);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
