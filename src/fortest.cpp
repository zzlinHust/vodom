//
// Created by cbt on 18-12-28.
//
#include "fortest.h"

using namespace std;

namespace myslam
{

void log(const std::string &where, const std::string &msg)
{
    static chrono::steady_clock::time_point init = chrono::steady_clock::now();
    static chrono::steady_clock::time_point last = chrono::steady_clock::now();

    chrono::steady_clock::time_point now = chrono::steady_clock::now();

    chrono::duration<double> cur_time = chrono::duration_cast<chrono::duration<double>>(now - init);
    chrono::duration<double> duration = chrono::duration_cast<chrono::duration<double>>(now - last);
    fprintf(stderr, "[%-4.3fs]  [%3.3fms] : %.10s , %s \n",cur_time.count(), 1000.0 * duration.count(), where.c_str(), msg.c_str());

    last = now;
}

}