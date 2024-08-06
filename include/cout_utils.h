//
// Created by hyx020222 on 8/6/24.
//

#ifndef PX4CTRL_COUT_UTILS_H
#define PX4CTRL_COUT_UTILS_H

#include <iostream>
#include <limits>

using namespace std;

const string RESET_COLOR = "\033[0m";
const string RED_COLOR = "\033[31m";
const string GREEN_COLOR = "\033[32m";
const string YELLOW_COLOR = "\033[33m";
const string BLUE_COLOR = "\033[34m";
const string MAGENTA_COLOR = "\033[35m";
const string CYAN_COLOR = "\033[36m";
const string WHITE_COLOR = "\033[37m";

void cout_color(const string &msg, const string &color) {
    cout << color << msg << RESET_COLOR << endl;
}

#endif //PX4CTRL_COUT_UTILS_H
