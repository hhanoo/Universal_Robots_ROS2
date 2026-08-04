#include <QApplication>
#include <QFont>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>

#include "mainwindow.h"

int main(int argc, char *argv[]) {
    // 1. ROS init
    rclcpp::init(argc, argv);

    // 2. Qt app + default font (stylesheet lives in mainwindow.ui)
    QApplication a(argc, argv);
    a.setFont(QFont("Ubuntu", 10));

    MainWindow w;
    w.show();

    // 3. Quit the GUI when ROS goes down (rclcpp consumes Ctrl+C)
    QTimer ros_shutdown_watcher;
    QObject::connect(&ros_shutdown_watcher, &QTimer::timeout, &a, [&a]() {
        if (!rclcpp::ok()) {
            a.quit();
        }
    });
    ros_shutdown_watcher.start(100);

    // 4. Event loop, then ROS shutdown
    int ret = a.exec();

    rclcpp::shutdown();
    return ret;
}
