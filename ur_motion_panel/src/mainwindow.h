#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <Eigen/Dense>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFuture>
#include <QLineEdit>
#include <QMainWindow>
#include <QTimer>
#include <QtConcurrent/QtConcurrent>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>

#include "setting_config.h"
#include "ur_robot_client/ur_robot_client.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

   public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

   private:
    // -------------------- UI --------------------
    Ui::MainWindow *ui;
    SettingConfig  *setting_config;

    // -------------------- Robot --------------------
    std::shared_ptr<URRobotClient> client_;  // owns its own executor thread

    // State snapshots (refreshed by statusUpdate on the GUI thread)
    std::array<double, 6>  current_jnt{};
    std::array<double, 16> current_mat{};
    std::array<double, 3>  current_rvec_val{};  // base-frame rotation vector [deg]
    bool                   tcp_available = false;

    bool                            is_connected = false;
    int                             digital_in   = 0;
    int                             digital_out  = 0;
    std::atomic<bool>               is_moving    = false;
    QFuture<void>                   move_future;
    std::array<QLineEdit *, 6>      current_joint;
    std::array<QDoubleSpinBox *, 6> target_joint;
    std::array<QLineEdit *, 3>      current_xyz;
    std::array<QLineEdit *, 3>      current_rvec;  // base-frame rotation vector [deg]
    std::array<QDoubleSpinBox *, 3> target_xyz;
    std::array<QDoubleSpinBox *, 3> target_rvec;

    // -------------------- Main --------------------
    QTimer *mainTimer;

    // Speed slider apply (worker keeps the service wait off the GUI thread)
    QFuture<void> speed_future;

    // Stylesheet state cache (repolish only on change)
    bool style_applied   = false;
    bool style_connected = false;
    bool style_moving    = false;

    // Motion helpers (MoveL / jog / rotate share startMoveL)
    void startMoveL(const std::array<double, 16> &move_T);
    void rotate_clicked(const char *axis, double sign_deg);
    void base_rotate_clicked(int axis_index, double sign);
    void xyz_jog_clicked(int axis_index, double sign);
    void updateStateStyle();

   private slots:
    // -------------------- Main --------------------
    void statusUpdate();

    // -------------------- Connect --------------------
    void btnRobotConnect_clicked();

    // -------------------- Speed slider --------------------
    void sendSpeedSlider();

    // -------------------- Joint --------------------
    void btn_print_joint_clicked();
    void btn_apply_joint_clicked();
    void btn_moveJ_clicked();

    // -------------------- TCP (X, Y, Z) --------------------
    void btn_print_kinematrics_clicked();
    void btn_apply_kinematrics_clicked();
    void btn_moveL_clicked();

    // -------------------- TCP (RX, RY, RZ) --------------------
    void tcp_rotate(const char *axis, double radians, double result_T[16]);
    void btn_rx_m_clicked();
    void btn_rx_p_clicked();
    void btn_ry_m_clicked();
    void btn_ry_p_clicked();
    void btn_rz_m_clicked();
    void btn_rz_p_clicked();

    // -------------------- Digital I/O --------------------
    void checkBox_DIO_update();
};
#endif  // MAINWINDOW_H
