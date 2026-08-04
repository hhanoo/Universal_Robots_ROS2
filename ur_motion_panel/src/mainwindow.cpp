#include "mainwindow.h"

#include <QDebug>
#include <QStyle>
#include <QThread>
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>

#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    // 1. UI setup
    ui->setupUi(this);
    this->setWindowTitle("UR Motion Panel");

    // 2. Widget wiring
    // 2-1. Connect / STOP
    connect(ui->btnRobotConnect, &QPushButton::clicked, this, &MainWindow::btnRobotConnect_clicked);
    connect(ui->btn_stop, &QPushButton::clicked, this, [this]() {
        if (client_) {
            client_->moveCancel();
        }
    });

    // 2-2. Joint
    connect(ui->btn_print_joint, &QPushButton::clicked, this, &MainWindow::btn_print_joint_clicked);
    connect(ui->btn_apply_joint, &QPushButton::clicked, this, &MainWindow::btn_apply_joint_clicked);
    connect(ui->btn_moveJ, &QPushButton::clicked, this, &MainWindow::btn_moveJ_clicked);
    current_joint = {ui->txt_Joint1, ui->txt_Joint2, ui->txt_Joint3, ui->txt_Joint4, ui->txt_Joint5, ui->txt_Joint6};
    target_joint  = {ui->spin_Joint1, ui->spin_Joint2, ui->spin_Joint3, ui->spin_Joint4, ui->spin_Joint5, ui->spin_Joint6};

    // 2-3. TCP (X, Y, Z)
    connect(ui->btn_print_kinematrics, &QPushButton::clicked, this, &MainWindow::btn_print_kinematrics_clicked);
    connect(ui->btn_apply_kinematrics, &QPushButton::clicked, this, &MainWindow::btn_apply_kinematrics_clicked);
    connect(ui->btn_moveL, &QPushButton::clicked, this, &MainWindow::btn_moveL_clicked);
    current_xyz  = {ui->txt_X, ui->txt_Y, ui->txt_Z};
    current_rvec = {ui->txt_RX, ui->txt_RY, ui->txt_RZ};
    target_xyz   = {ui->spin_X, ui->spin_Y, ui->spin_Z};
    target_rvec  = {ui->spin_RX, ui->spin_RY, ui->spin_RZ};

    // 2-4. TCP jog (± step)
    connect(ui->btn_x_m, &QPushButton::clicked, this, [this]() { xyz_jog_clicked(0, -1.0); });
    connect(ui->btn_x_p, &QPushButton::clicked, this, [this]() { xyz_jog_clicked(0, 1.0); });
    connect(ui->btn_y_m, &QPushButton::clicked, this, [this]() { xyz_jog_clicked(1, -1.0); });
    connect(ui->btn_y_p, &QPushButton::clicked, this, [this]() { xyz_jog_clicked(1, 1.0); });
    connect(ui->btn_z_m, &QPushButton::clicked, this, [this]() { xyz_jog_clicked(2, -1.0); });
    connect(ui->btn_z_p, &QPushButton::clicked, this, [this]() { xyz_jog_clicked(2, 1.0); });

    // 2-5. TCP (RX, RY, RZ) - tool frame
    connect(ui->btn_rx_m, &QPushButton::clicked, this, &MainWindow::btn_rx_m_clicked);
    connect(ui->btn_rx_p, &QPushButton::clicked, this, &MainWindow::btn_rx_p_clicked);
    connect(ui->btn_ry_m, &QPushButton::clicked, this, &MainWindow::btn_ry_m_clicked);
    connect(ui->btn_ry_p, &QPushButton::clicked, this, &MainWindow::btn_ry_p_clicked);
    connect(ui->btn_rz_m, &QPushButton::clicked, this, &MainWindow::btn_rz_m_clicked);
    connect(ui->btn_rz_p, &QPushButton::clicked, this, &MainWindow::btn_rz_p_clicked);

    // 2-5b. TCP (RX, RY, RZ) - base frame (position fixed)
    connect(ui->btn_brx_m, &QPushButton::clicked, this, [this]() { base_rotate_clicked(0, -1.0); });
    connect(ui->btn_brx_p, &QPushButton::clicked, this, [this]() { base_rotate_clicked(0, 1.0); });
    connect(ui->btn_bry_m, &QPushButton::clicked, this, [this]() { base_rotate_clicked(1, -1.0); });
    connect(ui->btn_bry_p, &QPushButton::clicked, this, [this]() { base_rotate_clicked(1, 1.0); });
    connect(ui->btn_brz_m, &QPushButton::clicked, this, [this]() { base_rotate_clicked(2, -1.0); });
    connect(ui->btn_brz_p, &QPushButton::clicked, this, [this]() { base_rotate_clicked(2, 1.0); });

    // 2-6. Speed apply button (Set value is sent only when pressed)
    connect(ui->btn_speed_apply, &QPushButton::clicked, this, &MainWindow::sendSpeedSlider);

    // 3. Load saved config
    setting_config = new SettingConfig(ui);
    setting_config->loadConfigFile();

    // 4. Status update timer (50 ms)
    mainTimer = new QTimer(this);
    mainTimer->setInterval(50);
    connect(mainTimer, SIGNAL(timeout()), this, SLOT(statusUpdate()));
    mainTimer->start();
}

MainWindow::~MainWindow() {
    setting_config->saveConfigFile();
    move_future.waitForFinished();
    client_.reset();
    delete ui;
    delete setting_config;
}

// -------------------- Main --------------------
void MainWindow::statusUpdate() {
    // 1. Connection state
    is_connected = client_ && client_->isConnected();

    // 2. Button enable (MoveL/jog/rotate also need a TCP pose)
    bool can_movej = is_connected && !is_moving;
    bool can_movel = can_movej && tcp_available;
    ui->btn_moveJ->setEnabled(can_movej);
    ui->btn_moveL->setEnabled(can_movel);
    ui->btn_x_m->setEnabled(can_movel);
    ui->btn_x_p->setEnabled(can_movel);
    ui->btn_y_m->setEnabled(can_movel);
    ui->btn_y_p->setEnabled(can_movel);
    ui->btn_z_m->setEnabled(can_movel);
    ui->btn_z_p->setEnabled(can_movel);
    ui->btn_rx_m->setEnabled(can_movel);
    ui->btn_rx_p->setEnabled(can_movel);
    ui->btn_ry_m->setEnabled(can_movel);
    ui->btn_ry_p->setEnabled(can_movel);
    ui->btn_rz_m->setEnabled(can_movel);
    ui->btn_rz_p->setEnabled(can_movel);
    ui->btn_brx_m->setEnabled(can_movel);
    ui->btn_brx_p->setEnabled(can_movel);
    ui->btn_bry_m->setEnabled(can_movel);
    ui->btn_bry_p->setEnabled(can_movel);
    ui->btn_brz_m->setEnabled(can_movel);
    ui->btn_brz_p->setEnabled(can_movel);
    ui->btn_stop->setEnabled(is_connected && is_moving);
    ui->btn_speed_apply->setEnabled(is_connected);

    // 3. Connect button text
    if (is_connected) {
        ui->btnRobotConnect->setText("Disconnect");
    } else {
        ui->btnRobotConnect->setText("Connect");
    }

    // 4. Status pill
    const char *st;
    const char *st_color;
    QString     status_text;
    if (!is_connected) {
        st          = "off";
        st_color    = "#8a94a3";
        status_text = "Disconnected — start run-all, then Connect";
    } else if (is_moving) {
        st          = "run";
        st_color    = "#e67e22";
        status_text = "Moving — STOP cancels the motion";
    } else if (client_->isRemoteControl() == 0) {
        st          = "warn";
        st_color    = "#c0392b";
        status_text = "Connected — pendant is in Local mode, motions will not run";
    } else {
        st          = "on";
        st_color    = "#35a860";
        status_text = "Connected";
    }
    if (ui->label_status->text() != status_text) {
        ui->label_status->setText(status_text);
    }
    if (ui->label_status->property("st").toString() != st) {
        ui->label_status->setProperty("st", st);
        ui->label_status->setStyleSheet(
            QString("background:%1; color:white; border-radius:12px;").arg(st_color));
    }

    // 5. Actual speed label
    QString actual = is_connected
                         ? QString::number(client_->getSpeedScaling() * 100.0, 'f', 0) + " %"
                         : QString("—");
    if (ui->label_speed_actual->text() != actual) {
        ui->label_speed_actual->setText(actual);
    }

    // 6. Robot state
    if (is_connected) {
        // 6-1. Joint positions
        current_jnt = client_->getJointPositions();
        for (int i = 0; i < 6; i++) {
            current_joint[i]->setText(QString::number(current_jnt[i] * 180. / M_PI, 'f', 2));
        }

        // 6-2. TCP pose (position + base-frame rotation vector, UR pendant convention)
        tcp_available = client_->isTcpPoseAvailable();
        if (tcp_available) {
            current_mat = client_->getTcpPose();
            current_xyz[0]->setText(QString::number(current_mat[3] * 1000, 'f', 2));
            current_xyz[1]->setText(QString::number(current_mat[7] * 1000, 'f', 2));
            current_xyz[2]->setText(QString::number(current_mat[11] * 1000, 'f', 2));

            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(current_mat.data());
            Eigen::AngleAxisd aa(T.block<3, 3>(0, 0));
            Eigen::Vector3d   rvec = aa.axis() * aa.angle() * 180. / M_PI;
            for (int i = 0; i < 3; i++) {
                current_rvec_val[i] = rvec[i];
                current_rvec[i]->setText(QString::number(rvec[i], 'f', 2));
            }
        } else {
            current_xyz[0]->setText("N/A");
            current_xyz[1]->setText("N/A");
            current_xyz[2]->setText("N/A");
            for (auto *e : current_rvec) {
                e->setText("N/A");
            }
        }

        // 6-3. Digital I/O
        checkBox_DIO_update();
    } else {
        tcp_available = false;
    }

    // 7. Stylesheet state
    updateStateStyle();
}

// Push connection/motion state to the stylesheet (repolish only on change)
void MainWindow::updateStateStyle() {
    bool moving_now = is_moving.load();
    if (style_applied && style_connected == is_connected && style_moving == moving_now) {
        return;
    }
    style_applied   = true;
    style_connected = is_connected;
    style_moving    = moving_now;

    const char *mode     = !is_connected ? "off" : (moving_now ? "run" : "live");
    auto        repolish = [](QWidget *w) {
        w->style()->unpolish(w);
        w->style()->polish(w);
    };

    for (auto *e : current_joint) {
        e->setProperty("mode", mode);
        repolish(e);
    }
    for (auto *e : current_xyz) {
        e->setProperty("mode", mode);
        repolish(e);
    }
    for (auto *e : current_rvec) {
        e->setProperty("mode", mode);
        repolish(e);
    }
}

// -------------------- Connect --------------------
void MainWindow::btnRobotConnect_clicked() {
    if (is_connected) {
        // 1. Disconnect (in-flight motions hold their own shared_ptr)
        client_.reset();
        is_connected  = false;
        tcp_available = false;
    } else {
        // 2. Connect
        client_ = std::make_shared<URRobotClient>();

        // 2-1. Wait for the first joint_states
        for (int i = 0; i < 20 && !client_->isConnected(); i++) {
            QThread::msleep(100);
        }
        is_connected = client_->isConnected();

        if (is_connected) {
            // 2-2. Push initial speed (best-effort; absent in fake hardware mode)
            if (client_->waitRobotReady(2.0)) {
                client_->setSpeedSlider(std::clamp(ui->spinBox_velocity->value() / 100.0, 0.01, 1.0));
            }
        } else {
            qWarning() << "UR stack not responding (no joint_states) - is the driver running?";
            client_.reset();
        }
    }
}

// -------------------- Speed slider --------------------
void MainWindow::sendSpeedSlider() {
    if (!is_connected) {
        return;
    }
    double fraction = std::clamp(ui->spinBox_velocity->value() / 100.0, 0.01, 1.0);
    auto   client   = client_;

    // Fire-and-forget off the GUI thread (service wait can block up to 1s)
    speed_future = QtConcurrent::run([client, fraction]() {
        client->setSpeedSlider(fraction);
    });
}

// -------------------- Joint --------------------
void MainWindow::btn_print_joint_clicked() {
    if (is_connected) {
        printf(
            "\n[Joint]: \n \
            [%.8f, %.8f, %.8f, %.8f, %.8f, %.8f]\n",
            current_jnt[0], current_jnt[1], current_jnt[2],
            current_jnt[3], current_jnt[4], current_jnt[5]);
    }
}

void MainWindow::btn_apply_joint_clicked() {
    if (is_connected) {
        for (int i = 0; i < 6; i++) {
            target_joint[i]->setValue(current_jnt[i] * 180. / M_PI);
        }
    }
}

void MainWindow::btn_moveJ_clicked() {
    // 1. Guard: connected & not already moving
    if (!is_connected) {
        return;
    }
    bool expected = false;
    if (!is_moving.compare_exchange_strong(expected, true)) {
        return;
    }

    // 2. Target joints (deg -> rad)
    std::vector<double> target_J(6);
    for (int i = 0; i < 6; i++) {
        target_J[i] = target_joint[i]->value() * M_PI / 180.;
    }

    // 3. MoveJ on a worker thread (plan at 1.0; real speed = robot speed slider)
    auto client = client_;

    move_future = QtConcurrent::run([this, client, target_J]() {
        auto motion = client->moveJ(target_J, 1.0);
        // 3-1. Wait, escaping on Ctrl+C (executor thread dies on shutdown)
        while (motion.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (!rclcpp::ok()) {
                is_moving.store(false);
                return;
            }
        }
        // 3-2. Report result
        auto result = motion.get();
        if (!result.success) {
            qWarning() << "MoveJ failed:" << QString::fromStdString(result.message);
        }
        is_moving.store(false);
    });
}

// -------------------- TCP (X, Y, Z) --------------------
void MainWindow::btn_print_kinematrics_clicked() {
    if (is_connected) {
        printf(
            "\n[Kinematics]: \n \
            [%.5f, %.5f, %.5f, %.5f,\n \
            %.5f, %.5f, %.5f, %.5f,\n \
            %.5f, %.5f, %.5f, %.5f,\n \
            %.5f, %.5f, %.5f, %.5f]\n",
            current_mat[0], current_mat[1], current_mat[2], current_mat[3],
            current_mat[4], current_mat[5], current_mat[6], current_mat[7],
            current_mat[8], current_mat[9], current_mat[10], current_mat[11],
            current_mat[12], current_mat[13], current_mat[14], current_mat[15]);
    }
}

void MainWindow::btn_apply_kinematrics_clicked() {
    if (is_connected && tcp_available) {
        target_xyz[0]->setValue(current_mat[3] * 1000);
        target_xyz[1]->setValue(current_mat[7] * 1000);
        target_xyz[2]->setValue(current_mat[11] * 1000);
        for (int i = 0; i < 3; i++) {
            target_rvec[i]->setValue(current_rvec_val[i]);
        }
    }
}

// Shared MoveL worker (plan at 1.0; real speed = robot speed slider)
void MainWindow::startMoveL(const std::array<double, 16> &move_T) {
    auto client = client_;

    move_future = QtConcurrent::run([this, client, move_T]() {
        auto motion = client->moveL(move_T, 1.0);
        // Wait, escaping on Ctrl+C (executor thread dies on shutdown)
        while (motion.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (!rclcpp::ok()) {
                is_moving.store(false);
                return;
            }
        }
        auto result = motion.get();
        if (!result.success) {
            qWarning() << "MoveL failed:" << QString::fromStdString(result.message);
        }
        is_moving.store(false);
    });
}

void MainWindow::btn_moveL_clicked() {
    if (!is_connected || !tcp_available) {
        return;
    }
    bool expected = false;
    if (!is_moving.compare_exchange_strong(expected, true)) {
        return;
    }

    // Keep orientation, replace position
    // Full 6-DOF target: position + base-frame rotation vector
    std::array<double, 16> move_T = current_mat;
    move_T[3]                     = target_xyz[0]->value() / 1000.;
    move_T[7]                     = target_xyz[1]->value() / 1000.;
    move_T[11]                    = target_xyz[2]->value() / 1000.;

    Eigen::Vector3d rv(target_rvec[0]->value(), target_rvec[1]->value(), target_rvec[2]->value());
    rv *= M_PI / 180.0;
    double angle = rv.norm();

    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(move_T.data());
    T.block<3, 3>(0, 0) = (angle < 1e-9)
                              ? Eigen::Matrix3d::Identity()
                              : Eigen::AngleAxisd(angle, rv / angle).toRotationMatrix();

    startMoveL(move_T);
}

void MainWindow::xyz_jog_clicked(int axis_index, double sign) {
    if (!is_connected || !tcp_available) {
        return;
    }
    bool expected = false;
    if (!is_moving.compare_exchange_strong(expected, true)) {
        return;
    }

    static const int       kPosOffset[3] = {3, 7, 11};
    std::array<double, 16> move_T        = current_mat;
    move_T[kPosOffset[axis_index]] += sign * ui->spinBox_xyz_step->value() / 1000.0;

    startMoveL(move_T);
}

// -------------------- TCP (RX, RY, RZ) --------------------
void MainWindow::tcp_rotate(const char *axis, double radians, double result_T[16]) {
    // 1. Current pose -> Eigen affine (row-major 4x4)
    double current_T[16];
    std::copy(current_mat.begin(), current_mat.end(), current_T);

    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> current_T_matrix(current_T);
    Eigen::Transform<double, 3, Eigen::Affine, Eigen::RowMajor>    current_affine(current_T_matrix);

    // 2. Rotation about the requested axis
    Eigen::Transform<double, 3, Eigen::Affine, Eigen::RowMajor> rotation_affine = Eigen::Transform<double, 3, Eigen::Affine, Eigen::RowMajor>::Identity();

    switch (*axis) {
        case 'X':
            rotation_affine.rotate(Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitX()));
            break;
        case 'Y':
            rotation_affine.rotate(Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitY()));
            break;
        case 'Z':
            rotation_affine.rotate(Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()));
            break;
        default:
            std::cerr << "Invalid axis: " << axis << std::endl;
            break;
    }

    // 3. Compose and write back to result_T
    Eigen::Transform<double, 3, Eigen::Affine, Eigen::RowMajor> result_affine   = current_affine * rotation_affine;
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor>                result_T_matrix = result_affine.matrix();

    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> result_T_matrix_map(result_T);
    result_T_matrix_map = result_T_matrix;
}

void MainWindow::rotate_clicked(const char *axis, double sign_deg) {
    if (!is_connected || !tcp_available) {
        return;
    }
    bool expected = false;
    if (!is_moving.compare_exchange_strong(expected, true)) {
        return;
    }

    double radians = sign_deg * M_PI / 180.0;
    double result_T[16];

    tcp_rotate(axis, radians, result_T);

    std::array<double, 16> move_T;
    std::copy(std::begin(result_T), std::end(result_T), move_T.begin());

    startMoveL(move_T);
}

// Rotate about a base axis while keeping the TCP position fixed
void MainWindow::base_rotate_clicked(int axis_index, double sign) {
    if (!is_connected || !tcp_available) {
        return;
    }
    bool expected = false;
    if (!is_moving.compare_exchange_strong(expected, true)) {
        return;
    }

    double radians = sign * ui->spinBox_rot_step->value() * M_PI / 180.0;

    std::array<double, 16> move_T = current_mat;

    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T(move_T.data());
    T.block<3, 3>(0, 0) = (Eigen::AngleAxisd(radians, Eigen::Vector3d::Unit(axis_index)) * T.block<3, 3>(0, 0)).eval();

    startMoveL(move_T);
}

void MainWindow::btn_rx_m_clicked() {
    rotate_clicked("X", -ui->spinBox_rx->value());
}

void MainWindow::btn_rx_p_clicked() {
    rotate_clicked("X", ui->spinBox_rx->value());
}

void MainWindow::btn_ry_m_clicked() {
    rotate_clicked("Y", -ui->spinBox_ry->value());
}

void MainWindow::btn_ry_p_clicked() {
    rotate_clicked("Y", ui->spinBox_ry->value());
}

void MainWindow::btn_rz_m_clicked() {
    rotate_clicked("Z", -ui->spinBox_rz->value());
}

void MainWindow::btn_rz_p_clicked() {
    rotate_clicked("Z", ui->spinBox_rz->value());
}

// -------------------- Digital I/O --------------------
void MainWindow::checkBox_DIO_update() {
    if (is_connected) {
        // 1. Inputs: robot -> checkboxes
        int temp_digital_in = 0;
        for (int pin = 0; pin < 8; pin++) {
            if (client_->getDigitalIn(pin)) {
                temp_digital_in |= (1 << pin);
            }
        }
        digital_in = temp_digital_in;
        ui->checkBox_input_0->setChecked(digital_in & 0b1);
        ui->checkBox_input_1->setChecked(digital_in & 0b10);
        ui->checkBox_input_2->setChecked(digital_in & 0b100);
        ui->checkBox_input_3->setChecked(digital_in & 0b1000);
        ui->checkBox_input_4->setChecked(digital_in & 0b10000);
        ui->checkBox_input_5->setChecked(digital_in & 0b100000);
        ui->checkBox_input_6->setChecked(digital_in & 0b1000000);
        ui->checkBox_input_7->setChecked(digital_in & 0b10000000);

        // 2. Outputs: read checkboxes
        int temp_digital_out = 0;
        if (ui->checkBox_output_0->isChecked())
            temp_digital_out += 0b1;
        if (ui->checkBox_output_1->isChecked())
            temp_digital_out += 0b10;
        if (ui->checkBox_output_2->isChecked())
            temp_digital_out += 0b100;
        if (ui->checkBox_output_3->isChecked())
            temp_digital_out += 0b1000;
        if (ui->checkBox_output_4->isChecked())
            temp_digital_out += 0b10000;
        if (ui->checkBox_output_5->isChecked())
            temp_digital_out += 0b100000;
        if (ui->checkBox_output_6->isChecked())
            temp_digital_out += 0b1000000;
        if (ui->checkBox_output_7->isChecked())
            temp_digital_out += 0b10000000;

        // 3. Push only the changed pins
        if (temp_digital_out != digital_out) {
            for (int pin = 0; pin < 8; pin++) {
                bool new_value = temp_digital_out & (1 << pin);
                bool old_value = digital_out & (1 << pin);
                if (new_value != old_value) {
                    client_->setDigitalOut(pin, new_value);
                }
            }
            digital_out = temp_digital_out;
        }
    }
}
