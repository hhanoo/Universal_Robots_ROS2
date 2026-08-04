#pragma once

#include <QApplication>
#include <QDesktopServices>
#include <QFile>
#include <QMainWindow>
#include <QSettings>
#include <QWidget>

class SettingConfig : public QSettings {
   public:
    SettingConfig(void* _ui);
    void    saveConfigFile();
    void    loadConfigFile();
    QString configFile();

   private:
    void* ui;
};
