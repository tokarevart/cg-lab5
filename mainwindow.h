#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsScene>
#include <QMainWindow>
#include <array>
#include "graphics-view-3d.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

  private slots:
    void on_render_button_clicked();

    void on_renderer_tabs_currentChanged(int index);

    void on_proj_tabs_currentChanged(int index);

    void on_meshimport_button_clicked();

    void on_pixelsize_sbox_valueChanged(double arg1);

    void on_fov_sbox_valueChanged(double arg1);

    void on_near_sbox_valueChanged(double arg1);

    void on_far_sbox_valueChanged(double arg1);

    void on_light_x_sbox_valueChanged(double arg1);

    void on_light_y_sbox_valueChanged(double arg1);

    void on_light_z_sbox_valueChanged(double arg1);

    void on_light_ia_sbox_valueChanged(double arg1);

    void on_light_ka_sbox_valueChanged(double arg1);

    void on_light_il_sbox_valueChanged(double arg1);

    void on_light_kd_sbox_valueChanged(double arg1);

    void on_light_ks_sbox_valueChanged(double arg1);

    void on_light_n_sbox_valueChanged(double arg1);

  private:
    Ui::MainWindow *ui;
    GraphicsView3D* gview3d;
};
#endif // MAINWINDOW_H
