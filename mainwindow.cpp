#include "mainwindow.h"

#include <QVector2D>
#include <QGraphicsPixmapItem>
#include <algorithm>
#include <memory>
#include "sptalgs.h"
#include "render.h"
#include "scene.h"

#include "ui_mainwindow.h"

spt::mat3f direct_z_along_vec(spt::mat3f orient, spt::vec3f v) {
    auto y = spt::cross(v, orient[0]).normalize();
    auto x = spt::cross(y, v).normalize();
    v.normalize();
    return spt::mat3f(x, y, v);
}

spt::vec3f parse_camera_pos(const Ui::MainWindow* ui) {
    return spt::vec3f({
        static_cast<float>(ui->campos_x_sbox->value()),
        static_cast<float>(ui->campos_y_sbox->value()),
        static_cast<float>(ui->campos_z_sbox->value())
    });
}

spt::mat3f parse_camera_orient(const Ui::MainWindow* ui) {
    return spt::mat3f(direct_z_along_vec(
        spt::mat3f::identity(),
        -spt::vec3f({
            static_cast<float>(ui->camdir_x_sbox->value()),
            static_cast<float>(ui->camdir_y_sbox->value()),
            static_cast<float>(ui->camdir_z_sbox->value()) })));
}

spt::vec3f parse_light_pos(const Ui::MainWindow* ui) {
    return spt::vec3f({
        static_cast<float>(ui->light_x_sbox->value()),
        static_cast<float>(ui->light_y_sbox->value()),
        static_cast<float>(ui->light_z_sbox->value())
    });
}

SimpleIllum parse_illum(const Ui::MainWindow* ui) {
    double ia = ui->light_ia_sbox->value();
    double ka = ui->light_ka_sbox->value();
    double il = ui->light_il_sbox->value();
    double kd = ui->light_kd_sbox->value();
    double ks = ui->light_ks_sbox->value();
    double n = ui->light_n_sbox->value();
    AmbientLight ambient(ia, ka);
    DiffuseLight diffuse(il, kd);
    PhongLight phong(il, ks, n);
    return SimpleIllum(ambient, diffuse, phong);
}

float parse_pixelsize(const Ui::MainWindow* ui) {
    return ui->pixelsize_sbox->value();
}

float parse_fov(const Ui::MainWindow* ui) {
    return ui->fov_sbox->value() / 180.0 * 3.14159265359;
}

Camera parse_camera(const Ui::MainWindow* ui) {
    Camera cam;
    cam.pos = parse_camera_pos(ui);
    cam.orient = parse_camera_orient(ui);
    return cam;
}

PointLight parse_light(const Ui::MainWindow* ui) {
    spt::vec3f lightpos = parse_light_pos(ui);
    PointLight light(lightpos, parse_illum(ui));
    return light;
}

LocalScene parse_scene(const Ui::MainWindow* ui) {
    LocalScene scene;
    scene.light = parse_light(ui);
    scene.mesh = UniqueMesh::load_obj(ui->meshfile_ledit->text().toLocal8Bit().constData());
    return scene;
}

std::unique_ptr<Projection> parse_projection(const Ui::MainWindow* ui) {
    switch (ui->proj_tabs->currentIndex()) {
    case 0: return std::make_unique<OrthographicProjection>(parse_pixelsize(ui));
    case 1: return std::make_unique<PerspectiveProjection>(parse_fov(ui));
    default: return nullptr;
    }
}

float parse_near(const Ui::MainWindow* ui) {
    return static_cast<float>(ui->near_sbox->value());
}

float parse_far(const Ui::MainWindow* ui) {
    return static_cast<float>(ui->far_sbox->value());
}

std::unique_ptr<Renderer> parse_renderer(const Ui::MainWindow* ui) {
    switch (ui->renderer_tabs->currentIndex()) {
    case 0: return std::make_unique<SimpleRenderer>();
    case 1: return std::make_unique<GouraudRenderer>();
    case 2: return std::make_unique<PhongRenderer>();
    case 3: return std::make_unique<WireframeRenderer>();
    case 4: return std::make_unique<DepthRenderer>(parse_near(ui), parse_far(ui));
    default: return nullptr;
    }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    gview3d = new GraphicsView3D(this);
    ui->gview3d_layout->addWidget(gview3d);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_render_button_clicked() {
    gview3d->renderer = parse_renderer(ui);
    gview3d->renderer->proj = parse_projection(ui);
    gview3d->render();
}

void MainWindow::on_renderer_tabs_currentChanged(int index) {
    gview3d->renderer = parse_renderer(ui);
    gview3d->renderer->proj = parse_projection(ui);
    gview3d->render();
}

void MainWindow::on_proj_tabs_currentChanged(int index) {
    gview3d->renderer->proj = parse_projection(ui);
    gview3d->render();
}

void MainWindow::on_meshimport_button_clicked() {
    gview3d->set_scene(parse_scene(ui));
    gview3d->set_camera(parse_camera(ui));
    gview3d->renderer = parse_renderer(ui);
    gview3d->renderer->proj = parse_projection(ui);
    gview3d->render();
}

void MainWindow::on_pixelsize_sbox_valueChanged(double arg1) {
    gview3d->renderer->proj = parse_projection(ui);
    gview3d->render();
}

void MainWindow::on_fov_sbox_valueChanged(double arg1) {
    gview3d->renderer->proj = parse_projection(ui);
    gview3d->render();
}

void MainWindow::on_near_sbox_valueChanged(double arg1) {
    gview3d->renderer = parse_renderer(ui);
    gview3d->renderer->proj = parse_projection(ui);
    gview3d->render();
}

void MainWindow::on_far_sbox_valueChanged(double arg1) {
    gview3d->renderer = parse_renderer(ui);
    gview3d->renderer->proj = parse_projection(ui);
    gview3d->render();
}

void MainWindow::on_light_x_sbox_valueChanged(double arg1) {
    gview3d->set_light(parse_light(ui));
    gview3d->render();
}

void MainWindow::on_light_y_sbox_valueChanged(double arg1) {
    gview3d->set_light(parse_light(ui));
    gview3d->render();
}

void MainWindow::on_light_z_sbox_valueChanged(double arg1) {
    gview3d->set_light(parse_light(ui));
    gview3d->render();
}

void MainWindow::on_light_ia_sbox_valueChanged(double arg1) {
    gview3d->set_light(parse_light(ui));
    gview3d->render();
}

void MainWindow::on_light_ka_sbox_valueChanged(double arg1) {
    gview3d->set_light(parse_light(ui));
    gview3d->render();
}

void MainWindow::on_light_il_sbox_valueChanged(double arg1) {
    gview3d->set_light(parse_light(ui));
    gview3d->render();
}

void MainWindow::on_light_kd_sbox_valueChanged(double arg1) {
    gview3d->set_light(parse_light(ui));
    gview3d->render();
}

void MainWindow::on_light_ks_sbox_valueChanged(double arg1) {
    gview3d->set_light(parse_light(ui));
    gview3d->render();
}

void MainWindow::on_light_n_sbox_valueChanged(double arg1) {
    gview3d->set_light(parse_light(ui));
    gview3d->render();
}
