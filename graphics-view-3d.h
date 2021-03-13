#pragma once

#include <QGraphicsView>
#include <QMouseEvent>
#include <QGuiApplication>
#include "scene.h"
#include "render.h"

class GraphicsView3D : public QGraphicsView
{
    Q_OBJECT

  public:
    std::unique_ptr<Renderer> renderer;

    Camera camera() const {
        return m_cam;
    }

    PointLight light() const {
        return m_scene.light;
    }

    void set_scene(LocalScene scene) {
        m_scene = std::move(scene);
        m_scene.transform_to_camera(m_cam);
    }

    void set_camera(Camera cam) {
        m_scene.transform_to_camera(m_cam, cam);
        m_cam = cam;
    }

    void set_light(PointLight light) {
        m_scene.light = light;
        m_scene.transform_light_to_camera(m_cam);
    }

    void set_mesh(UniqueMesh mesh) {
        m_scene.mesh = std::move(mesh);
        m_scene.transform_mesh_to_camera(m_cam);
    }

    GraphicsView3D& move_camera(spt::vec3f shift) {
        m_cam.pos += spt::dot(m_cam.orient, shift);
        m_scene.translate(-shift);
        return *this;
    }

    GraphicsView3D& move_camera_to(spt::vec3f to) {
        auto shift = to - m_cam.pos;
        return move_camera(shift);
    }

    GraphicsView3D& rotate_camera(spt::vec3f along, float angle) {
        auto cam_rot = spt::rotation(along, angle);
        m_cam.orient = spt::dot(cam_rot, m_cam.orient);
        auto scene_rot = spt::rotation(along, -angle);
        m_scene.rotate(scene_rot);
        return *this;
    }

    void render() {
        QImage image(size(), QImage::Format_RGB32);
        image.fill(Qt::white);
        Viewport viewport(image);
        renderer->proj->set_viewport(std::move(viewport));
        auto scene = LocalScene(m_scene);
        renderer->render(scene);

        m_gscene->clear();
        m_gscene->setSceneRect(rect());
        m_gscene->addPixmap(
            QPixmap::fromImage(
                renderer->proj->viewport().image))->setPos(0, 0);
    }

//    void render_ghost(Camera actual_cam) {
//        QImage image(size(), QImage::Format_RGB32);
//        image.fill(Qt::white);
//        Viewport viewport(image);
//        // scene.backface_cull();
//        renderer->proj->set_viewport(std::move(viewport));
//        GhostScene ghost(m_scene, actual_cam, m_cam);
//        renderer->render(ghost);

//        m_gscene->clear();
//        m_gscene->setSceneRect(rect());
//        m_gscene->addPixmap(
//            QPixmap::fromImage(
//                renderer->proj->viewport().image))->setPos(0, 0);
//    }

    void mousePressEvent(QMouseEvent* event) override {
        qprevpos = event->pos();
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        QPoint qcurpos = event->pos();
        QPoint qdelta = qcurpos - qprevpos;
        qprevpos = qcurpos;

        if (event->buttons() & Qt::LeftButton) {
            if (QGuiApplication::queryKeyboardModifiers().testFlag(Qt::ShiftModifier)) {
                spt::vec3f shift;
                shift[2] = qdelta.y();
                float coef = 0.01f;
                move_camera(shift * coef);
            } else {
                spt::vec3f along;
                along[0] = -qdelta.y();
                along[1] = -qdelta.x();
                float coef = 0.004f;
                rotate_camera(spt::vec3f(along).normalize(), along.magnitude() * coef);
            }
            render();

        } else if (event->buttons() & Qt::RightButton) {
            if (QGuiApplication::queryKeyboardModifiers().testFlag(Qt::ShiftModifier)) {
                spt::vec3f along({0.0f, 0.0f, -1.0f});
                float angle = qdelta.x();
                float coef = 0.003f;
                rotate_camera(along, angle * coef);
            } else {
                spt::vec3f shift;
                shift[0] = -qdelta.x();
                shift[1] = qdelta.y();
                float coef = 0.01f;
                move_camera(shift * coef);

            }
            render();
        }
    }

    GraphicsView3D(QWidget* parent = nullptr)
        : QGraphicsView(parent), m_gscene(new QGraphicsScene()) {
        setScene(m_gscene);
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    }

  private:
    LocalScene m_scene;
    Camera m_cam;
    QGraphicsScene* m_gscene = nullptr;
    QPoint qprevpos;
};
