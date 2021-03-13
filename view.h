#pragma once

#include <QGraphicsPixmapItem>
#include <cmath>
#include "vec.h"
#include "mesh.h"

struct Viewport {
    QImage image;

    int width() const {
        return image.width();
    }

    int height() const {
        return image.height();
    }

    // 0 <= intensity <= 1
    void draw_point_grayscale(QPoint p, float intensity) {
        int c = std::min(static_cast<int>(intensity * 255.0f), 255);
        image.setPixel(p, qRgb(c, c, c));
    }

    Viewport() {}
    Viewport(QImage im) : image(std::move(im)) {}
};

struct ProjectedMesh {
    std::vector<float> depths;
    std::vector<spt::vec2f> verts;
    std::vector<Face> faces;

    std::array<float, 2> min_max_coor(std::size_t c) const {
        float minc = std::numeric_limits<float>::max();
        float maxc = std::numeric_limits<float>::lowest();
        for (auto& v : verts) {
            if (v[c] < minc) {
                minc = v[c];
            }
            if (v[c] > maxc) {
                maxc = v[c];
            }
        }
        return {minc, maxc};
    }

    std::array<float, 2> min_max_face_coor(const Face& face, std::size_t c) const {
        float minc = std::numeric_limits<float>::max();
        float maxc = std::numeric_limits<float>::lowest();
        for (auto& v : face.verts) {
            float curc = verts[v.pos][c];
            if (curc < minc) {
                minc = curc;
            }
            if (curc > maxc) {
                maxc = curc;
            }
        }
        return {minc, maxc};
    }

    float face_normal_z(const Face& face) const {
        auto edgevec0 = verts[face.verts[1].pos] - verts[face.verts[0].pos];
        auto edgevec1 = verts[face.verts[2].pos] - verts[face.verts[1].pos];
        return spt::cross(edgevec0, edgevec1);
    }

    spt::vec2f face_center(const Face& face) const {
        return (verts[face.verts[0].pos]
                + verts[face.verts[1].pos]
                + verts[face.verts[2].pos]) / 3.0f;
    }

    //    void update_normals() {
    //        normals = std::vector(verts.size(), spt::vec3f());
    //        for (auto& face : faces) {
    //            auto normal = face_normal(face);
    //            for (auto& v : face.verts) {
    //                normals[v.normal] += normal;
    //            }
    //        }
    //        for (auto& normal : normals) {
    //            normal.normalize();
    //        }
    //    }
};

class Projection {
  public:
    const Viewport& viewport() const {
        return m_viewport;
    }

    Viewport& viewport() {
        return m_viewport;
    }

    virtual void set_viewport(Viewport view) {
        m_viewport = std::move(view);
        m_image_center = spt::vec2f({
            (m_viewport.width() - 1) * 0.5f,
            (m_viewport.height() - 1) * 0.5f
        });
    }

    virtual spt::vec2f to_world(spt::vec2i p) const = 0;

    virtual spt::vec2i to_viewport(spt::vec2f p) const = 0;

    virtual spt::vec2f proj_on_lens(spt::vec3f p) const = 0;

    spt::vec2i proj_on_viewport(spt::vec3f p) const {
        return to_viewport(proj_on_lens(p));
    }

    virtual spt::vec3f proj_on_lens_3d(spt::vec3f p) const = 0;

    virtual spt::vec3f view_vec_to_point(spt::vec3f p) const = 0;

    spt::vec2f bottom_left() const {
        spt::vec2i im_pos({0, m_viewport.height() - 1});
        return to_world(im_pos);
    }

    spt::vec2f bottom_right() const {
        spt::vec2i im_pos({m_viewport.width() - 1, m_viewport.height() - 1});
        return to_world(im_pos);
    }

    spt::vec2f top_left() const {
        spt::vec2i im_pos({0, 0});
        return to_world(im_pos);
    }

    spt::vec2f top_right() const {
        spt::vec2i im_pos({m_viewport.width() - 1, 0});
        return to_world(im_pos);
    }

    // near=1, far=2
    virtual float depth_simple(float z) const = 0;

    // inside [0; 1] segment
    virtual float depth(float z, float near, float far) const = 0;

    virtual std::vector<float> depths(const std::vector<spt::vec3f>& verts) const {
        std::vector<float> res;
        res.reserve(verts.size());
        for (auto& v : verts) {
            res.push_back(depth_simple(v[2]));
        }
        return res;
    }

    virtual bool behind(float depth) const = 0;

    virtual std::vector<spt::vec2f> project_verts(const std::vector<spt::vec3f>& verts) const {
        std::vector<spt::vec2f> res;
        res.reserve(verts.size());
        for (auto& v : verts) {
            res.push_back(proj_on_lens(v));
        }
        return res;
    }

    virtual ProjectedMesh project_mesh(const UniqueMesh& mesh) const {
        ProjectedMesh res;
        res.depths = depths(mesh.verts);
        res.verts = project_verts(mesh.verts);
        res.faces = mesh.faces;
        return res;
    }

    Projection() {}
    Projection(Viewport view) : m_viewport(std::move(view)) {
        m_image_center = spt::vec2f({
            (m_viewport.width() - 1) * 0.5f,
            (m_viewport.height() - 1) * 0.5f
        });
    }

  protected:
    Viewport m_viewport;
    spt::vec2f m_image_center;
};

class OrthographicProjection : public Projection {
  public:
    float pixelsize() const {
        return m_pixelsize;
    }

    void set_pixelsize(float pixelsize) {
        m_pixelsize = pixelsize;
    }

    spt::vec2f to_world(spt::vec2i p) const override {
        spt::vec2f fpos(p);
        spt::vec2f res2d = (fpos - m_image_center) / m_pixelsize;
        return spt::vec3f({res2d[0], -res2d[1], 0.0f});
    }

    spt::vec2i to_viewport(spt::vec2f p) const override {
        p[1] = -p[1];
        auto scaled = p * m_pixelsize;
        return spt::vec2i(scaled + m_image_center);
    }

    spt::vec2f proj_on_lens(spt::vec3f p) const override {
        return spt::vec2f({p[0], p[1]});
    }

    spt::vec3f proj_on_lens_3d(spt::vec3f p) const override {
        auto lensproj = proj_on_lens(p);
        return spt::vec3f({lensproj[0], lensproj[1], 0.0f});
    }

    spt::vec3f view_vec_to_point(spt::vec3f p) const override {
        return spt::vec3f({0.0f, 0.0f, -1.0f});
    }

    float depth_simple(float z) const override {
        return -z;
    }

    float depth(float z, float near, float far) const override {
        return (-z - near) / (far - near);
    }

    bool behind(float depth) const override {
        return false;
    }

    OrthographicProjection() {}
    OrthographicProjection(float pixelsize)
        : m_pixelsize(pixelsize) {}
    OrthographicProjection(Viewport view, float pixelsize)
        : Projection(view), m_pixelsize(pixelsize) {}

  private:
    float m_pixelsize = 1.0f;
};

class PerspectiveProjection : public Projection {
  public:
    float fov() const {
        return m_fov;
    }

    void set_viewport(Viewport view) override {
        m_viewport = std::move(view);
        m_image_center = spt::vec2f({
            (m_viewport.width() - 1) * 0.5f,
            (m_viewport.height() - 1) * 0.5f
        });
        set_fov(m_fov);
    }

    void set_fov(float fov) {
        m_dxim = m_image_center[0];
        m_dyim = m_image_center[1];
        m_dx = std::tan(fov * 0.5f);
        m_dy = m_dyim / m_dxim * m_dx;
    }

    spt::vec2f to_world(spt::vec2i p) const override {
        spt::vec2f fpos(p);
        auto t = fpos - m_image_center;
        return spt::vec3f({
            t[0] * m_dx / m_dxim,
            -t[1] * m_dy / m_dyim
        });
    }

    spt::vec2i to_viewport(spt::vec2f p) const override {
        spt::vec2f t({
            p[0] * m_dxim / m_dx,
            -p[1] * m_dyim / m_dy
        });
        return spt::vec2i(t + m_image_center);
    }

    spt::vec2f proj_on_lens(spt::vec3f p) const override {
        float t = -1.0f / p[2];
        return t * spt::vec2f(p);
    }

    spt::vec3f proj_on_lens_3d(spt::vec3f p) const override {
        auto lensproj = proj_on_lens(p);
        return spt::vec3f({lensproj[0], lensproj[1], -1.0f});
    }

    spt::vec3f view_vec_to_point(spt::vec3f p) const override {
        return p;
    }

    float depth_simple(float z) const override {
        return 1.0f / z;
    }

    float depth(float z, float near, float far) const override {
        return ((far + near) * 0.5f + far * near / z) / (far - near) + 0.5f;
    }

    bool behind(float depth) const override {
        return depth > 0.0;
    }

    PerspectiveProjection() {}
    PerspectiveProjection(float fov)
        : m_fov(fov) {}
    PerspectiveProjection(Viewport view, float fov)
        : Projection(view), m_fov(fov) {
        m_dxim = m_image_center[0];
        m_dyim = m_image_center[1];
        m_dx = std::tan(fov * 0.5f);
        m_dy = m_dyim / m_dxim * m_dx;
    }

  private:
    float m_fov = 1.0f;
    float m_dx, m_dy;
    float m_dxim, m_dyim;
};
