#pragma once

#include "view.h"
#include "helpers.h"
#include "scene.h"

struct Renderer {
    std::unique_ptr<Projection> proj;
    virtual void render(LocalScene& scene) = 0;
};

struct SimpleRenderer : public Renderer {
    void render(LocalScene& scene) {
        auto& viewport = proj->viewport();
        auto& mesh = scene.mesh;
        ProjectedMesh projmesh = proj->project_mesh(mesh);
        scene.
            behind_cull(*proj, projmesh).
            backface_cull(*proj, projmesh).
            depth_sort(projmesh);

        std::vector<std::array<float, 2>> faces_y_minmaxes;
        faces_y_minmaxes.reserve(projmesh.faces.size());
        for (auto& face : projmesh.faces) {
            faces_y_minmaxes.push_back(projmesh.min_max_face_coor(face, 1));
        }

        auto minmax_y = min_max_image_coor(viewport, proj.get(), projmesh, 1);
        std::vector<float> depth_field(
            viewport.height() * viewport.width(),
            std::numeric_limits<float>::max());

        for (int ay = minmax_y[0]; ay < minmax_y[1]; ++ay) {
            for (std::size_t i = 0; i < projmesh.faces.size(); ++i) {
                auto& face = projmesh.faces[i];
                float world_y = proj->to_world(spt::vec2f({0.0f, static_cast<float>(ay)}))[1];
                if (faces_y_minmaxes[i][0] > world_y || world_y > faces_y_minmaxes[i][1]) {
                    continue;
                }

                std::array<float, 2> ts;
                auto oxinters = triangle_horizline_intersections(projmesh, face, world_y, ts);
                if (!oxinters.has_value()) {
                    continue;
                }
                auto xinters = oxinters.value();

                std::array<float, 2> bnd_depths;
                for (std::size_t j = 0; j < 2; ++j) {
                    auto& edge = xinters[j].edge;
                    float d0 = projmesh.depths[edge[0].pos];
                    float d1 = projmesh.depths[edge[1].pos];
                    bnd_depths[j] = interpolate(d0, d1, ts[j]);
                }

                auto xrange = image_c_range(proj.get(), {xinters[0].p, xinters[1].p}, 0);
                auto xdrawrange = to_image_c_draw_range(viewport, xrange, 0);
                int axlen = xrange[1] - xrange[0];
                int axdrawlen = xdrawrange[1] - xdrawrange[0];
                float inv_axlen = 1.0f / axlen;
                float delta_depth = (bnd_depths[1] - bnd_depths[0]) * inv_axlen;

                auto normal =
                    (mesh.normals[face.verts[0].normal]
                     + mesh.normals[face.verts[1].normal]
                     + mesh.normals[face.verts[2].normal]).normalize();
                float intensity = scene.local_intensity_normalized(
                    mesh.face_center(face), normal);
                int off = xdrawrange[0] - xrange[0];
                float depth = bnd_depths[0] + off * delta_depth;
                int idx = ay * viewport.width() + xdrawrange[0];
                for (int rel_ax = 0; rel_ax < axdrawlen; ++rel_ax) {
                    depth += delta_depth;
                    if (depth < depth_field[idx]) {
                        depth_field[idx] = depth;
                        viewport.draw_point_grayscale({rel_ax + xdrawrange[0], ay}, intensity);
                    }
                    ++idx;
                }
            }
        }
    }
};

struct GouraudRenderer : public Renderer {
    void render(LocalScene& scene) {
        auto& viewport = proj->viewport();
        auto& mesh = scene.mesh;
        ProjectedMesh projmesh = proj->project_mesh(mesh);
        scene.
            behind_cull(*proj, projmesh).
            backface_cull(*proj, projmesh).
            depth_sort(projmesh);

        std::vector<std::vector<float>> faces_verts_intens;
        faces_verts_intens.reserve(mesh.faces.size());
        for (auto& face : projmesh.faces) {
            std::vector<float> intens;
            intens.reserve(face.verts.size());
            for (auto& v : face.verts) {
                float inten = scene.local_intensity_normalized(
                    mesh.verts[v.pos],
                    mesh.normals[v.normal]);
                intens.push_back(inten);
            }
            faces_verts_intens.push_back(std::move(intens));
        }

        std::vector<std::array<float, 2>> faces_y_minmaxes;
        faces_y_minmaxes.reserve(projmesh.faces.size());
        for (auto& face : projmesh.faces) {
            faces_y_minmaxes.push_back(projmesh.min_max_face_coor(face, 1));
        }

        auto minmax_y = min_max_image_coor(viewport, proj.get(), projmesh, 1);
        std::vector<float> depth_field(
            viewport.height() * viewport.width(),
            std::numeric_limits<float>::max());

        for (int ay = minmax_y[0]; ay < minmax_y[1]; ++ay) {
            for (std::size_t i = 0; i < projmesh.faces.size(); ++i) {
                float world_y = proj->to_world(spt::vec2f({0.0f, static_cast<float>(ay)}))[1];
                if (faces_y_minmaxes[i][0] > world_y || world_y > faces_y_minmaxes[i][1]) {
                    continue;
                }

                std::array<float, 2> ts;
                auto oxinters = triangle_horizline_intersections(projmesh, projmesh.faces[i], world_y, ts);
                if (!oxinters.has_value()) {
                    continue;
                }
                auto xinters = oxinters.value();

                std::array<float, 2> intens;
                std::array<float, 2> bnd_depths;
                for (std::size_t j = 0; j < 2; ++j) {
                    auto v0 = faces_verts_intens[i][xinters[j].face_vids[0]];
                    auto v1 = faces_verts_intens[i][xinters[j].face_vids[1]];
                    intens[j] = interpolate(v0, v1, ts[j]);
                    auto& edge = xinters[j].edge;
                    float d0 = projmesh.depths[edge[0].pos];
                    float d1 = projmesh.depths[edge[1].pos];
                    bnd_depths[j] = interpolate(d0, d1, ts[j]);
                }

                auto xrange = image_c_range(proj.get(), {xinters[0].p, xinters[1].p}, 0);
                auto xdrawrange = to_image_c_draw_range(viewport, xrange, 0);
                int axlen = xrange[1] - xrange[0];
                int axdrawlen = xdrawrange[1] - xdrawrange[0];
                float inv_axlen = 1.0f / axlen;

                float delta_inten = (intens[1] - intens[0]) * inv_axlen;
                float delta_depth = (bnd_depths[1] - bnd_depths[0]) * inv_axlen;

                int off = xdrawrange[0] - xrange[0];
                float inten = intens[0] + off * delta_inten;
                float depth = bnd_depths[0] + off * delta_depth;
                int idx = ay * viewport.width() + xdrawrange[0];
                for (int rel_ax = 0; rel_ax < axdrawlen; ++rel_ax) {
                    inten += delta_inten;
                    depth += delta_depth;
                    if (depth < depth_field[idx]) {
                        depth_field[idx] = depth;
                        viewport.draw_point_grayscale({rel_ax + xdrawrange[0], ay}, inten);
                    }
                    ++idx;
                }
            }
        }
    }
};

struct PhongRenderer : public Renderer {
    void render(LocalScene& scene) {
        auto& viewport = proj->viewport();
        auto& mesh = scene.mesh;
        ProjectedMesh projmesh = proj->project_mesh(mesh);
        scene.
            behind_cull(*proj, projmesh).
            backface_cull(*proj, projmesh).
            depth_sort(projmesh);

        std::vector<std::array<float, 2>> faces_y_minmaxes;
        faces_y_minmaxes.reserve(projmesh.faces.size());
        for (auto& face : projmesh.faces) {
            faces_y_minmaxes.push_back(projmesh.min_max_face_coor(face, 1));
        }

        auto minmax_y = min_max_image_coor(viewport, proj.get(), projmesh, 1);
        std::vector<float> depth_field(
            viewport.height() * viewport.width(),
            std::numeric_limits<float>::max());

        for (int ay = minmax_y[0]; ay < minmax_y[1]; ++ay) {
            for (std::size_t i = 0; i < projmesh.faces.size(); ++i) {
                float world_y = proj->to_world(spt::vec2f({0.0f, static_cast<float>(ay)}))[1];
                if (faces_y_minmaxes[i][0] > world_y || world_y > faces_y_minmaxes[i][1]) {
                    continue;
                }

                std::array<float, 2> ts;
                auto oxinters = triangle_horizline_intersections(projmesh, projmesh.faces[i], world_y, ts);
                if (!oxinters.has_value()) {
                    continue;
                }
                auto xinters = oxinters.value();

                auto xrange = image_c_range(proj.get(), {xinters[0].p, xinters[1].p}, 0);
                auto xdrawrange = to_image_c_draw_range(viewport, xrange, 0);
                int axlen = xrange[1] - xrange[0];
                int axdrawlen = xdrawrange[1] - xdrawrange[0];
                float inv_axlen = 1.0f / axlen;

                std::array<spt::vec3f, 2> bnd_pts;
                std::array<float, 2> bnd_depths;
                for (std::size_t j = 0; j < 2; ++j) {
                    auto& edge = xinters[j].edge;
                    auto v0p = mesh.verts[edge[0].pos];
                    auto v1p = mesh.verts[edge[1].pos];
                    bnd_pts[j] = interpolate(v0p, v1p, ts[j]);
                    float d0 = projmesh.depths[edge[0].pos];
                    float d1 = projmesh.depths[edge[1].pos];
                    bnd_depths[j] = interpolate(d0, d1, ts[j]);
                }
                float min_depth = std::min(bnd_depths[0], bnd_depths[1]);

                bool draw = false;
                int off = xdrawrange[0] - xrange[0];
                int idx = ay * viewport.width() + xdrawrange[0];
                for (int i = 0; i < axdrawlen; ++i) {
                    if (min_depth < depth_field[idx]) {
                        draw = true;
                        break;
                    }
                    ++idx;
                }
                if (!draw) {
                    continue;
                }

                std::array<spt::vec3f, 2> bnd_normals;
                for (std::size_t j = 0; j < 2; ++j) {
                    auto& edge = xinters[j].edge;
                    auto v0n = mesh.normals[edge[0].normal];
                    auto v1n = mesh.normals[edge[1].normal];
                    bnd_normals[j] = interpolate(v0n, v1n, ts[j]).normalize();
                }

                spt::vec3f delta_p = (bnd_pts[1] - bnd_pts[0]) * inv_axlen;
                spt::vec3f delta_normal = (bnd_normals[1] - bnd_normals[0]) * inv_axlen;
                float delta_depth = (bnd_depths[1] - bnd_depths[0]) * inv_axlen;

                auto p = bnd_pts[0] + static_cast<float>(off) * delta_p;
                auto notnormal = bnd_normals[0];
                float depth = bnd_depths[0] + off * delta_depth;
                idx = (ay - minmax_y[0]) * viewport.width() + xdrawrange[0];
                for (int rel_ax = 0; rel_ax < axdrawlen; ++rel_ax) {
                    depth += delta_depth;
                    p += delta_p;
                    notnormal += delta_normal;
                    if (depth < depth_field[idx]) {
                        depth_field[idx] = depth;
                        auto normal = notnormal.normalize();
                        float intensity = scene.local_intensity_normalized(p, normal);
                        viewport.draw_point_grayscale({rel_ax + xdrawrange[0], ay}, intensity);
                    }
                    ++idx;
                }
            }
        }
    }
};

struct WireframeRenderer : public Renderer {
    void render(LocalScene& scene) override {
        ProjectedMesh projmesh = proj->project_mesh(scene.mesh);
        scene.behind_cull(*proj, projmesh);
        std::vector<std::array<std::size_t, 2>> edges;


        std::array<std::array<spt::vec2f, 2>, 4> sides{
            std::array{proj->bottom_left(), proj->bottom_right()},
            std::array{proj->bottom_right(), proj->top_right()},
            std::array{proj->top_left(), proj->top_right()},
            std::array{proj->bottom_left(), proj->top_left()}
        };

        float world_width = proj->top_right()[0];
        float world_height = proj->top_right()[1];

        auto push_segm_from_edge =
            [&projmesh, &sides, world_height, world_width]
            (auto& segms, std::size_t v0, std::size_t v1) {

            spt::vec2f p0 = projmesh.verts[v0];
            spt::vec2f p1 = projmesh.verts[v1];
            if (p0[0] > p1[0]) {
                std::swap(p0, p1);
            }
            bool in0 = std::abs(p0[0]) <= world_width && std::abs(p0[1]) <= world_height;
            bool in1 = std::abs(p1[0]) <= world_width && std::abs(p1[1]) <= world_height;
            if (in0 && in1) {
                segms.push_back({p0, p1});
            } else if (!in0 && in1) {
                for (auto& side : std::array{sides[0], sides[2], sides[3]}) {
                    auto ointer = spt::segment_intersect_segment(p0, p1, side[0], side[1]);
                    if (ointer.has_value()) {
                        segms.push_back({ointer.value(), p1});
                        break;
                    }
                }
            } else if (in0 && !in1) {
                for (auto& side : std::array{sides[0], sides[1], sides[2]}) {
                    auto ointer = spt::segment_intersect_segment(p0, p1, side[0], side[1]);
                    if (ointer.has_value()) {
                        segms.push_back({p0, ointer.value()});
                        break;
                    }
                }
            } else {
                std::vector<spt::vec2f> inters;
                for (auto& side : sides) {
                    auto ointer = spt::segment_intersect_segment(p0, p1, side[0], side[1]);
                    if (ointer.has_value()) {
                        segms.push_back({p0, ointer.value()});
                    }
                }
                if (!inters.empty()) {
                    if (inters[0][0] > inters[1][0]) {
                        std::swap(inters[0], inters[1]);
                    }
                    segms.push_back({inters[0], inters[1]});
                }
            }
        };

        std::vector<std::array<spt::vec2f, 2>> segms;
        segms.reserve(edges.size());
        for (Face& face : projmesh.faces) {
            for (std::size_t i = 0; i < 3; ++i) {
                auto fedge = face.edge(i);
                std::size_t p0 = fedge[0].pos;
                std::size_t p1 = fedge[1].pos;
                push_segm_from_edge(segms, p0, p1);
            }
        }

        for (auto segm : segms) {
            spt::vec2i p0 = proj->to_viewport(segm[0]);
            spt::vec2i p1 = proj->to_viewport(segm[1]);
            draw_segm(p0, p1);
        }
    }

  private:
    void try_draw_point(spt::vec2<int> p) {
        auto& viewport = proj->viewport();
        if (p[0] >= 0 && p[0] < viewport.width()
            && p[1] >= 0 && p[1] < viewport.height()) {
            QPoint qp(p[0], p[1]);
            viewport.draw_point_grayscale(qp, 0.0f);
        }
    }

    // Bresenham's line algorithm
    void draw_segm(spt::vec2i p0, spt::vec2i p1) {
        int x0 = p0[0];
        int x1 = p1[0];
        int y0 = p0[1];
        int y1 = p1[1];
        int dx = std::abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true) {
            try_draw_point(std::array{x0, y0});
            if (x0 == x1 && y0 == y1) {
                break;
            }
            int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
};

struct DepthRenderer : public Renderer {
    void render(LocalScene& scene) {
        auto& viewport = proj->viewport();
        auto& mesh = scene.mesh;
        ProjectedMesh projmesh = proj->project_mesh(mesh);
        scene.
            behind_cull(*proj, projmesh).
            backface_cull(*proj, projmesh).
            depth_sort(projmesh);

        std::vector<std::array<float, 2>> faces_y_minmaxes;
        faces_y_minmaxes.reserve(projmesh.faces.size());
        for (auto& face : projmesh.faces) {
            faces_y_minmaxes.push_back(projmesh.min_max_face_coor(face, 1));
        }

        auto minmax_y = min_max_image_coor(viewport, proj.get(), projmesh, 1);
        std::vector<float> depth_field(
            viewport.height() * viewport.width(),
            std::numeric_limits<float>::max());

        for (int ay = minmax_y[0]; ay < minmax_y[1]; ++ay) {
            for (std::size_t i = 0; i < projmesh.faces.size(); ++i) {
                auto& face = projmesh.faces[i];
                float world_y = proj->to_world(spt::vec2f({0.0f, static_cast<float>(ay)}))[1];
                if (faces_y_minmaxes[i][0] > world_y || world_y > faces_y_minmaxes[i][1]) {
                    continue;
                }

                std::array<float, 2> ts;
                auto oxinters = triangle_horizline_intersections(projmesh, face, world_y, ts);
                if (!oxinters.has_value()) {
                    continue;
                }
                auto xinters = oxinters.value();

                std::array<float, 2> bnd_depths;
                for (std::size_t j = 0; j < 2; ++j) {
                    auto& edge = xinters[j].edge;
                    float d0 = proj->depth(mesh.verts[edge[0].pos][2], m_near, m_far);
                    float d1 = proj->depth(mesh.verts[edge[1].pos][2], m_near, m_far);
                    bnd_depths[j] = interpolate(d0, d1, ts[j]);
                }

                auto xrange = image_c_range(proj.get(), {xinters[0].p, xinters[1].p}, 0);
                auto xdrawrange = to_image_c_draw_range(viewport, xrange, 0);
                int axlen = xrange[1] - xrange[0];
                int axdrawlen = xdrawrange[1] - xdrawrange[0];
                float inv_axlen = 1.0f / axlen;
                float delta_depth = (bnd_depths[1] - bnd_depths[0]) * inv_axlen;

                int off = xdrawrange[0] - xrange[0];
                float depth = bnd_depths[0] + off * delta_depth;
                int idx = ay * viewport.width() + xdrawrange[0];
                for (int rel_ax = 0; rel_ax < axdrawlen; ++rel_ax) {
                    depth += delta_depth;
                    if (depth < depth_field[idx]) {
                        depth_field[idx] = depth;
                        float intensity = std::clamp(depth, 0.0f, 1.0f);
                        viewport.draw_point_grayscale({rel_ax + xdrawrange[0], ay}, intensity);
                    }
                    ++idx;
                }
            }
        }
    }

    DepthRenderer(float near, float far) : m_near(near), m_far(far) {}

  private:
    float m_near, m_far;
};
