#pragma once

#include <fstream>
#include "point-light.h"
#include "mesh.h"
#include "view.h"
#include "camera.h"
#include "helpers.h"

struct LocalScene {
    PointLight light;
    UniqueMesh mesh;

    double local_intensity(spt::vec3f at, spt::vec3f normal) const {
        return light.local_intensity(at, normal);
    }

    double local_intensity_normalized(spt::vec3f at, spt::vec3f normal) const {
        return light.local_intensity_normalized(at, normal);
    }

    LocalScene& translate_light(spt::vec3f move) {
        light.pos += move;
        return *this;
    }

    LocalScene& translate_mesh(spt::vec3f move) {
        for (auto& vert : mesh.verts) {
            vert += move;
        }
        return *this;
    }

    LocalScene& translate(spt::vec3f move) {
        return translate_light(move).translate_mesh(move);
    }

    LocalScene& rotate_light(spt::mat3f rot) {
        light.pos = spt::dot(rot, light.pos);
        return *this;
    }

    LocalScene& rotate_mesh(spt::mat3f rot) {
        for (auto& vert : mesh.verts) {
            vert = spt::dot(rot, vert);
        }
        for (auto& normal : mesh.normals) {
            normal = spt::dot(rot, normal);
        }
        return *this;
    }

    LocalScene& rotate(spt::mat3f rot) {
        return rotate_light(rot).rotate_mesh(rot);
    }

    LocalScene& linear_transform(spt::mat3f a, spt::vec3f b) {
        light.pos = spt::dot(a, light.pos) + b;
        for (auto& vert : mesh.verts) {
            vert = spt::dot(a, vert) + b;
        }
        for (auto& normal : mesh.normals) {
            normal = spt::dot(a, normal);
        }
        return *this;
    }

    // run second
    LocalScene& backface_cull(const Projection& proj, ProjectedMesh& projmesh) {
        std::vector<Face> vis_faces;
        vis_faces.reserve(projmesh.faces.size());
        for (std::size_t i = 0; i < projmesh.faces.size(); ++i) {
            auto& face = projmesh.faces[i];
            auto p0 = mesh.verts[face.verts[0].pos];
            auto p1 = mesh.verts[face.verts[1].pos];
            auto p2 = mesh.verts[face.verts[2].pos];
            auto n0 = mesh.normals[face.verts[0].normal];
            auto n1 = mesh.normals[face.verts[1].normal];
            auto n2 = mesh.normals[face.verts[2].normal];
            if (spt::dot(n0, proj.view_vec_to_point(p0)) < 0.0f
                || spt::dot(n1, proj.view_vec_to_point(p1)) < 0.0f
                || spt::dot(n2, proj.view_vec_to_point(p2)) < 0.0f) {
                vis_faces.push_back(projmesh.faces[i]);
            }
        }
        vis_faces.shrink_to_fit();
        projmesh.faces = std::move(vis_faces);
        return *this;
    }

    // run first
    LocalScene& behind_cull(const Projection& proj, ProjectedMesh& projmesh) {
        std::vector<std::size_t> behind_verts;
        behind_verts.reserve(projmesh.depths.size());
        for (std::size_t i = 0; i < projmesh.depths.size(); ++i) {
            if (proj.behind(projmesh.depths[i])) {
                behind_verts.push_back(i);
            }
        }

        auto behind_verts_inface_ids = [&proj, &projmesh](const Face& f) {
            std::vector<std::size_t> beh;
            std::vector<std::size_t> ins;
            for (std::size_t i = 0; i < f.verts.size(); ++i) {
                if (proj.behind(projmesh.depths[f.verts[i].pos])) {
                    beh.push_back(i);
                } else {
                    ins.push_back(i);
                }
            }
            return std::pair(std::move(beh), std::move(ins));
        };

        auto behind_vert_new_pos = [](spt::vec3f bp, spt::vec3f q, float& t) {
            auto d = q - bp;
            t = std::min(bp[2] / -d[2] + 1e5f * std::numeric_limits<float>::epsilon(), 1.0f);
            return bp + t * d;
        };

        auto add_vert = [this, &projmesh, &proj](spt::vec3f p, spt::vec3f n) {
            mesh.verts.push_back(p);
            mesh.normals.push_back(n);
            projmesh.verts.push_back(proj.proj_on_lens(p));
            projmesh.depths.push_back(proj.depth_simple(p[2]));
            return Vert{mesh.verts.size() - 1, mesh.normals.size() - 1};
        };

        std::vector<Face> vis_faces;
        vis_faces.reserve(behind_verts.size());
        for (std::size_t i = 0; i < projmesh.faces.size(); ++i) {
            Face face = projmesh.faces[i];
            if (!std::any_of(
                    behind_verts.begin(), behind_verts.end(),
                    [&face](std::size_t bv) { return face.contains_vert(bv); })) {
                vis_faces.push_back(face);
            } else {
                auto [beh, ins] = behind_verts_inface_ids(face);
                if (beh.size() == 1) {
                    Vert ins_v0 = face.verts[ins[0]];
                    Vert ins_v1 = face.verts[ins[1]];
                    Vert beh_v = face.verts[beh[0]];
                    float t;
                    auto new_p0 = behind_vert_new_pos(mesh.verts[beh_v.pos], mesh.verts[ins_v0.pos], t);
                    auto new_n0 = interpolate(mesh.normals[beh_v.normal], mesh.normals[ins_v0.normal], t).normalize();
                    auto new_p1 = behind_vert_new_pos(mesh.verts[beh_v.pos], mesh.verts[ins_v1.pos], t);
                    auto new_n1 = interpolate(mesh.normals[beh_v.normal], mesh.normals[ins_v1.normal], t).normalize();

                    auto new_v0 = add_vert(new_p0, new_n0);
                    auto new_v1 = add_vert(new_p1, new_n1);
                    mesh.faces[i].verts[beh[0]] = new_v0;
                    projmesh.faces[i].verts[beh[0]] = new_v0;
                    Face new_f = face;
                    new_f.verts[ins[0]] = new_v0;
                    new_f.verts[beh[0]] = new_v1;
                    mesh.faces.push_back(new_f);
                    projmesh.faces.push_back(new_f);

                    vis_faces.push_back(mesh.faces[i]);
                    vis_faces.push_back(new_f);

                } else if (beh.size() == 2) {
                    Vert ins_v = face.verts[ins[0]];
                    Vert beh_v0 = face.verts[beh[0]];
                    Vert beh_v1 = face.verts[beh[1]];
                    float t;
                    auto new_p0 = behind_vert_new_pos(mesh.verts[beh_v0.pos], mesh.verts[ins_v.pos], t);
                    auto new_n0 = interpolate(mesh.normals[beh_v0.normal], mesh.normals[ins_v.normal], t).normalize();
                    auto new_p1 = behind_vert_new_pos(mesh.verts[beh_v1.pos], mesh.verts[ins_v.pos], t);
                    auto new_n1 = interpolate(mesh.normals[beh_v1.normal], mesh.normals[ins_v.normal], t).normalize();

                    auto new_v0 = add_vert(new_p0, new_n0);
                    auto new_v1 = add_vert(new_p1, new_n1);
                    mesh.faces[i].verts[beh[0]] = new_v0;
                    mesh.faces[i].verts[beh[1]] = new_v1;
                    projmesh.faces[i].verts[beh[0]] = new_v0;
                    projmesh.faces[i].verts[beh[1]] = new_v1;

                    vis_faces.push_back(projmesh.faces[i]);
                }
            }
        }

        projmesh.faces = std::move(vis_faces);
        return *this;
    }

    // run third
    const LocalScene& depth_sort(ProjectedMesh& projmesh) const {
        std::sort(
            projmesh.faces.begin(), projmesh.faces.end(),
            [&projmesh](auto& left, auto& right) {
                auto min_depth = [&projmesh](Face& f) {
                    return std::min({
                        projmesh.depths[f.verts[0].pos],
                        projmesh.depths[f.verts[1].pos],
                        projmesh.depths[f.verts[2].pos]
                    });
                };
                return min_depth(left) < min_depth(right);
        });
        return *this;
    }

    LocalScene& transform_light_to_camera(Camera cam) {
        auto rot = spt::mat3f(cam.orient).inversed();
        return translate_light(-cam.pos).rotate_light(rot);
    }

    LocalScene& transform_mesh_to_camera(Camera cam) {
        auto rot = spt::mat3f(cam.orient).transpose().inversed();
        return translate_mesh(-cam.pos).rotate_mesh(rot);
    }

    LocalScene& transform_to_camera(Camera cam) {
        auto rot = spt::mat3f(cam.orient).transpose().inversed();
        return translate(-cam.pos).rotate(rot);
    }

    LocalScene& transform_to_camera(Camera from, Camera to) {
        auto cl = from.pos;
        auto cg = to.pos;
        auto l = from.orient;
        auto g = to.orient;
        auto inv_g = g.inversed();
        auto a = spt::dot(l, inv_g).transpose();
        auto inv_g_tr = inv_g.transpose();
        auto b = spt::dot(inv_g_tr, cl - cg);
        return linear_transform(a, b);
    }
};

struct GhostScene : public LocalScene {
    Camera cam;

    double local_intensity(spt::vec3f at, spt::vec3f normal) const {
        return light.intensity(at, normal, cam.orient[2]);
    }

    double local_intensity_normalized(spt::vec3f at, spt::vec3f normal) const {
        return light.intensity_normalized(at, normal, -cam.orient[2]);
    }

    GhostScene(LocalScene loc, Camera actual_cam, Camera ghost_cam) {
        light = loc.light;
        mesh = std::move(loc.mesh);

        auto cl = actual_cam.pos;
        auto cg = ghost_cam.pos;
        auto l = actual_cam.orient;
        auto g = ghost_cam.orient;
        auto inv_g = g.inversed();
        auto a = spt::dot(l, inv_g).transpose();
        auto inv_g_tr = inv_g.transpose();
        auto b = spt::dot(inv_g_tr, cl - cg);

        cam = actual_cam;
        cam.orient = spt::dot(inv_g_tr, actual_cam.orient);
        cam.pos += cl - cg;
        linear_transform(a, b);
    }
};
