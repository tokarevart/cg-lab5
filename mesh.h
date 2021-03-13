#pragma once

#include <vector>
#include <memory>
#include <iostream>
#include "sptops.h"
#include "tiny_obj_loader.h"

struct Vert {
    std::size_t pos;
    std::size_t normal;
};

struct Face {
    std::array<Vert, 3> verts;

    std::array<Vert, 2> edge(std::size_t i) const {
        if (i < 2) {
            return {verts[i], verts[i + 1]};
        } else {
            return {verts[i], verts[0]};
        }
    }

    bool contains_vert(std::size_t vert_pos_id) const {
        if (verts[0].pos == vert_pos_id ||
            verts[1].pos == vert_pos_id ||
            verts[2].pos == vert_pos_id) {
            return true;
        } else {
            return false;
        }
    }
};

struct UniqueMesh {
    std::vector<spt::vec3f> verts;
    std::vector<spt::vec3f> normals;
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

    spt::vec3f face_normal(const Face& face) const {
        return (normals[face.verts[0].normal]
                + normals[face.verts[1].normal]
                + normals[face.verts[2].normal]).normalize();
//        auto edgevec0 = verts[face.verts[1].pos] - verts[face.verts[0].pos];
//        auto edgevec1 = verts[face.verts[2].pos] - verts[face.verts[1].pos];
//        return spt::cross(edgevec0, edgevec1).normalize();
    }

    spt::vec3f face_center(const Face& face) const {
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

    static UniqueMesh load_obj(std::string filename) {
        std::vector<spt::vec3f> verts;
        std::vector<spt::vec3f> normals;
        std::vector<Face> faces;

        tinyobj::ObjReaderConfig reader_config;
        reader_config.triangulate = true;
        tinyobj::ObjReader reader;
        if (!reader.ParseFromFile(filename)) {
            if (!reader.Error().empty()) {
                std::cerr << "TinyObjReader: " << reader.Error();
            }
        }

        auto& attrib = reader.GetAttrib();
        std::size_t num_vertices = attrib.vertices.size() / 3;
        for (std::size_t i = 0; i < num_vertices; ++i) {
            float vx = attrib.vertices[3 * i + 0];
            float vy = attrib.vertices[3 * i + 1];
            float vz = attrib.vertices[3 * i + 2];
            verts.emplace_back(std::array{vx, vy, vz});
        }
        std::size_t num_normals = attrib.normals.size() / 3;
        for (std::size_t i = 0; i < num_normals; ++i) {
            float vx = attrib.normals[3 * i + 0];
            float vy = attrib.normals[3 * i + 1];
            float vz = attrib.normals[3 * i + 2];
            normals.emplace_back(std::array{vx, vy, vz});
        }

        auto& shapes = reader.GetShapes();
        for (std::size_t s = 0; s < shapes.size(); s++) {
            std::size_t index_offset = 0;
            for (std::size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
                std::size_t fv = shapes[s].mesh.num_face_vertices[f];

                if (fv == 3) {
                    std::array<tinyobj::index_t, 3> inds{
                        shapes[s].mesh.indices[index_offset + 0],
                        shapes[s].mesh.indices[index_offset + 1],
                        shapes[s].mesh.indices[index_offset + 2]
                    };
                    Face face;
                    for (std::size_t i = 0; i < 3; ++i) {
                        Vert vert;
                        vert.pos = inds[i].vertex_index;
                        vert.normal = inds[i].normal_index;
                        face.verts[i] = vert;
                    }
                    faces.push_back(face);
                }

                index_offset += fv;
            }
        }

        return UniqueMesh(std::move(verts), std::move(normals), std::move(faces));
    }

    UniqueMesh() {}
    UniqueMesh(std::vector<spt::vec3f> verts,
               std::vector<spt::vec3f> normals,
               std::vector<Face> faces)
        : verts(std::move(verts)),
          normals(std::move(normals)),
          faces(std::move(faces)) {}
};
