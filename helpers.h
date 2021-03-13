#pragma once

#include "sptalgs.h"
#include "mesh.h"
#include "view.h"

template <typename T>
QList<typename T::value_type> qlist_from_vec(T vec) {
    return QList(vec.x.begin(), vec.x.end());
}

struct Inter {
    spt::vec3f at;
    spt::vec3f normal;
    std::size_t face_id;
};

static std::optional<Inter> ray_mesh_nearest_intersection(
    spt::vec3f origin, spt::vec3f dir, const UniqueMesh& mesh) {

    spt::vec3f normal;
    spt::vec3f at;
    std::size_t face_id;
    float max_z = std::numeric_limits<float>::lowest();
    bool no_inters = true;
    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        auto face = mesh.faces[i];
        auto ointer = spt::ray_intersect_thick_triangle(
            origin, dir,
            mesh.verts[face.verts[0].pos],
            mesh.verts[face.verts[1].pos],
            mesh.verts[face.verts[2].pos]);
        if (!ointer.has_value()) {
            continue;
        }

        auto inter = ointer.value();
        if (inter[2] > max_z) {
            max_z = inter[2];
            at = inter;
            normal = mesh.face_normal(face);
            no_inters = false;
            face_id = i;
        }
    }

    if (no_inters) {
        return std::nullopt;
    } else {
        return Inter{at, normal, face_id};
    }
}

static std::optional<Inter> ray_mesh_intersection(
    spt::vec3f origin, spt::vec3f dir, const UniqueMesh& mesh) {

    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        auto face = mesh.faces[i];
        auto ointer = spt::ray_intersect_thick_triangle(
            origin, dir,
            mesh.verts[face.verts[0].pos],
            mesh.verts[face.verts[1].pos],
            mesh.verts[face.verts[2].pos]);
        if (ointer.has_value()) {
            return Inter{
                ointer.value(),
                mesh.face_normal(face),
                i
            };
        }
    }
    return std::nullopt;
}

static std::optional<Inter> vertray_mesh_nearest_intersection(
    std::size_t origin_vert, spt::vec3f dir, const UniqueMesh& mesh) {

    spt::vec3f normal;
    spt::vec3f at;
    std::size_t face_id;
    float max_z = std::numeric_limits<float>::lowest();
    bool no_inters = true;
    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        auto face = mesh.faces[i];
        if (face.contains_vert(origin_vert)) {
            continue;
        }

        auto origin = mesh.verts[origin_vert];
        auto ointer = spt::ray_intersect_thick_triangle(
            origin, dir,
            mesh.verts[face.verts[0].pos],
            mesh.verts[face.verts[1].pos],
            mesh.verts[face.verts[2].pos]
        );
        if (!ointer.has_value()) {
            continue;
        }

        auto inter = ointer.value();
        if (inter[2] > max_z) {
            max_z = inter[2];
            at = inter;
            normal = mesh.face_normal(face);
            no_inters = false;
            face_id = i;
        }
    }

    if (no_inters) {
        return std::nullopt;
    } else {
        return Inter{at, normal, face_id};
    }
}

static bool vertray_intersect_mesh(
    std::size_t origin_vert, spt::vec3f dir, const UniqueMesh& mesh) {

    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        auto face = mesh.faces[i];
        if (face.contains_vert(origin_vert)) {
            continue;
        }

        auto origin = mesh.verts[origin_vert];
        auto ointer = spt::ray_intersect_thick_triangle(
            origin, dir,
            mesh.verts[face.verts[0].pos],
            mesh.verts[face.verts[1].pos],
            mesh.verts[face.verts[2].pos]);
        if (ointer.has_value()) {
            return true;
        }
    }
    return false;
}

static bool faceray_intersect_mesh(
    std::size_t rayface_id, spt::vec3f dir, const ProjectedMesh& mesh, const std::vector<float>& zs) {

    auto& rf = mesh.faces[rayface_id];
    auto& rf_vs = rf.verts;
    float oriz = (zs[rf_vs[0].pos] + zs[rf_vs[1].pos] + zs[rf_vs[2].pos]) / 3.0;
    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        if (i == rayface_id) {
            continue;
        }
        auto face = mesh.faces[i];
        auto ori_2d = mesh.face_center(rf);
        auto v0_2d = mesh.verts[face.verts[0].pos];
        auto v1_2d = mesh.verts[face.verts[1].pos];
        auto v2_2d = mesh.verts[face.verts[2].pos];

        spt::vec3f origin({ori_2d[0], ori_2d[1], oriz});
        spt::vec3f v0({v0_2d[0], v0_2d[1], zs[face.verts[0].pos]});
        spt::vec3f v1({v1_2d[0], v1_2d[1], zs[face.verts[1].pos]});
        spt::vec3f v2({v2_2d[0], v2_2d[1], zs[face.verts[2].pos]});
        auto ointer = spt::ray_intersect_thick_triangle(origin, dir, v0, v1, v2);
        if (ointer.has_value()) {
            return true;
        }
    }
    return false;
}

static bool vertray_intersect_mesh(
    std::size_t origin_vert, spt::vec3f dir, const ProjectedMesh& mesh, const std::vector<float>& zs) {

    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        auto face = mesh.faces[i];
        if (face.contains_vert(origin_vert)) {
            continue;
        }

        auto ori_2d = mesh.verts[origin_vert];
        auto v0_2d = mesh.verts[face.verts[0].pos];
        auto v1_2d = mesh.verts[face.verts[1].pos];
        auto v2_2d = mesh.verts[face.verts[2].pos];
        spt::vec3f origin({ori_2d[0], ori_2d[1], zs[origin_vert]});
        spt::vec3f v0({v0_2d[0], v0_2d[1], zs[face.verts[0].pos]});
        spt::vec3f v1({v1_2d[0], v1_2d[1], zs[face.verts[1].pos]});
        spt::vec3f v2({v2_2d[0], v2_2d[1], zs[face.verts[2].pos]});
        auto ointer = spt::ray_intersect_thick_triangle(
            origin, dir, v0, v1, v2);
        if (ointer.has_value()) {
            return true;
        }
    }
    return false;
}

static std::array<int, 2> min_max_image_coor(
    const Viewport& viewport, const Projection* proj, const ProjectedMesh& mesh, std::size_t c) {

    auto minmax = mesh.min_max_coor(c);
    int end = viewport.width();
    if (c == 1) {
        std::swap(minmax[0], minmax[1]);
        end = viewport.height();
    }
    spt::vec2f wmin;
    wmin[c] = minmax[0];
    spt::vec2f wmax;
    wmax[c] = minmax[1];
    int min_c = std::max(
        0,
        std::min(static_cast<int>(proj->to_viewport(wmin)[c]), end));
    int max_c = std::min(
        std::max(static_cast<int>(proj->to_viewport(wmax)[c] + 1), 0),
        end);
    return { min_c, max_c };
}

static std::optional<spt::vec2f> segment_horizline_intersection(
    const std::array<spt::vec2f, 2> pts, float y, float& t) {

    float p0y = pts[0][1];
    float p1y = pts[1][1];
    float maxabsy = std::max(std::abs(p0y), std::abs(p1y));
    float scaled_eps = std::numeric_limits<float>::epsilon() * maxabsy;
    if (std::abs(p0y - p1y) <= scaled_eps) {
        return std::nullopt;
    }
    t = static_cast<float>(y - p0y) / (p1y - p0y);
    if (t < 0.0 || t > 1.0) {
        return std::nullopt;
    } else {
        return pts[0] + t * (pts[1] - pts[0]);
    }
}

static std::optional<spt::vec2f> segment_vertline_intersection(
    const std::array<spt::vec2f, 2> pts, float x, float& t) {

    float p0x = pts[0][0];
    float p1x = pts[1][0];
    float maxabsx = std::max(std::abs(p0x), std::abs(p1x));
    float scaled_eps = std::numeric_limits<float>::epsilon() * maxabsx;
    if (std::abs(p0x - p1x) <= scaled_eps) {
        return std::nullopt;
    }
    t = static_cast<float>(x - p0x) / (p1x - p0x);
    if (t < 0.0 || t > 1.0) {
        return std::nullopt;
    } else {
        return pts[0] + t * (pts[1] - pts[0]);
    }
}

static std::optional<spt::vec2f> segment_ortholine_intersection(
    const std::array<spt::vec2f, 2> pts, std::size_t axis, float c, float& t) {

    float p0c = pts[0][axis];
    float p1c = pts[1][axis];
    float maxabsx = std::max(std::abs(p0c), std::abs(p1c));
    float scaled_eps = std::numeric_limits<float>::epsilon() * maxabsx;
    if (std::abs(p0c - p1c) <= scaled_eps) {
        return std::nullopt;
    }
    t = static_cast<float>(c - p0c) / (p1c - p0c);
    if (t < 0.0 || t > 1.0) {
        return std::nullopt;
    } else {
        return pts[0] + t * (pts[1] - pts[0]);
    }
}

static std::optional<spt::vec2f> segment_horizline_intersection(
    const std::array<spt::vec2f, 2> pts, float y) {

    float t;
    return segment_horizline_intersection(pts, y, t);
}

static std::optional<spt::vec2f> segment_vertline_intersection(
    const std::array<spt::vec2f, 2> pts, float x) {

    float t;
    return segment_vertline_intersection(pts, x, t);
}

static std::optional<spt::vec2f> segment_ortholine_intersection(
    const std::array<spt::vec2f, 2> pts, std::size_t axis, float c) {

    float t;
    return segment_ortholine_intersection(pts, axis, c, t);
}

struct EdgeInter {
    spt::vec2f p;
    std::array<Vert, 2> edge;
    std::array<std::size_t, 2> face_vids;
};

static std::optional<std::array<EdgeInter, 2>> triangle_horizline_intersections(
    const ProjectedMesh& mesh, const Face& face, float y, std::array<float, 2>& ts) {

    std::array<spt::vec2f, 2> pres;
    std::array<std::array<Vert, 2>, 2> eres;
    std::array<std::array<std::size_t, 2>, 2> vres;
    std::size_t res_i = 0;
    std::size_t previd = 2;
    Vert prev = face.verts[previd];
    auto prevpos = mesh.verts[prev.pos];
    for (std::size_t i = 0; i < face.verts.size(); ++i) {
        std::size_t curid = i;
        Vert cur = face.verts[curid];
        auto curpos = mesh.verts[cur.pos];
        float t;
        auto ointer = segment_horizline_intersection({prevpos, curpos}, y, t);
        if (ointer.has_value()) {
            if (curpos[1] == y) {
                Vert next = face.verts[0];
                if (i < 2) {
                    next = face.verts[i + 1];
                }
                auto nextpos = mesh.verts[next.pos];
                if ((curpos[1] - prevpos[1]) * (nextpos[1] - curpos[1]) < 0) {
                    pres[res_i] = ointer.value();
                    eres[res_i] = { prev, cur };
                    vres[res_i] = { previd, curid };
                    ts[res_i] = t;
                    if (++res_i == 2) {
                        break;
                    }
                }
            } else {
                pres[res_i] = ointer.value();
                eres[res_i] = { prev, cur };
                vres[res_i] = { previd, curid };
                ts[res_i] = t;
                if (++res_i == 2) {
                    break;
                }
            }
        }
        previd = curid;
        prev = cur;
        prevpos = curpos;
    }
    if (res_i < 2) {
        return std::nullopt;
    }

    if (pres[0][0] > pres[1][0]) {
        std::swap(pres[0], pres[1]);
        std::swap(eres[0], eres[1]);
        std::swap(vres[0], vres[1]);
        std::swap(ts[0], ts[1]);
    }
    return std::array{
        EdgeInter{ pres[0], eres[0], vres[0] },
        EdgeInter{ pres[1], eres[1], vres[1] }
    };
}

static std::optional<std::array<EdgeInter, 2>> triangle_vertline_intersections(
    const ProjectedMesh& mesh, const Face& face, float x, std::array<float, 2>& ts) {

    std::array<spt::vec2f, 2> pres;
    std::array<std::array<Vert, 2>, 2> eres;
    std::array<std::array<std::size_t, 2>, 2> vres;
    std::size_t res_i = 0;
    std::size_t previd = 2;
    Vert prev = face.verts[previd];
    auto prevpos = mesh.verts[prev.pos];
    for (std::size_t i = 0; i < face.verts.size(); ++i) {
        std::size_t curid = i;
        Vert cur = face.verts[curid];
        auto curpos = mesh.verts[cur.pos];
        float t;
        auto ointer = segment_vertline_intersection({prevpos, curpos}, x, t);
        if (ointer.has_value()) {
            if (curpos[0] == x) {
                Vert next = face.verts[0];
                if (i < 2) {
                    next = face.verts[i + 1];
                }
                auto nextpos = mesh.verts[next.pos];
                if ((curpos[0] - prevpos[0]) * (nextpos[0] - curpos[0]) < 0) {
                    pres[res_i] = ointer.value();
                    eres[res_i] = { prev, cur };
                    vres[res_i] = { previd, curid };
                    ts[res_i] = t;
                    if (++res_i == 2) {
                        break;
                    }
                }
            } else {
                pres[res_i] = ointer.value();
                eres[res_i] = { prev, cur };
                vres[res_i] = { previd, curid };
                ts[res_i] = t;
                if (++res_i == 2) {
                    break;
                }
            }
        }
        previd = curid;
        prev = cur;
        prevpos = curpos;
    }
    if (res_i < 2) {
        return std::nullopt;
    }

    if (pres[0][1] > pres[1][1]) {
        std::swap(pres[0], pres[1]);
        std::swap(eres[0], eres[1]);
        std::swap(vres[0], vres[1]);
        std::swap(ts[0], ts[1]);
    }
    return std::array{
        EdgeInter{ pres[0], eres[0], vres[0] },
        EdgeInter{ pres[1], eres[1], vres[1] }
    };
}

static std::optional<std::array<EdgeInter, 2>> triangle_ortholine_intersections(
    const ProjectedMesh& mesh, const Face& face, std::size_t axis, float c, std::array<float, 2>& ts) {

    std::array<spt::vec2f, 2> pres;
    std::array<std::array<Vert, 2>, 2> eres;
    std::array<std::array<std::size_t, 2>, 2> vres;
    std::size_t res_i = 0;
    std::size_t previd = 2;
    Vert prev = face.verts[previd];
    auto prevpos = mesh.verts[prev.pos];
    for (std::size_t i = 0; i < face.verts.size(); ++i) {
        std::size_t curid = i;
        Vert cur = face.verts[curid];
        auto curpos = mesh.verts[cur.pos];
        float t;
        auto ointer = segment_ortholine_intersection({prevpos, curpos}, axis, c, t);
        if (ointer.has_value()) {
            if (curpos[axis] == c) {
                Vert next = face.verts[0];
                if (i < 2) {
                    next = face.verts[i + 1];
                }
                auto nextpos = mesh.verts[next.pos];
                if ((curpos[axis] - prevpos[axis]) * (nextpos[axis] - curpos[axis]) < 0) {
                    pres[res_i] = ointer.value();
                    eres[res_i] = { prev, cur };
                    vres[res_i] = { previd, curid };
                    ts[res_i] = t;
                    if (++res_i == 2) {
                        break;
                    }
                }
            } else {
                pres[res_i] = ointer.value();
                eres[res_i] = { prev, cur };
                vres[res_i] = { previd, curid };
                ts[res_i] = t;
                if (++res_i == 2) {
                    break;
                }
            }
        }
        previd = curid;
        prev = cur;
        prevpos = curpos;
    }
    if (res_i < 2) {
        return std::nullopt;
    }
    std::size_t other = 0;
    if (axis == 0) {
        other = 1;
    }
    if (pres[0][other] > pres[1][other]) {
        std::swap(pres[0], pres[1]);
        std::swap(eres[0], eres[1]);
        std::swap(vres[0], vres[1]);
        std::swap(ts[0], ts[1]);
    }
    return std::array{
        EdgeInter{ pres[0], eres[0], vres[0] },
        EdgeInter{ pres[1], eres[1], vres[1] }
    };
}

static std::optional<std::array<EdgeInter, 2>> triangle_horizline_intersections(
    const ProjectedMesh& mesh, const Face& face, float y) {

    std::array<float, 2> ts;
    return triangle_horizline_intersections(mesh, face, y, ts);
}

static std::optional<std::array<EdgeInter, 2>> triangle_vertline_intersections(
    const ProjectedMesh& mesh, const Face& face, float x) {

    std::array<float, 2> ts;
    return triangle_vertline_intersections(mesh, face, x, ts);
}

static std::optional<std::array<EdgeInter, 2>> triangle_ortholine_intersections(
    const ProjectedMesh& mesh, const Face& face, std::size_t axis, float c) {

    std::array<float, 2> ts;
    return triangle_ortholine_intersections(mesh, face, axis, c, ts);
}

template <typename T>
T interpolate(const T& v0, const T& v1, float t) {
    return v0 + t * (v1 - v0);
}

static std::array<int, 2> image_c_range(
    const Projection* proj, const std::array<spt::vec2f, 2>& pts, std::size_t c) {

    int t0 = static_cast<int>(proj->to_viewport(pts[0])[c]);
    int t1 = static_cast<int>(proj->to_viewport(pts[1])[c]);
    if (t0 > t1) {
        std::swap(t0, t1);
    }
    return { t0, t1 + 1 };
}

static std::array<int, 2> to_image_c_draw_range(
    const Viewport& viewport, std::array<int, 2> crange, std::size_t c) {

    int end;
    if (c == 0) {
        end = viewport.width();
    } else {
        end = viewport.height();
    }
    return {
        std::max(std::min(crange[0], end), 0),
        std::min(std::max(crange[1], 0), end)
    };
}

static std::array<int, 2> image_c_draw_range(
    const Viewport& viewport, const Projection* proj,
    const std::array<spt::vec2f, 2>& pts, std::size_t c) {

    auto crange = image_c_range(proj, pts, c);
    return to_image_c_draw_range(viewport, crange, c);
}
