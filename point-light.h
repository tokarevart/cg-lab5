#pragma once

#include "sptops.h"

struct AmbientLight {
    float ia() const {
        return m_ia;
    }

    float ka() const {
        return m_ka;
    }

    void set_params(float ia, float ka) {
        m_ia = ia;
        m_ka = ka;
        m_ia_ka = ia * ka;
    }

    float intensity() const {
        return m_ia_ka;
    }

    float max_intensity() const {
        return m_ia_ka;
    }

    AmbientLight() {}
    AmbientLight(float ia, float ka)
        : m_ia(ia), m_ka(ka) {
        m_ia_ka = ia * ka;
    }

  private:
    float m_ia, m_ka;
    float m_ia_ka;
};

struct DiffuseLight {
    float il() const {
        return m_il;
    }

    float kd() const {
        return m_kd;
    }

    void set_params(float il, float kd) {
        m_il = il;
        m_kd = kd;
        m_il_kd = il * kd;
    }

    float intensity(spt::vec3f light, spt::vec3f normal) const {
        return m_il_kd * std::max(0.0f, -spt::dot(normal, light));
    }

    float max_intensity() const {
        return m_il_kd;
    }

    DiffuseLight() {}
    DiffuseLight(float il, float kd)
        : m_il(il), m_kd(kd) {
        m_il_kd = il * kd;
    }

  private:
    float m_il, m_kd;
    float m_il_kd;
};

struct PhongLight {
    float il() const {
        return m_il;
    }

    float ks() const {
        return m_ks;
    }

    float n() const {
        return m_n;
    }

    void set_params(float il, float ks, float n) {
        m_il = il;
        m_ks = ks;
        m_n = n;
        m_il_ks = il * ks;
    }

    float intensity(spt::vec3f light, spt::vec3f normal, spt::vec3f camdir) const {
        auto proj = spt::dot(light, normal) * normal;
        auto refl = light - proj - proj;
        return m_il_ks * std::pow(std::max(0.0f, -spt::dot(refl, camdir)), m_n);
    }

    float local_intensity(spt::vec3f light, spt::vec3f normal) const {
        auto projz = spt::dot(light, normal) * normal[2];
        auto reflz = light[2] - projz - projz;
        return m_il_ks * std::pow(std::max(0.0f, reflz), m_n);
    }

    float max_intensity() const {
        return m_il_ks;
    }

    PhongLight() {}
    PhongLight(float il, float ks, float n)
        : m_il(il), m_ks(ks), m_n(n) {
        m_il_ks = il * ks;
    }

  private:
    float m_il, m_ks, m_n;
    float m_il_ks;
};

struct SimpleIllum {
    AmbientLight ambient() const {
        return m_ambient;
    }

    DiffuseLight diffuse() const {
        return m_diffuse;
    }

    PhongLight phong() const {
        return m_phong;
    }

    void set_params(AmbientLight ambient, DiffuseLight diffuse, PhongLight phong) {
        m_ambient = ambient;
        m_diffuse = diffuse;
        m_phong = phong;
        m_inv_max_intensity = 1.0f / max_intensity();
    }

    float intensity(spt::vec3f light, spt::vec3f normal, spt::vec3f camdir) const {
        return m_ambient.intensity()
               + m_diffuse.intensity(light, normal)
               + m_phong.intensity(light, normal, camdir);
    }

    float intensity_normalized(spt::vec3f light, spt::vec3f normal, spt::vec3f camdir) const {
        return intensity(light, normal, camdir) * m_inv_max_intensity;
    }

    float local_intensity(spt::vec3f light, spt::vec3f normal) const {
        return m_ambient.intensity()
               + m_diffuse.intensity(light, normal)
               + m_phong.local_intensity(light, normal);
    }

    float local_intensity_normalized(spt::vec3f light, spt::vec3f normal) const {
        return local_intensity(light, normal) * m_inv_max_intensity;
    }

    float max_intensity() const {
        return m_ambient.max_intensity()
               + m_diffuse.max_intensity()
               + m_phong.max_intensity();
    }

    SimpleIllum() {}
    SimpleIllum(AmbientLight ambient, DiffuseLight diffuse, PhongLight phong)
        : m_ambient(ambient), m_diffuse(diffuse), m_phong(phong) {
        m_inv_max_intensity = 1.0f / max_intensity();
    }

  private:
    AmbientLight m_ambient;
    DiffuseLight m_diffuse;
    PhongLight m_phong;
    float m_inv_max_intensity;
};

struct PointLight {
    spt::vec3f pos;
    SimpleIllum illum;

    float intensity(spt::vec3f at, spt::vec3f normal, spt::vec3f camdir) const {
        return illum.intensity((at - this->pos).normalize(), normal, camdir);
    }

    float intensity_normalized(spt::vec3f at, spt::vec3f normal, spt::vec3f camdir) const {
        return illum.intensity_normalized((at - this->pos).normalize(), normal, camdir);
    }

    float local_intensity(spt::vec3f at, spt::vec3f normal) const {
        return illum.local_intensity((at - this->pos).normalize(), normal);
    }

    float local_intensity_normalized(spt::vec3f at, spt::vec3f normal) const {
        return illum.local_intensity_normalized((at - this->pos).normalize(), normal);
    }

    PointLight() {}
    PointLight(spt::vec3f pos, SimpleIllum illum)
        : pos(pos), illum(illum) {}
};
