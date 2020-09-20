#include <vector>
#include <fstream>
#include "geometry.h"
#include <cmath>
#include <chrono>
#include <iostream>
#include "stb_image_write.h"

namespace RayTracer
{
#define M_PI 3.14159265358979323846

constexpr unsigned width = 1366;
constexpr unsigned height = 768;
constexpr unsigned numberOfChannels = 3;

constexpr float fov = M_PI / 3.0;

const math::Vec3f backgroundColor = math::Vec3f(0.2f, 0.7f, 0.87f);

void writeIntoJPG(const char *fileName,
  const std::vector<math::Vec3f> &framebuffer,
  const unsigned width,
  const unsigned height,
  const unsigned numberOfChannels)
{
  auto t_start = std::chrono::high_resolution_clock().now();
  std::vector<unsigned char> buffer(width * height * numberOfChannels);

  const unsigned int size = framebuffer.size();

  for (int i = 0, j = 0; i < size; ++i)
  {
    buffer[j++] = static_cast<unsigned char>(255 * std::max(0.0f, std::min(1.0f, framebuffer[i].x)));
    buffer[j++] = static_cast<unsigned char>(255 * std::max(0.0f, std::min(1.0f, framebuffer[i].y)));
    buffer[j++] = static_cast<unsigned char>(255 * std::max(0.0f, std::min(1.0f, framebuffer[i].z)));
  }

  stbi_write_jpg(fileName, width, height, numberOfChannels, buffer.data(), 100);

  auto t_now = std::chrono::high_resolution_clock::now();
  std::cout << "Time spended on -> "
            << " Writing into a file : "
            << std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count() << '\n';
}

struct Material
{
  Material() : diffuse_color(), albedo(1.0f, 0.0f, 0.0f, 0.0f), specular_exponent(), refractive_index(1) {}
  Material(const math::Vec3f &Color, const math::Vec4f &Albedo, const float &Specular, const float &Refractive_index)
    : diffuse_color(Color), albedo(Albedo), specular_exponent(Specular), refractive_index(Refractive_index)
  {
  }

  math::Vec3f diffuse_color;
  math::Vec4f albedo;
  float specular_exponent;
  float refractive_index;
};

math::Vec3f refract(const math::Vec3f &direction, const math::Vec3f &normal, const float &refractive_index)
{
  // cosine of insidence angle
  float cosi = -std::max(-1.0f, std::min(1.0f, direction * normal));
  float etai = 1;
  float etat = refractive_index;

  math::Vec3f n = normal;
  if (cosi < 0)
  {
    cosi = -cosi;
    std::swap(etai, etat);
    n = -normal;
  }

  float eta = etai / etat;
  float k = 1.0f - eta * eta * (1.0f - cosi * cosi);

  return k < 0 ? math::Vec3f(0.0f, 0.0f, 0.0f) : direction * eta + n * (eta * cosi - sqrtf(k));
}

math::Vec3f reflect(const math::Vec3f &direction, const math::Vec3f &normal)
{
  return direction - normal * 2.0f * (direction * normal);
}

struct LightSource
{
  LightSource(const math::Vec3f &Position, const float &Intensity) : position(Position), intensity(Intensity) {}

  math::Vec3f position;
  float intensity;
};

struct Sphere
{
  math::Vec3f center;
  float radius;
  Material material;

  Sphere(const math::Vec3f &Center, const float &Radius, const Material &Material)
    : center(Center), radius(Radius), material(Material)
  {
  }

  bool ray_intersect(const math::Vec3f &ray_origin, const math::Vec3f &ray_direction, float &intersectionDistance) const
  {
    // vector from ray's origin to the center of sphere
    const math::Vec3f L = center - ray_origin;

    const float projectionLength = L * ray_direction;
    const float d2 = L * L - projectionLength * projectionLength;

    const float r2 = radius * radius;
    // whether the ray and sphere intersect
    if (d2 > r2) return false;

    float thc = sqrtf(r2 - d2);
    // distance to the first point on the sphere
    intersectionDistance = projectionLength - thc;

    // distance to the second point on the sphere
    float t1 = projectionLength + thc;

    if (intersectionDistance < 0) intersectionDistance = t1;

    if (intersectionDistance < 0) return false;

    return true;
  }
};

bool scene_intersect(const math::Vec3f &origin,
  const math::Vec3f &direction,
  const std::vector<Sphere> &spheres,
  math::Vec3f &hit,
  math::Vec3f &normal,
  Material &material)
{
  float spheres_dist = std::numeric_limits<float>::max();
  for (const auto &sphere : spheres)
  {
    float dist_i;
    if (sphere.ray_intersect(origin, direction, dist_i) && dist_i < spheres_dist)
    {
      spheres_dist = dist_i;
      hit = origin + direction * dist_i;
      normal = (hit - sphere.center).normalize();
      material = sphere.material;
    }
  }
  float checkerboard_dist = std::numeric_limits<float>::max();
  if (fabs(direction.y) > 1e-3)
  {
    float d = -(origin.y + 4) / direction.y;
    math::Vec3f pt     = origin + direction * d;
    if (d > 0 && fabs(pt.x) < 10 && pt.z < -10 && pt.z > -30 && d < spheres_dist)
    {
      checkerboard_dist = d;
      hit = pt;
      normal = math::Vec3f(0.0f, 1.0f, 0.0f);
      material.diffuse_color = (int(.5 * hit.x + 1000) + int(.5 * hit.z)) & 1 ? math::Vec3f(1, 1, 1) : math::Vec3f(1, .7, .3);
      material.diffuse_color = material.diffuse_color * .3;
    }
  }

  return std::min(spheres_dist, checkerboard_dist) < 1000;
}

// return color that intersects with the ray
math::Vec3f cast_ray(const math::Vec3f &origin,
  const math::Vec3f &direction,
  const std::vector<Sphere> &spheres,
  const std::vector<LightSource> &lightSources,
  size_t depth = 0)
{
  using math::Vec3f;
  // intersection point and normal in that point
  Vec3f point, normal;
  Material material;

  if (depth > 4 || !scene_intersect(origin, direction, spheres, point, normal, material)) return backgroundColor;

  Vec3f reflect_dir = reflect(direction, normal).normalize();
  Vec3f refract_dir = refract(direction, normal, material.refractive_index).normalize();

  Vec3f reflect_origin = reflect_dir * normal < 0 ? point - normal * 1e-3 : point + normal * 1e-3;
  Vec3f refract_origin = refract_dir * normal < 0 ? point - normal * 1e-3 : point + normal * 1e-3;

  Vec3f reflect_color = cast_ray(reflect_origin, reflect_dir, spheres, lightSources, depth + 1);
  Vec3f refract_color = cast_ray(refract_origin, refract_dir, spheres, lightSources, depth + 1);

  float diffuse_light_intensity{}, specular_light_intensity{};
  for (const auto &lightSrc : lightSources)
  {
    Vec3f ld = lightSrc.position - point;
    float light_distance = ld.norm();
    Vec3f light_dir = ld.normalize();

    Vec3f shadow_orig = light_dir * normal < 0 ? point - normal * 1e-3 : point + normal * 1e-3;
    Vec3f shadow_pt, shadow_n;
    Material tmpMaterial;

    if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_n, tmpMaterial)
        && (shadow_pt - shadow_orig).norm() < light_distance)
      continue;

    diffuse_light_intensity += lightSrc.intensity * std::max(0.0f, light_dir * normal);
    specular_light_intensity +=
      powf(std::max(0.0f, reflect(light_dir, normal) * direction), material.specular_exponent) * lightSrc.intensity;
  }
  return material.diffuse_color * diffuse_light_intensity * material.albedo[0]
         + Vec3f(1.0f, 1.0f, 1.0f) * specular_light_intensity * material.albedo[1] + reflect_color * material.albedo[2]
         + refract_color * material.albedo[3];
}

void render(const std::vector<Sphere> &spheres, const std::vector<LightSource> &lightSources)
{
  using math::Vec3f;

  auto t_start = std::chrono::high_resolution_clock::now();

  std::vector<Vec3f> framebuffer(width * height);

  constexpr float widthF = static_cast<float>(width);
  constexpr float heightF = static_cast<float>(height);

  for (size_t j = 0; j < height; ++j)
  {
    for (size_t i = 0; i < width; ++i)
    {
      float x = (2.0f * (i + 0.5) / widthF - 1.0f) * tan(fov / 2.0f) * widthF / heightF;
      float y = -(2.0f * (j + 0.5) / heightF - 1.0f) * tan(fov / 2.0f);
      Vec3f dir = Vec3f(x, y, -1).normalize();

      framebuffer[i + j * width] = cast_ray(Vec3f(0.0f, 0.0f, 0.0f), dir, spheres, lightSources);
    }
  }

  // rendering into a file
  writeIntoJPG("render.jpg", framebuffer, width, height, numberOfChannels);

  auto t_now = std::chrono::high_resolution_clock::now();
  std::cout << "Time spended on -> "
            << " RENDERING : " << std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count()
            << '\n';
}
}// namespace RayTracer

int main(int argc, char *argv[])
{
  using namespace RayTracer;
  using namespace math;

  Material ivory(Vec3f(0.4f, 0.4f, 0.3f), Vec4f(0.6f, 0.3f, 0.1f, 0.0f), 50.0f, 1.0f);
  Material glass(Vec3f(0.6f, 0.7f, 0.8f), Vec4f(0.0f, 0.5f, 0.1f, 0.8f), 125.0f, 1.5f);
  Material red_rubber(Vec3f(0.3f, 0.1f, 0.1f), Vec4f(0.9f, 0.1f, 0.0f, 0.0f), 10.0f, 1.0f);
  Material mirror(Vec3f(1.0f, 1.0f, 1.0f), Vec4f(0.0f, 10.0f, 0.8f, 0.0f), 1425.0f, 1.0f);

  std::vector<Sphere> spheres;
  spheres.push_back(Sphere(Vec3f(-3, 0, -16), 2, ivory));
  spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2, glass));
  spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 3, red_rubber));
  spheres.push_back(Sphere(Vec3f(7, 5, -18), 4, mirror));

  std::vector<LightSource> lightSources;
  lightSources.push_back(LightSource(Vec3f(-20.0f, 20.0f, 20.0f), 1.5f));
  lightSources.push_back(LightSource(Vec3f(30.0f, 50.0f, -25.0f), 1.8f));
  lightSources.push_back(LightSource(Vec3f(30.0f, 20.0f, 30.0f), 1.7f));

  render(spheres, lightSources);
  return 0;
}
