namespace RayTracer
{
#define M_PI 3.14159265358979323846

constexpr unsigned width = 1366;
constexpr unsigned height = 768;
constexpr unsigned numberOfChannels = 3;

constexpr float fov = M_PI / 3.0;

const math::Vec3f backgroundColor = math::Vec3f(0.2f, 0.7f, 0.87f);
struct Material
{
  Material() : diffuse_color(), albedo(1, 0), specular_exponent() {}
  Material(const math::Vec3f &Color, const math::Vec2f &Albedo, const float &Specular)
    : diffuse_color(Color), albedo(Albedo), specular_exponent(Specular)
  {
  }

  math::Vec3f diffuse_color;
  math::Vec2f albedo;
  float specular_exponent;
};
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
}
}// namespace RayTracer

int main(int argc, char *argv[])
{
  return 0;
}
