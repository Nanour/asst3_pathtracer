#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  Vector3D ray_o = r.o;
  Vector3D d = r.d;

  double a = dot(d, d);
  double b = dot(2.0*(ray_o - o), d);
  double c = dot((ray_o - o), (ray_o - o)) - r2;
  double check = pow(b, 2) - 4.0*a*c;

  if (check < 0) {
    return false;
  }
  double res1 = (-b + sqrt(check))/(2.0*a);
  double res2 = (-b - sqrt(check))/(2.0*a);
  t1 = std::min(res1, res2);
  t2 = std::max(res1, res2);
  return true;
}

unsigned int Sphere::get_MortonCode(const Vector3D& max, const Vector3D& min) const{
  Vector3D unit_center = normalization(o, max, min);
  return morton3D((float)unit_center.x, (float)unit_center.y, (float)unit_center.z);
}

Vector3D Sphere::normalization(Vector3D center, Vector3D max, Vector3D min) const{
  center.x = (center.x - min.x) / (max.x - min.x);
  center.y = (center.y - min.y) / (max.y - min.y);
  center.z = (center.z - min.z) / (max.z - min.z);

  return center;
}

unsigned int Sphere::expandBits(unsigned int v) const{ 
  //Expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
  v = (v * 0x00010001u) & 0xFF0000FFu;
  v = (v * 0x00000101u) & 0x0F00F00Fu;
  v = (v * 0x00000011u) & 0xC30C30C3u;
  v = (v * 0x00000005u) & 0x49249249u;
  return v;
}

unsigned int Sphere::morton3D(float x, float y, float z) const{ 
  //Calculates a 30-bit Morton code for the given 3D point located within the unit cube [0, 1].
  x = min(max(x * 1024.0f, 0.0f), 1023.0f);
  y = min(max(y * 1024.0f, 0.0f), 1023.0f);
  z = min(max(z * 1024.0f, 0.0f), 1023.0f);
  unsigned int xx = expandBits((unsigned int)x);
  unsigned int yy = expandBits((unsigned int)y);
  unsigned int zz = expandBits((unsigned int)z);
  return xx * 4 + yy * 2 + zz;
}

bool Sphere::intersect(const Ray& r) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1, t2;
  bool check = test(r, t1, t2);
  if (!check || t2 < r.min_t || t1 > r.max_t || (t1 < 0 && t2 > r.max_t)) {
    return false;
  }
  if (t1 <= 0) {
    t1 = t2;
  }
  r.max_t = t1;

  return true;

}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1 = 0.0;
  double t2 = 0.0;
  bool check = test(r, t1, t2);

  if (!check || t2 < r.min_t || t1 > r.max_t) {
    return false;
  }

  if (t1 <= 0) {
    if (t2 > r.max_t) {
      return false;
    }
    t1 = t2;
  }

  r.max_t = t1;

  Vector3D p = r.o + r.d*t1;
  Vector3D n = (p - o);
  n.normalize();

  i->t = t1;
  i->primitive = this;
  i->bsdf = get_bsdf();
  i->n = n;
  
  return true;
}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CMU462
