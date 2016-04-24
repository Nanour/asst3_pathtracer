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
  t1 = min(res1, res2);
  t2 = max(res1, res2);
  return true;
}

unsigned int Sphere::get_MortonCode() const{
  cout<<"1"<<endl;
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
