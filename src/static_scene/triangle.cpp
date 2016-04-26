#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { 
      //max = Vector3D(1, 1, 1);
      //min = Vector3D(0, 0, 0);
    }

BBox Triangle::get_bbox() const {
  
  // TODO: 
  // compute the bounding box of the triangle
  Vector3D p0 = mesh->positions[v1];
  Vector3D p1 = mesh->positions[v2];
  Vector3D p2 = mesh->positions[v3];

  double minx = std::min(p0.x, std::min(p1.x, p2.x));
  double miny = std::min(p0.y, std::min(p1.y, p2.y));
  double minz = std::min(p0.z, std::min(p1.z, p2.z));

  double maxx = std::max(p0.x, std::max(p1.x, p2.x));
  double maxy = std::max(p0.y, std::max(p1.y, p2.y));
  double maxz = std::max(p0.z, std::max(p1.z, p2.z));

  BBox Tri_bbox = BBox(minx, miny, minz, maxx, maxy, maxz);
  
  return Tri_bbox;
}

unsigned int Triangle::get_MortonCode(const Vector3D& max, const Vector3D& min) const{
  Vector3D p0 = mesh->positions[v1];
  Vector3D p1 = mesh->positions[v2];
  Vector3D p2 = mesh->positions[v3];

  double minx = std::min(p0.x, std::min(p1.x, p2.x));
  double miny = std::min(p0.y, std::min(p1.y, p2.y));
  double minz = std::min(p0.z, std::min(p1.z, p2.z));

  double maxx = std::max(p0.x, std::max(p1.x, p2.x));
  double maxy = std::max(p0.y, std::max(p1.y, p2.y));
  double maxz = std::max(p0.z, std::max(p1.z, p2.z));

  Vector3D bb_min = Vector3D(minx, miny, minz);
  Vector3D bb_max = Vector3D(maxx, maxy, maxz);

  Vector3D center = (bb_min + bb_max) / 2;
  Vector3D unit_center = normalization(center, max, min);

  return morton3D((float)unit_center.x, (float)unit_center.y, (float)unit_center.z);

}

Vector3D Triangle::normalization(Vector3D center, Vector3D max, Vector3D min) const{
  // x ~ (-0.4, 0.4)
  // y ~ (-0.76, 0.69)
  // z ~ (-0.83, 0.65)
  // center.x = (center.x + 0.4)/ 0.8;
  // center.y = (center.y + 0.76) / (0.76 + 0.69);
  // center.z = (center.z + 0.83) / (0.83 + 0.65);
  
  center.x = (center.x - min.x) / (max.x - min.x);
  center.y = (center.y - min.y) / (max.y - min.y);
  center.z = (center.z - min.z) / (max.z - min.z);

  return center;
}

unsigned int Triangle::expandBits(unsigned int v) const{ 
  //Expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
  v = (v * 0x00010001u) & 0xFF0000FFu;
  v = (v * 0x00000101u) & 0x0F00F00Fu;
  v = (v * 0x00000011u) & 0xC30C30C3u;
  v = (v * 0x00000005u) & 0x49249249u;
  return v;
}

unsigned int Triangle::morton3D(float x, float y, float z) const{ 
  //Calculates a 30-bit Morton code for the given 3D point located within the unit cube [0, 1].
  x = min(max(x * 1024.0f, 0.0f), 1023.0f);
  y = min(max(y * 1024.0f, 0.0f), 1023.0f);
  z = min(max(z * 1024.0f, 0.0f), 1023.0f);
  unsigned int xx = expandBits((unsigned int)x);
  unsigned int yy = expandBits((unsigned int)y);
  unsigned int zz = expandBits((unsigned int)z);
  return xx * 4 + yy * 2 + zz;
}

bool Triangle::intersect(const Ray& r) const {
  
  // TODO: implement ray-triangle intersection
  Vector3D p0 = mesh->positions[v1];
  Vector3D p1 = mesh->positions[v2];
  Vector3D p2 = mesh->positions[v3];

  Vector3D e1 = p1-p0;
  Vector3D e2 = p2-p0;
  Vector3D s = r.o-p0;

  Vector3D e1xd = cross(e1, r.d);
  double test = dot(e1xd, e2);
  if(test == 0){
    return false;
  }

  Vector3D sxe2 = cross(s, e2);

  double u = dot(-sxe2, r.d)/test;
  double v = dot(e1xd, s)/test;
  double t = dot(-sxe2, e1)/test;

  if(0 <= u && 0 <= v && u+v <= 1 && t >= 0 && t >= r.min_t && t <= r.max_t){
    return true;
  }
  else
    return false;
}

bool Triangle::intersect(const Ray& r, Intersection *isect) const {
  
  // TODO: 
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  Vector3D p0 = mesh->positions[v1];
  Vector3D p1 = mesh->positions[v2];
  Vector3D p2 = mesh->positions[v3];

  Vector3D e1 = p1-p0;
  Vector3D e2 = p2-p0;
  Vector3D s = r.o-p0;

  Vector3D e1xd = cross(e1, r.d);
  double test = dot(e1xd, e2);
  if(test == 0){
    return false;
  }

  Vector3D sxe2 = cross(s, e2);

  double u = dot(-sxe2, r.d)/test;
  double v = dot(e1xd, s)/test;
  double t = dot(-sxe2, e1)/test;

  if(0 <= u && 0 <= v && u < 1 && v < 1 && u+v <= 1 && t >= r.min_t && t <= r.max_t){
    isect->t = t;
    isect->primitive = this;
    Vector3D nP0 = mesh->normals[v1];
    Vector3D nP1 = mesh->normals[v2];
    Vector3D nP2 = mesh->normals[v3];
    isect->n = u * nP1 + v * nP2 + (1-u-v)*nP0;

    isect->bsdf = mesh->get_bsdf();
    if(dot(isect->n, r.d) < 0){
      return true;
    }else{
      isect->n *= -1;
      return true; 
    }
  }

  return false;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CMU462
