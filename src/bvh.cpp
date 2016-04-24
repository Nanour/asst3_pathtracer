#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CMU462 { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  this->primitives = _primitives;

  // TODO:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bb;
  double minx = INF_D;
  double maxx = 0;
  for (size_t i = 0; i < primitives.size(); ++i) {
    bb.expand(primitives[i]->get_bbox());
    double centerx =  primitives[i]->get_bbox().centroid().z;
    maxx = std::max(maxx, centerx);
    minx = std::min(minx, centerx);
    //cout << primitives[i]->get_bbox().centroid() << endl;
    //cout << primitives[i]->get_MortonCode() << endl;
  }
  cout << maxx << endl;
  cout << minx << endl;

  //cout << primitives.size() << endl;
  std::sort(primitives.begin(),primitives.end(),mCodeCmp());
  root = new BVHNode(bb, 0, primitives.size());

  std::stack<BVHNode*> stk;
  stk.push(root);
  while(stk.size() > 0){
    BVHNode* currNode = stk.top();
    stk.pop();
    bool hasChild = generateHierarchy(currNode, max_leaf_size);
    if(hasChild){
      stk.push(currNode->l);
      stk.push(currNode->r);
    }
  }
  this->root = root;
  this->primitives = primitives;

}

bool BVHAccel::generateHierarchy(BVHNode* currNode, size_t max_leaf_size){
  if(currNode->range <= max_leaf_size || currNode->bb.empty()){
    currNode->l = NULL;
    currNode->r = NULL;
    return false;
  }

  // std::vector<Primitive *> currPrimList;
  // for(size_t i = currNode->start; i < (currNode->start + currNode->range); i++){
  //   currPrimList.push_back(primitives[i]);
  // }

  int split = findSplit(currNode); 
  cout << currNode->start+split << endl;
  
  BBox leftbb, rightbb;
  size_t left = 0, right = 0;

  for(size_t i = currNode->start; i < currNode->start+split; i++){
    Primitive* prim = primitives[i];

    leftbb.expand(prim->get_bbox());
    left++;
  }

  for(size_t i = currNode->start+split; i < currNode->start+currNode->range; i++){
    Primitive* prim = primitives[i];

    rightbb.expand(prim->get_bbox());
    right++;
  }

  if(currNode->range == left || currNode->range == right){
      return false;
    }
  else{
    currNode->l = new BVHNode(leftbb, currNode->start, left);
    currNode->r = new BVHNode(rightbb, currNode->start+left, right);
    return true;
  }

}

int BVHAccel::findSplit(BVHNode* currNode){

  std::vector<Primitive *> currPrimList;
  for(size_t i = currNode->start; i < (currNode->start + currNode->range); i++){
    currPrimList.push_back(primitives[i]);
  }

  int first = currNode->start;
  int last = currNode->start+currNode->range-1;
  // Identical Morton codes => split the range in the middle.
  cout << "first: " << currNode->start << endl;
  unsigned int firstCode = primitives[first]->get_MortonCode();
  cout << "firstCode: " << firstCode << endl;

  cout << "last: " << currNode->start+currNode->range-1 << endl;
  unsigned int lastCode = primitives[last]->get_MortonCode();
  cout << "lastCode: " << lastCode << endl;

  if (firstCode == lastCode)
    //return (first + last) >> 1;
    return first;

  // Calculate the number of highest bits that are the same
  // for all objects, using the count-leading-zeros intrinsic.

  int commonPrefix = CLZ1(firstCode ^ lastCode);

  // Use binary search to find where the next bit differs.
  // Specifically, we are looking for the highest object that
  // shares more than commonPrefix bits with the first one.

  int split = first; // initial guess
  int step = last - first;

  do{
    step = (step + 1) >> 1; // exponential decrease
    int newSplit = split + step; // proposed new position

    if(newSplit < last){
      unsigned int splitCode = primitives[newSplit]->get_MortonCode();
      int splitPrefix = CLZ1(firstCode ^ splitCode);
      if(splitPrefix > commonPrefix)
        split = newSplit; // accept proposal
    }
  }
  while (step > 1);

  return split;

}

uint32_t BVHAccel::CLZ1(uint32_t x) {
    static uint8_t const clz_lkup[] = {
        32U, 31U, 30U, 30U, 29U, 29U, 29U, 29U,
        28U, 28U, 28U, 28U, 28U, 28U, 28U, 28U,
        27U, 27U, 27U, 27U, 27U, 27U, 27U, 27U,
        27U, 27U, 27U, 27U, 27U, 27U, 27U, 27U,
        26U, 26U, 26U, 26U, 26U, 26U, 26U, 26U,
        26U, 26U, 26U, 26U, 26U, 26U, 26U, 26U,
        26U, 26U, 26U, 26U, 26U, 26U, 26U, 26U,
        26U, 26U, 26U, 26U, 26U, 26U, 26U, 26U,
        25U, 25U, 25U, 25U, 25U, 25U, 25U, 25U,
        25U, 25U, 25U, 25U, 25U, 25U, 25U, 25U,
        25U, 25U, 25U, 25U, 25U, 25U, 25U, 25U,
        25U, 25U, 25U, 25U, 25U, 25U, 25U, 25U,
        25U, 25U, 25U, 25U, 25U, 25U, 25U, 25U,
        25U, 25U, 25U, 25U, 25U, 25U, 25U, 25U,
        25U, 25U, 25U, 25U, 25U, 25U, 25U, 25U,
        25U, 25U, 25U, 25U, 25U, 25U, 25U, 25U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U,
        24U, 24U, 24U, 24U, 24U, 24U, 24U, 24U
    };
    uint32_t n;
    if (x >= (1U << 16)) {
        if (x >= (1U << 24)) {
            n = 24U;
        }
        else {
            n = 16U;
        }
    }
    else {
        if (x >= (1U << 8)) {
            n = 8U;
        }
        else {
            n = 0U;
        }
    }
    return (uint32_t)clz_lkup[x >> n] - n;
}


unsigned int BVHAccel::expandBits(unsigned int v) const{ 
  //Expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
  v = (v * 0x00010001u) & 0xFF0000FFu;
  v = (v * 0x00000101u) & 0x0F00F00Fu;
  v = (v * 0x00000011u) & 0xC30C30C3u;
  v = (v * 0x00000005u) & 0x49249249u;
  return v;
}

unsigned int BVHAccel::morton3D(float x, float y, float z) const{ 
  //Calculates a 30-bit Morton code for the given 3D point located within the unit cube [0, 1].
  x = min(max(x * 1024.0f, 0.0f), 1023.0f);
  y = min(max(y * 1024.0f, 0.0f), 1023.0f);
  z = min(max(z * 1024.0f, 0.0f), 1023.0f);
  unsigned int xx = expandBits((unsigned int)x);
  unsigned int yy = expandBits((unsigned int)y);
  unsigned int zz = expandBits((unsigned int)z);
  return xx * 4 + yy * 2 + zz;
}

BVHAccel::~BVHAccel() {

  // TODO:
  // Implement a proper destructor for your BVH accelerator aggregate
  std::stack<BVHNode*> stk;
  stk.push(root);
  while(stk.size() > 0){
    BVHNode* currNode = stk.top();
    stk.pop();
    if(currNode->l != NULL){
      stk.push(currNode->l);
    }
    if(currNode->r != NULL){
      stk.push(currNode->r);
    }
    free(currNode);
  }

}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

unsigned int BVHAccel::get_MortonCode() const{
  return root->mCode;
}

bool BVHAccel::intersect(const Ray &ray) const {

  // TODO:
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate.
  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if(primitives[p]->intersect(ray)) hit = true;
  }

  return hit;

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i) const {

  // TODO:
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.

  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if(primitives[p]->intersect(ray, i)) hit = true;
  }

  return hit;

}

}  // namespace StaticScene
}  // namespace CMU462
