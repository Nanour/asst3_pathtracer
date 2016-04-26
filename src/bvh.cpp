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
  double minx = INF_D, miny = INF_D, minz = INF_D;
  double maxx = 0, maxy = 0, maxz = 0;
  for (size_t i = 0; i < primitives.size(); ++i) {
    bb.expand(primitives[i]->get_bbox());
    double centerx =  primitives[i]->get_bbox().centroid().x;
    maxx = std::max(maxx, centerx);
    minx = std::min(minx, centerx);

    double centery =  primitives[i]->get_bbox().centroid().y;
    maxy = std::max(maxy, centery);
    miny = std::min(miny, centery);

    double centerz =  primitives[i]->get_bbox().centroid().z;
    maxz = std::max(maxz, centerz);
    minz = std::min(minz, centerz);
  }
  // cout << maxx << endl;
  // cout << minx << endl;
  valueRangeMin = Vector3D(minx, miny, minz);
  valueRangeMax = Vector3D(maxx, maxy, maxz);

  for(size_t i = 0; i < primitives.size(); i++){
    mCodeList.push_back(primitives[i]->get_MortonCode(valueRangeMax, valueRangeMin));
  }

  //cout << primitives.size() << endl;
  //std::sort(primitives.begin(),primitives.end(),mCodeCmp());
  SortPrimitive(0, primitives.size()-1);
  root = new BVHNode(bb, 0, primitives.size());

  cudaLNodeList = new CudaLNode[primitives.size()];
  deviceBboxList = new DeviceBBox[2 * primitives.size()];
  cudaBVHNodeList = new CudaBVHNode[2 * primitives.size()];

  for(size_t i = 0; i < primitives.size(); i++){
    Vector3D nodeCentroid = primitives[i]->get_bbox().centroid();
    cudaLNodeList[i].mCode = mCodeList[i];
    cudaLNodeList[i].x = nodeCentroid.x;
    cudaLNodeList[i].y = nodeCentroid.y;
    cudaLNodeList[i].z = nodeCentroid.z;

    Vector3D bbox_max = primitives[i]->get_bbox().max;
    Vector3D bbox_min = primitives[i]->get_bbox().min;
    deviceBboxList[i+primitives.size()].maxx = bbox_max.x;
    deviceBboxList[i+primitives.size()].maxy = bbox_max.y;
    deviceBboxList[i+primitives.size()].maxz = bbox_max.z;

    deviceBboxList[i+primitives.size()].minx = bbox_min.x;
    deviceBboxList[i+primitives.size()].miny = bbox_min.y;
    deviceBboxList[i+primitives.size()].minz = bbox_min.z;
  }

  //Compute GPU
  
  //Reconstruct BVH
  generateHierarchy(root, 0);

/*
  std::stack<BVHNode*> stk;
  stk.push(root);
  while(stk.size() > 0){
    BVHNode* currNode = stk.top();
    stk.pop();
    bool hasChild = generateHierarchy(currNode, max_leaf_size);
    if(hasChild){
      stk.push(currNode->r);
      stk.push(currNode->l);
    }
  }
  this->root = root;
  this->primitives = primitives;
*/
}

bool BVHAccel::generateHierarchy(BVHNode* currNode, int currIdx) {
    int lcIdx = cudaBVHNodeList[currIdx].l;
    if (lcIdx >= 0) {
      BBox lbox = BBox((double)deviceBboxList[lcIdx].minx, 
        (double)deviceBboxList[lcIdx].miny,
        (double)deviceBboxList[lcIdx].minz,
        (double)deviceBboxList[lcIdx].maxx,
        (double)deviceBboxList[lcIdx].maxy,
        (double)deviceBboxList[lcIdx].maxz);
      BVHNode* lc = new BVHNode(lbox, cudaBVHNodeList[lcIdx].start, cudaBVHNodeList[lcIdx].range);
      currNode->l = lc;
      generateHierarchy(lc, lcIdx);
    }

    int rcIdx = cudaBVHNodeList[currIdx].r;
    if (rcIdx >= 0) {
      BBox rbox = BBox((double)deviceBboxList[rcIdx].minx, 
        (double)deviceBboxList[rcIdx].miny,
        (double)deviceBboxList[rcIdx].minz,
        (double)deviceBboxList[rcIdx].maxx,
        (double)deviceBboxList[rcIdx].maxy,
        (double)deviceBboxList[rcIdx].maxz);
      BVHNode* rc = new BVHNode(rbox, cudaBVHNodeList[rcIdx].start, cudaBVHNodeList[rcIdx].range);
      currNode->r = rc;
      generateHierarchy(rc, rcIdx);
    }
}

/*
bool BVHAccel::generateHierarchy(BVHNode* currNode, size_t max_leaf_size){
  if(currNode->range <= max_leaf_size || currNode->bb.empty()){
    cout << "Not Processed" << endl;
    currNode->l = NULL;
    currNode->r = NULL;
    return false;
  }

  int split = findSplit(currNode); 
  cout << currNode->start+split << endl;
  
  BBox leftbb, rightbb;
  size_t left = 0, right = 0;

  for(size_t i = currNode->start; i <= currNode->start+split; i++){
    Primitive* prim = primitives[i];

    leftbb.expand(prim->get_bbox());
    left++;
  }

  for(size_t i = currNode->start+split+1; i < currNode->start+currNode->range; i++){
    Primitive* prim = primitives[i];

    rightbb.expand(prim->get_bbox());
    right++;
  }

  if(currNode->range == left || currNode->range == right){
    cout << "All in same side" << endl;
      return false;
    }
  else{
    currNode->l = new BVHNode(leftbb, currNode->start, left);
    currNode->r = new BVHNode(rightbb, currNode->start+left, right);
    return true;
  }

}
*/
int BVHAccel::findSplit(BVHNode* currNode){

  int first = currNode->start;
  int last = currNode->start+currNode->range-1;
  // Identical Morton codes => split the range in the middle.
  cout << "first: " << currNode->start << endl;
  unsigned int firstCode = primitives[first]->get_MortonCode(valueRangeMax, valueRangeMin);
  cout << "firstCode: " << firstCode << endl;

  cout << "last: " << currNode->start+currNode->range-1 << endl;
  unsigned int lastCode = primitives[last]->get_MortonCode(valueRangeMax, valueRangeMin);
  cout << "lastCode: " << lastCode << endl;

  if (firstCode == lastCode)
    return (first + last) >> 1;
    //return first;

  // Calculate the number of highest bits that are the same
  // for all objects, using the count-leading-zeros intrinsic.

  int commonPrefix = CLZ1(firstCode ^ lastCode);

  // Use binary search to find where the next bit differs.
  // Specifically, we are looking for the highest object that
  // shares more than commonPrefix bits with the first one.

  int split = first; // initial guess
  int step = last - first;
  //cout << "S " << step << endl;

  do{
    step = (step + 1) >> 1; // exponential decrease
    int newSplit = split + step; // proposed new position

    if(newSplit < last){
      unsigned int splitCode = primitives[newSplit]->get_MortonCode(valueRangeMax, valueRangeMin);
      int splitPrefix = CLZ1(firstCode ^ splitCode);
      if(splitPrefix > commonPrefix)
        split = newSplit; // accept proposal
    }
  }
  while (step > 1);

  return split - first;

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

void BVHAccel::SortPrimitive(int h, int t) {
  if (h >= t) return;
  int i = h;
  int j = t;
  unsigned int k = mCodeList[(i + j) >> 1];

  do {
    while ((mCodeList[i] < k) && (i < t)) i++;
    while ((k < mCodeList[j]) && (j > h)) j--;
    if (i <= j) {
      Primitive* s = primitives[i];
      primitives[i] = primitives[j];
      primitives[j] = s;
      unsigned int s1 = mCodeList[i];
      mCodeList[i] = mCodeList[j];
      mCodeList[j] = s1;
      i++;
      j--;
    }
  }
  while (i <= j);

  if (i <= t) SortPrimitive(i, t);
  if (j >= h) SortPrimitive(h, j);
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

unsigned int BVHAccel::get_MortonCode(const Vector3D& max, const Vector3D& min) const{
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
