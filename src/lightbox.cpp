#include <iostream>
#include <geomc/linalg/Vec.h>
#include <geomc/linalg/AffineTransform.h>

#include "Image.h"

using namespace geom;
using namespace std;

struct sphere {
  Vec3d center;
  Vec3d albedo;
  float radius;
};

Vec3d camera = Vec3d(0., 0., -1.);

sphere spheres[3];

float rrand() {
  return (double)rand()/(double)RAND_MAX;
}

float intersect(sphere s, Vec3d cam, Vec3d raydir) {
  Vec3d L = s.center - cam;
  float tca = L.dot(raydir.x, raydir.y, raydir.z);
  if(tca < 0) return -1;
  float d = sqrt(L.mag2() - tca*tca);
  if(d<0 || d >s.radius) return -1;
  float tch = sqrt(s.radius*s.radius - d*d);
  float t0 = tca - tch;
  float t1 = tca + tch;
  if(t0<0) return t1;
  if(t1<0) return -1;
  return t0;
}


Vec3d trace(Vec3d ray_origin, Vec3d ray, int depth) {
  if(depth == 0) {
    return Vec3d(.8, .8, .8); // bg
  } else {
    Vec3d px = Vec3d(.8, .8, .8); // bg
    Vec3d hitP;
    Vec3d norm;
    Vec3d L;
    Vec3d lamb;
    float d = 1000; // ?
    int idx = -1;
    bool hit = false;
    for(int i=0; i<3; i++) {
      float hitDist = intersect(spheres[i], ray_origin, ray);
      if(hitDist > 0 && hitDist < d) {
        d = hitDist;
        idx = i;
        hit = true;
      }
    }
    // back up a bit to ensure point isn't inside the sphere
    hitP = ray_origin + (d-0.0001)*ray; 
    // calculate normal
    norm = hitP - spheres[idx].center;
    norm /= norm.mag();
    // pick a random direction
    L = Vec3d(rrand()*2-1, rrand()*2-1, rrand()*2-1);
    L = L/L.mag();
    // ensure that it isn't going inside the sphere    
    L = norm.dot(L) < 0 ? -1*L : L;
    if(hit) {
      px = trace(hitP, L, depth-1) * spheres[idx].albedo * norm.dot(L);
    } 
    return px;
  }
}


// Produce a pixel color based on the image coordinates x,y.
// The image center will be (0,0), and x will range from -0.5 to 0.5.
inline Vec3d render_pixel(Vec2d xy) {
  Vec3d px = Vec3d(0.);
  int count = 100;
  for(int i=0; i<count; i++) {
    Vec3d ray= Vec3d(xy.x, xy.y, 0) - camera;
    Vec3d rayDirection = ray/ray.mag();
    px += trace(camera, rayDirection, 120)/count;
  } 
  return px;
}


void render_image(Image<double,3>& img) {
  for (index_t y = 0; y < img.height; ++y) {
    for (index_t x = 0; x < img.width; ++x) {
      Vec2i xy = Vec2i(x,y);                // integer pixel coordinates
      Vec2d st = ((Vec2d)(xy)) / img.width; // floating point image coordinates
      st -= Vec2d(0.5, (0.5 * img.height) / img.width); // center on (0,0)
      img[xy] = render_pixel(st);
    }
  }
}


int main(int argc, char** argv) {
  Image<double,3> img(1024, 512); // a 3-channel image.
  
  // for(int i=0; i<100; i++) {
  //   cout << (double)rand()/(double)RAND_MAX << endl;
  // }
  
  spheres[0].center = Vec3d(-0.4, -.2, .7);
  spheres[0].albedo = Vec3d(0.1, 0., 1.);
  spheres[0].radius = 0.4;

  spheres[1].center = Vec3d(.0, .0, 1.);
  spheres[1].albedo = Vec3d(.2, .7, .4);
  spheres[1].radius = 0.35;

  spheres[2].center = Vec3d(.55, .3, 1.6);
  spheres[2].albedo = Vec3d(1., 0.5, 0.4);
  spheres[2].radius = .6;

  render_image(img);
  
  img.save_png("output/test.png");
  
  return 0;
}
