#include <iostream>
#include <geomc/linalg/Vec.h>
#include <geomc/linalg/AffineTransform.h>

#include "Image.h"

using namespace geom;
using namespace std;

struct Sphere {
  Vec3d center;
  Vec3d albedo;
  float radius;
  float reflectiveness;
};

struct PointLight {
  Vec3d center;
  Vec3d albedo;
};

struct Plane {
  Vec3d point;
  Vec3d norm;
  Vec3d albedo;
  float reflectiveness;
};

struct HitPoint {
  Vec3d point;
  Vec3d norm;
  Vec3d albedo;
  float reflect;
};

Vec3d camera;
int sphereCount = 4;
Sphere spheres[4];
PointLight plight;
Plane ground;

float rrand() {
  return (double)rand()/(double)RAND_MAX;
}

Vec3d normalize(Vec3d v) {
  return v/v.mag();
}

/* adjust these to assign hit point as well, so its abstracted out */
float intersect(Sphere s, Vec3d rayorigin, Vec3d raydir, Vec3d &hitP, Vec3d &norm) {
  Vec3d L = s.center - rayorigin;
  float tca = L.dot(raydir.x, raydir.y, raydir.z);
  if(tca < 0) return -1;
  float d = sqrt(L.mag2() - tca*tca);
  if(d<0 || d >s.radius) return -1;
  float tch = sqrt(s.radius*s.radius - d*d);
  float t0 = tca - tch;
  float t1 = tca + tch;
  if(t0<0) t0 = t1;
  if(t1<0) return -1;
  hitP = rayorigin + (t0-0.0001)*raydir; // backup just a bit..
  norm = normalize(hitP - s.center);
  return t0;
}

float intersect(Plane p, Vec3d rayorigin, Vec3d raydir, Vec3d &hitP, Vec3d &norm) {
  float d = p.norm.dot(raydir); 
  if(abs(d)>0.000001) { // hit plane from either side
    Vec3d ray = p.point - rayorigin;
    float t = ray.dot(p.norm) / d;
    if(t<0) return -1;
    hitP = rayorigin + (t-0.0001)*raydir; // backup just a bit..
    norm = p.norm;
    return t;
  }
  return -1;
}


Vec3d trace(Vec3d ray_origin, Vec3d ray, int depth) {
  if(depth == 0) {
    //180-233-250
    return Vec3d(0.705,0.913,.980); // bg
  } else {
    Vec3d px = Vec3d(0.705,0.913,.980); // bg
    HitPoint hp;
    Vec3d L;
    float d = 1000000000; // ?
    int idx = -1;
    bool hit = false;

    // plane(s)
    Vec3d hitP;
    Vec3d norm;
    float hitDist = intersect(ground, ray_origin, ray, hitP, norm);
    if(hitDist > 0 && hitDist < d) {
      d = hitDist;
      hit = true;
      hp.point = hitP;
      hp.norm = norm;
      hp.albedo = ground.albedo;
      hp.reflect = ground.reflectiveness;
    }

    // spheres
    for(int i=0; i<sphereCount; i++) {
      Vec3d hitP;
      Vec3d norm;
      float hitDist = intersect(spheres[i], ray_origin, ray, hitP, norm);
      if(hitDist > 0 && hitDist < d) {
        d = hitDist;
        idx = i;
        hit = true;
        hp.point = hitP;
        hp.norm = norm;
        hp.albedo = spheres[i].albedo;
        hp.reflect = spheres[i].reflectiveness;
      }
    }



    // pick a random direction
    L = normalize(Vec3d(rrand(), rrand(), rrand()));
    // ensure that it isn't going inside the sphere  
    // (reflect ray instead?)  
    L = hp.norm.dot(L) < 0 ? -1*L : L;

    // // first sphere is reflective
    // //  put this inside loop so rays can be weighed 
    // if(idx == 2) {
    //   L = ray - 2*norm.dot(ray)*norm;
    // }

    if(hit) {
      if(rrand() < hp.reflect) {
        L = normalize(ray - 2*hp.norm.dot(ray)*hp.norm);
        px = trace(hp.point, L, depth-1) * hp.albedo * hp.norm.dot(L);
      } else {
        px = trace(hp.point, L, depth-1) * hp.albedo * hp.norm.dot(L);
        // go hit the light
        Vec3d hitLight = normalize(plight.center - hp.point);
        Vec3d lightCol = plight.albedo * hp.norm.dot(hitLight);

        // average out ray effects
        // weight them differently... 
        px = 0.8*px + 0.2*lightCol;
      }      
    } 
    return px;
  }
}


/*
triangle intersection - barycentric cooridinates
for normal : use two vectors out from the same point and cross them
how does that relate to right hand / left hand rule for seeing direction of normal? 
 */

// Produce a pixel color based on the image coordinates x,y.
// The image center will be (0,0), and x will range from -0.5 to 0.5.
inline Vec3d render_pixel(Vec2d xy) {
  Vec3d px = Vec3d(0.);
  int count = 160;
  for(int i=0; i<count; i++) {
    Vec3d ray= Vec3d(xy.x, xy.y, 0) - camera;
    Vec3d rayDirection = normalize(ray);
    px += trace(camera, rayDirection, 20)/count;
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
  
  camera = Vec3d(0., 0., 1.);
  
  spheres[0].center = Vec3d(-0.65, -.2, -1.5);
  spheres[0].albedo = Vec3d(.7, .5, .4 );
  spheres[0].radius = 0.27;
  spheres[0].reflectiveness = 0.;

  spheres[1].center = Vec3d(0.,0., -2.7);
  spheres[1].albedo = Vec3d(.2, .7, .4);
  spheres[1].radius = 0.3;
  spheres[1].reflectiveness = 0.08;

  spheres[2].center = Vec3d(1.5, .7, -6.1);
  spheres[2].albedo = Vec3d(0.9, 0.9, 0.9);
  spheres[2].radius = .5;
  spheres[2].reflectiveness = 1.;

  spheres[3].center = Vec3d(.3, -.2, -.9);
  spheres[3].albedo = Vec3d(0.1, 0.1, 0.8);
  spheres[3].radius = .1;
  spheres[3].reflectiveness = .1;

  plight.center = Vec3d(-3.,10.,5.);
  plight.albedo = Vec3d(1.3,1.3,0.);

  ground.point = Vec3d(0., -.3, 0.);
  ground.norm = normalize(Vec3d(0.,1.,0.));
  ground.albedo = Vec3d(.128, 0.813, 0.291);
  ground.reflectiveness = .0;

  render_image(img);
  
  img.save_png("output/test.png");
  
  return 0;
}
