#include <iostream>
#include <geomc/linalg/Vec.h>
#include <geomc/linalg/AffineTransform.h>
// #include <FastNoise/FastNoise.h>

#include "Image.h"

using namespace geom;
using namespace std;

struct Material {
  Vec3d albedo;
  float reflectiveness;
  float refractiveIndex;
  float fuzziness;
};

struct Sphere {
  Vec3d center;
  float radius;
  Material mtrl;
};

struct Plane {
  Vec3d point;
  Vec3d norm;
  Material mtrl;
};

struct HitPoint {
  Vec3d point;
  Vec3d norm;
  Material mtrl;
};

struct PointLight {
  Vec3d center;
  Vec3d albedo;
};

Vec3d camera;
int sphereCount = 4;
Sphere spheres[4];
PointLight plight;
Plane ground;

// FastNoise myNoise; // Create a FastNoise object


float rrand() {
  return (double)rand()/(double)RAND_MAX;
}

Vec3d normalize(Vec3d v) {
  return v/v.mag();
}

/* adjust these to assign hit point as well, so its abstracted out */
float intersect(Sphere s, Vec3d rayorigin, Vec3d raydir) {
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
  return t0;
}

float intersect(Plane p, Vec3d rayorigin, Vec3d raydir) {
  float d = p.norm.dot(raydir); 
  if(abs(d)>0.000001) { // hit plane from either side
    Vec3d ray = p.point - rayorigin;
    float t = ray.dot(p.norm) / d;
    if(t<0) return -1;
    return t;
  }
  return -1;
}


Vec3d trace(Vec3d ray_origin, Vec3d ray, float medium, int depth) {
  if(depth == 0) {
    return Vec3d(0.705,0.913,.980); // bg
    // return Vec3d(0.,0.,0.);
  } else {
    Vec3d px = Vec3d(0.705,0.913,.980); // bg
    // Vec3d px = Vec3d(0.,0.,0.);
    HitPoint hp;
    Vec3d L;
    float d = 1000000; // ?
    bool hit = false;

    // plane(s)
    float hitDist = intersect(ground, ray_origin, ray);
    if(hitDist > 0 && hitDist < d) {
      d = hitDist;
      hit = true;
      hp.point = ray_origin + d*ray; 
      hp.norm = ground.norm;
      hp.point += 0.0001 * hp.norm; // backup just a bit..
      hp.mtrl = ground.mtrl;
      Vec3d idx = floor(hp.point*10.);
      float checkerboard = abs(fmod((idx.x+idx.z),2.));
      // hp.mtrl.albedo = myNoise.GetNoise(float(hp.point.x), float(hp.point.y), float(hp.point.z));
      hp.mtrl.albedo  = checkerboard * Vec3d(.9,.9,.9) + (1.-checkerboard) * Vec3d(.01,.01,.01);
    }

    // spheres
    for(int i=0; i<sphereCount; i++) {
      float hitDist = intersect(spheres[i], ray_origin, ray);
      if(hitDist > 0 && hitDist < d) {
        d = hitDist;
        hit = true;
        hp.point = ray_origin + d*ray; 
        hp.norm = normalize(hp.point - spheres[i].center);
        hp.point += 0.0001 * hp.norm; // backup just a bit..
        hp.mtrl = spheres[i].mtrl;
      }
    }



    if(hit) {
      /* 
        factor this stuff out.. 
        if refraxcted, ray will go inside. pass parameter to trace to indicate this
        normal should be adjusted accordingly.. and new random ray needs to leave sphere (or plane ... )  
      */
      // pick a random direction
      L = normalize(Vec3d(rrand(), rrand(), rrand()));
      // ensure that it isn't going inside the sphere  
     
     // refraction + reflection
      // if(hp.mtrl.refractiveIndex != medium) {
      //   if(hp.norm.dot(ray) < 0) { //out to in

      //   } else { // in to out

      //   }
      //   // trace ( . ... , hp.mtrl.refractiveIndex, depth-1)
      // } else {
        L = hp.norm.dot(L) < 0 ? -1*L : L;
      // }

      // reflection
      if(rrand() < hp.mtrl.reflectiveness) {
        L = normalize(ray - 2*hp.norm.dot(ray)*hp.norm);
        L += (rrand()*2.-1.)*hp.mtrl.fuzziness;
        L = normalize(L);
        px = trace(hp.point, L, medium, depth-1) * hp.mtrl.albedo * hp.norm.dot(L);
      } 
      // diffusion
      else {
        px = trace(hp.point, L, medium, depth-1) * hp.mtrl.albedo * hp.norm.dot(L);
      } 

      // go hit the light
      Vec3d hitLight = normalize(plight.center - hp.point);
      Vec3d lightCol = plight.albedo * hp.norm.dot(hitLight);
      px = 0.95*px + 0.05*lightCol;     
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
  int count = 120;
  for(int i=0; i<count; i++) {
    Vec3d ray= Vec3d(xy.x, xy.y, 0) - camera;
    Vec3d rayDirection = normalize(ray);
    px += trace(camera, rayDirection, 1, 20)/count;
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
  // myNoise.SetNoiseType(FastNoise::SimplexFractal); // Set the desired noise type

  camera = Vec3d(0., 0., 1.);
  
  // fuzzy reflective ball
  spheres[0].center = Vec3d(-0.65, -.2, -1.5);
  spheres[0].radius = 0.27;
  spheres[0].mtrl.albedo = Vec3d(.7, .5, .4 );  
  spheres[0].mtrl.reflectiveness = .8;
  spheres[0].mtrl.fuzziness = .08;
  spheres[0].mtrl.refractiveIndex = 1.;

  // green ball
  spheres[1].center = Vec3d(0.,0., -2.7);
  spheres[1].radius = 0.3;
  spheres[1].mtrl.albedo = Vec3d(.2, .7, .4);  
  spheres[1].mtrl.reflectiveness = 0.08;
  spheres[1].mtrl.fuzziness = .0;
  spheres[1].mtrl.refractiveIndex = 1.;

  // floating ball
  spheres[2].center = Vec3d(1.5, .7, -6.1);
  spheres[2].radius = .5;
  spheres[2].mtrl.albedo = Vec3d(0.9, 0.9, 0.9);  
  spheres[2].mtrl.reflectiveness = 1.;
  spheres[2].mtrl.fuzziness = .0;
  spheres[2].mtrl.refractiveIndex = 1.3;

  // blue ball
  spheres[3].center = Vec3d(.3, -.2, -.9);
  spheres[3].radius = .1;
  spheres[3].mtrl.albedo = Vec3d(0.1, 0.1, 0.8);  
  spheres[3].mtrl.reflectiveness = .1;
  spheres[3].mtrl.fuzziness = .0;
  spheres[3].mtrl.refractiveIndex = 1.;

  plight.center = Vec3d(20.,100., 112.7);
  plight.albedo = Vec3d(4.3,4.3,4.1);

  ground.point = Vec3d(0., -.3, 0.);
  ground.norm = normalize(Vec3d(0.,1.,0.));
  ground.mtrl.albedo = Vec3d(.128, 0.813, 0.291);
  ground.mtrl.reflectiveness = 1.;
  ground.mtrl.fuzziness = .0;
  ground.mtrl.refractiveIndex = 1.;

  render_image(img);
  
  img.save_png("output/test.png");
  
  return 0;
}
