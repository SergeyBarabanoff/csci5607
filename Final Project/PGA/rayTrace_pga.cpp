//To Compile: brew install libomp -> brew install gcc -> 
// g++-15 -fopenmp -IPGA/gaalet/include -std=c++14 -O3 PGA/rayTrace_pga.cpp -o ray

// ORIGINAL:
// g++ -fsanitize=address -std=c++11 3A_PGA/rayTrace_pga.cpp -o ray
// Newer:
// g++-15 -fopenmp -std=c++11 -O3 3A_PGA/rayTrace_pga.cpp -o ray

//For Visual Studios
#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS // For fopen and sscanf
#define _USE_MATH_DEFINES 
#endif

//Images Lib includes:
#define STB_IMAGE_IMPLEMENTATION //only place once in one .cpp file
#define STB_IMAGE_WRITE_IMPLEMENTATION //only place once in one .cpp files
#include "image_lib.h" //Defines an image class and a color class

//#3D PGA
// #include "PGA_3D.h"
#include "PGA_3D_Gaalet.h"

//High resolution timer
#include <chrono>

//Scene file parser
#include "parse_pga.h"

// Parallelization
#include <omp.h>

// Use Gaalet for 4D PGA
#include <cpp0x/gaalet.h>
using PGA4D = gaalet::algebra< gaalet::signature<4,0,1> >;

// 4D ray
struct Ray4D {
  ga4d::Vec4 origin;
  ga4d::Vec4 dir;
};

// Function to determine intersection between ray and hypersphere
bool rayHypersphereIntersect(Ray4D& ray, Hypersphere& sphere, pga_legacy::Scalar& tHit) {
    ga4d::Vec4  toStart = ray.origin - sphere.center;
    pga_legacy::Scalar a = ga4d::dot(ray.dir, ray.dir);
    pga_legacy::Scalar b = 2 * ga4d::dot(toStart, ray.dir);
    pga_legacy::Scalar c = ga4d::dot(toStart, toStart) - sphere.radius * sphere.radius;

    pga_legacy::Scalar discr = b * b - pga_legacy::Scalar(4.0) * a * c;
    if (discr < pga_legacy::Scalar(0.0)) {
      return false;
    }

    pga_legacy::Scalar t0 = (-1 * b - std::sqrt(discr)) / (pga_legacy::Scalar(2.0) * a);
    pga_legacy::Scalar t1 = (-1 * b + std::sqrt(discr)) / (pga_legacy::Scalar(2.0) * a);

    pga_legacy::Scalar t; 
    if (t0 > inside_epsilon) {
      t = t0;
    } else {
      t = t1;
    }

    if (t <= inside_epsilon) {
      return false;
    }

    tHit = t;
    return true;
}

// Function to determine interesction between ray and hyperplane
bool rayHyperplaneIntersect(Ray4D &ray, HyperPlane &hp, pga_legacy::Scalar &tHit) {
    pga_legacy::Scalar denominator = ga4d::dot(hp.normal, ray.dir);

    // If denom is ~0, ray is parallel to hyperplane
    if (std::fabs(denominator) < denominator_epsilon) {
        return false;
    }

    // Distance from camera to intersection with plane
    pga_legacy::Scalar numerator = -1 * (ga4d::dot(hp.normal, ray.origin) + hp.offset);
    pga_legacy::Scalar t = numerator / denominator;

    // Check if plane is too close to or behind camera, don't register intersection
    if (t <= inside_epsilon) {
        return false;
    }

    tHit = t;
    return true;
}

// Project 4D points into 3D
Point3D project4DPointTo3D(ga4d::Vec4 &vector) {
    // Camera position embedded in 4 dimensions
    ga4d::Vec4 eye4d = ga4d::make_vec4(eye.x(), eye.y(), eye.z(), camera_w);

    // Point relative to camera in 4D
    ga4d::Vec4 relative = vector - eye4d;

    // This determines how much of the 4D space is essentially flattened into 3D
    pga_legacy::Scalar lens = pga_legacy::Scalar(4.0);

    // Factor to scale objects by based on distance from camera
    pga_legacy::Scalar denom = lens - relative[3];
    if (std::fabs(denom) < pga_legacy::Scalar(1e-6)) {
      if (denom >= 0) {
        denom = pga_legacy::Scalar(1e-6);
      }
      else {
        denom =  pga_legacy::Scalar(-1e-6);
      }
    }
    pga_legacy::Scalar scale = lens / denom;

    // Scale projected components
    pga_legacy::Scalar X = relative[0] * scale;
    pga_legacy::Scalar Y = relative[1] * scale;
    pga_legacy::Scalar Z = relative[2] * scale;

    // Project object into 3D
    return Point3D(eye.x() + X, eye.y() + Y, eye.z() + Z);
}

// For directions, just take 3D components
Dir3D project4DDirTo3D(ga4d::Vec4 &vector){
    Dir3D dir(vector[0], vector[1], vector[2]);
    return dir.normalized();
}

// Check for shadows from 4D objects
bool shadow_hit_4D(ga4d::Vec4 &start, ga4d::Vec4 &light, int hit_index, bool hit_type) {
    // Direction to light
    ga4d::Vec4 dir = light - start;

    // Distance to light
    pga_legacy::Scalar distance = ga4d::dot(dir, dir);
    if (distance <= pga_legacy::Scalar(inside_epsilon * inside_epsilon)) {
        return false; // Ignore if light is at 
    }

    // Normalize
    dir = ga4d::normalize(dir);

    // Send out ray from intersection to check shadows
    Ray4D shadowRay;
    shadowRay.origin = start;
    shadowRay.dir = dir;

    // Check hyperspheres
    for (int i = 0; i < num_hyperspheres; ++i) {

        // Avoid shadows from self
        if (!hit_type && i == hit_index) {
          continue;
        }

        pga_legacy::Scalar hit_distance;

        if (rayHypersphereIntersect(shadowRay, hyperspheres[i], hit_distance)) {
            if (hit_distance > inside_epsilon && hit_distance < std::sqrt(distance)) {
                return true;
            }
        }
    }

    // Check hyperplanes
    for (int i = 0; i < num_hyperplanes; ++i) {

        // Avoid shadows from self
        if (!hit_type && i == hit_index) {
          continue;
        }

        pga_legacy::Scalar hit_distance;

        if (rayHyperplaneIntersect(shadowRay, hyperplanes[i], hit_distance)) {
            if (hit_distance > inside_epsilon && hit_distance < std::sqrt(distance)) {
                return true;
            }
        }
    }

    return false;
}

// Function to calculate lighting based on illumination model
Color lighting_model(ga4d::Vec4& hit, ga4d::Vec4& normal_4d, int hit_index, bool hit_type, Point3D& intersect, Dir3D& normal, Material& material) {
  Color color(material.a.r * ambient.r, material.a.g * ambient.g, material.a.b * ambient.b); // Ambient lighting

  // Loop over point lights
  for (int i = 0; i < num_point_lights; ++i) {

    // Light, eye, and reflection vectors
    Dir3D L = (point_lights[i].pos - intersect).normalized();

    // Check to see if the light is blocked
    bool shadow = false;
    
    // Distance to light (with small perturbation to avoid divide by zero)
    pga_legacy::Scalar distance = dot(point_lights[i].pos - intersect, point_lights[i].pos - intersect);
    pga_legacy::Scalar adjustment = 1 / (distance + epsilon);

    // Shadow ray origin in 4D
    ga4d::Vec4 shadow_start = hit + normal_4d * pga_legacy::Scalar(epsilon);

    // Make light 4D
    ga4d::Vec4 light_4d = ga4d::make_vec4(point_lights[i].pos.x(), point_lights[i].pos.y(), point_lights[i].pos.z(), camera_w);

    bool in_shadow = shadow_hit_4D(shadow_start, light_4d, hit_index, hit_type);
    if (in_shadow) {
        continue;
    }

    Dir3D V = (eye - intersect).normalized();
    Dir3D R = (2 * dot(normal, L) * normal - L).normalized();

    // Dot products
    pga_legacy::Scalar normal_dot_l = std::max(pga_legacy::Scalar(0.0), dot(normal, L));
    pga_legacy::Scalar v_dot_r = std::max(pga_legacy::Scalar(0.0), dot(V, R));

    // Diffuse light
    Color diffuse(
        normal_dot_l * material.d.r * point_lights[i].color.r * adjustment,
        normal_dot_l * material.d.g * point_lights[i].color.g * adjustment,
        normal_dot_l * material.d.b * point_lights[i].color.b * adjustment);

    // Specular light
    Color specular(
      powf(v_dot_r, material.ns) * material.s.r * point_lights[i].color.r * adjustment,
      powf(v_dot_r, material.ns) * material.s.g * point_lights[i].color.g * adjustment,
      powf(v_dot_r, material.ns) * material.s.b * point_lights[i].color.b * adjustment);

    // Update color with this light
    color.r = color.r + diffuse.r + specular.r;
    color.g = color.g + diffuse.g + specular.g;
    color.b = color.b + diffuse.b + specular.b;
  };

  // Loop over directional lights
  for (int i = 0; i < num_dir_lights; ++i) {

    // Vectors at intersection
    Dir3D L = (-1 * dir_lights[i].dir).normalized(); // Direction toward light source

    // Check to see if the light is blocked
    bool shadow = false;

    // Shadow ray origin in 4D
    ga4d::Vec4 shadow_start = hit + normal_4d * pga_legacy::Scalar(epsilon);
    
    Dir3D V = (eye - intersect).normalized();
    Dir3D R = (2 * dot(normal, L) * normal - L).normalized();

    // Dot products
    pga_legacy::Scalar normal_dot_l = std::max(pga_legacy::Scalar(0.0), dot(normal, L));
    pga_legacy::Scalar v_dot_r = std::max(pga_legacy::Scalar(0.0), dot(V, R));

    // Diffuse light
    Color diffuse(
        normal_dot_l * material.d.r * dir_lights[i].color.r,
        normal_dot_l * material.d.g * dir_lights[i].color.g,
        normal_dot_l * material.d.b * dir_lights[i].color.b);

    // Specular light
    Color specular(
      powf(v_dot_r, material.ns) * material.s.r * dir_lights[i].color.r,
      powf(v_dot_r, material.ns) * material.s.g * dir_lights[i].color.g,
      powf(v_dot_r, material.ns) * material.s.b * dir_lights[i].color.b);

    // Update color with this light
    color.r = color.r + diffuse.r + specular.r;
    color.g = color.g + diffuse.g + specular.g;
    color.b = color.b + diffuse.b + specular.b;
  }

  // Loop over spot lights
  for (int i = 0; i < num_spot_lights; ++i) {

    // Vectors at intersection
    Dir3D L = (spot_lights[i].pos - intersect).normalized();

    // Check to see if the light is blocked
    bool shadow = false;

    // Shadow ray origin in 4D
    ga4d::Vec4 shadow_start = hit + normal_4d * pga_legacy::Scalar(epsilon);

    // Make light 4D
    ga4d::Vec4 light_4d = ga4d::make_vec4(spot_lights[i].pos.x(), spot_lights[i].pos.y(), spot_lights[i].pos.z(), camera_w);

    bool in_shadow = shadow_hit_4D(shadow_start, light_4d, hit_index, hit_type);
    if (in_shadow) {
        continue;
    }

    // Distance to light (with small perturbation to avoid divide by zero)
    pga_legacy::Scalar distance = dot(spot_lights[i].pos - intersect, spot_lights[i].pos - intersect);
    pga_legacy::Scalar adjustment = 1 / (distance + epsilon);

    Dir3D V = (eye - intersect).normalized();
    Dir3D R = (2 * dot(normal, L) * normal - L).normalized();

    // Dot products
    pga_legacy::Scalar normal_dot_l = std::max(pga_legacy::Scalar(0.0), dot(normal, L));
    pga_legacy::Scalar v_dot_r = std::max(pga_legacy::Scalar(0.0), dot(V, R));

    // Calculate cosine(angle) from light to intersection in radians
    pga_legacy::Scalar cos_theta = dot(spot_lights[i].dir.normalized(), -1 * L);
    pga_legacy::Scalar cos_inside = std::cos(spot_lights[i].angle1 * M_PI / 180); // Inner angle
    pga_legacy::Scalar cos_outside = std::cos(spot_lights[i].angle2 * M_PI / 180); // Outer angle

    // Make sure inner is actually inside and outer is outside
    if (cos_inside < cos_outside) {
      pga_legacy::Scalar temp = cos_inside;
      cos_inside = cos_outside;
      cos_outside = temp;
    }

    // Adjust falloff based on angle
    if (cos_theta <= cos_outside) { // 0 if angle is greater than angle2
      adjustment = 0;
    }
    else if (cos_theta < cos_inside) { // Multiply by linear slope if between
      adjustment = adjustment * (cos_theta - cos_outside) / (cos_inside - cos_outside);
    };

    // Diffuse light
    Color diffuse(
        normal_dot_l * material.d.r * spot_lights[i].color.r * adjustment,
        normal_dot_l * material.d.g * spot_lights[i].color.g * adjustment,
        normal_dot_l * material.d.b * spot_lights[i].color.b * adjustment);

    // Specular light
    Color specular(
      powf(v_dot_r, material.ns) * material.s.r * spot_lights[i].color.r * adjustment,
      powf(v_dot_r, material.ns) * material.s.g * spot_lights[i].color.g * adjustment,
      powf(v_dot_r, material.ns) * material.s.b * spot_lights[i].color.b * adjustment);

    // Update color with this light
    color.r = color.r + diffuse.r + specular.r;
    color.g = color.g + diffuse.g + specular.g;
    color.b = color.b + diffuse.b + specular.b;
  }

  // Clamp colors
  color.r = std::min(color.r, 1.0f);
  color.g = std::min(color.g, 1.0f);
  color.b = std::min(color.b, 1.0f);

  return color;
};

// Function to evaluate 4D ray tree
Color traceRay4D(Ray4D &ray, int depth) {
    if (depth > max_depth) {
        return background;
    }

    // Nearest object information
    pga_legacy::Scalar nearest = pga_legacy::Scalar(1e15);
    int best_index = -1;
    bool best_type = false;

    // Check hyperspheres
    for (int i = 0; i < num_hyperspheres; ++i) {
        pga_legacy::Scalar hit_distance;

        if (rayHypersphereIntersect(ray, hyperspheres[i], hit_distance)) {
            if (hit_distance > inside_epsilon && hit_distance < nearest) {
                nearest = hit_distance;
                best_index = i;
                best_type = false;
            }
        }
    }

    // Check hyperplanes
    for (int i = 0; i < num_hyperplanes; ++i) {
        pga_legacy::Scalar hit_distance;

        if (rayHyperplaneIntersect(ray, hyperplanes[i], hit_distance)) {
            if (hit_distance > inside_epsilon && hit_distance < nearest) {
                nearest = hit_distance;
                best_index = i;
                best_type = true;
            }
        }
    }

    // If no object was hit return background color
    if (best_index == -1) {
        return background;
    }

    // Intersection information
    ga4d::Vec4 hit;
    ga4d::Vec4 normal_4d;
    Material material;

    // Get information about intersected object
    if (!best_type) {
        Hypersphere &hs = hyperspheres[best_index];

        hit = ray.origin + ray.dir * nearest;
        normal_4d = hit - hs.center;
        material = hs.material;
    } else {
        HyperPlane &hp = hyperplanes[best_index];

        hit = ray.origin + ray.dir * nearest;

        // Adjust normal to face the ray
        ga4d::Vec4 N = hp.normal;
        if (ga4d::dot(N, ray.dir) > 0.0) {
            N = N * pga_legacy::Scalar(-1.0);
        }

        normal_4d = N;
        material = hp.material;
    }

    ga4d::Vec4 n4_norm = ga4d::normalize(normal_4d);

    // Project to 3D
    Point3D intersect = project4DPointTo3D(hit);
    Dir3D normal = project4DDirTo3D(normal_4d);

    // Get color of object from lighting model
    Color color_here = lighting_model(hit, n4_norm, best_index, best_type, intersect, normal, material);

    // Calculate color from reflection
    Color color_reflection(0, 0, 0);
    if (material.s.r > 0 || material.s.g > 0 || material.s.b > 0) {

        // Reflect ray in 4D around normal
        pga_legacy::Scalar cosine = ga4d::dot(ray.dir, ga4d::normalize(normal_4d));
        ga4d::Vec4 reflected_dir = ray.dir - ga4d::normalize(normal_4d) * (2.0 * cosine);

        // Normalize and offset slightly
        reflected_dir = ga4d::normalize(reflected_dir);
        ga4d::Vec4 reflected_origin = hit + reflected_dir * pga_legacy::Scalar(epsilon);

        // Make a reflected ray
        Ray4D reflected_ray;
        reflected_ray.origin = reflected_origin;
        reflected_ray.dir = reflected_dir;

        // Trace reflected ray
        color_reflection = traceRay4D(reflected_ray, depth + 1);

        // Adjust based on specularity
        color_reflection = Color(
            color_reflection.r * material.s.r,
            color_reflection.g * material.s.g,
            color_reflection.b * material.s.b
        );
    }

    // Get color from refraction
    Color color_refraction(0, 0, 0);
    if ((material.t.r > 0 || material.t.g > 0 || material.t.b > 0) && material.ior > 0) {
        pga_legacy::Scalar eta_r = material.ior;
        pga_legacy::Scalar eta_i = 1.0;
        ga4d::Vec4 n = n4_norm;
        ga4d::Vec4 I = ga4d::normalize(ray.dir);

        // External angle from ray
        pga_legacy::Scalar cos_i = -ga4d::dot(n, I);

        // Flip parameters if ray is exiting object
        if (cos_i < 0.0) {
            n = n * pga_legacy::Scalar(-1.0);
            cos_i = -cos_i;
            eta_i = material.ior;
            eta_r = 1.0;
        }

        pga_legacy::Scalar ratio = eta_i / eta_r;
        pga_legacy::Scalar under_sqrt = 1.0 - ratio * ratio * (1.0 - cos_i * cos_i);

        if (under_sqrt >= 0.0) {
            pga_legacy::Scalar cos_r = std::sqrt(under_sqrt);

            // Refraction direction in 4D
            ga4d::Vec4 refracted_dir = n * ((ratio * cos_i) - cos_r) + I * ratio;
            refracted_dir = ga4d::normalize(refracted_dir);

            // Refraction origin
            ga4d::Vec4 refracted_origin = hit + refracted_dir * pga_legacy::Scalar(epsilon);

            // Refracted ray
            Ray4D refracted_ray;
            refracted_ray.origin = refracted_origin;
            refracted_ray.dir = refracted_dir;

            // Evaluate refracted ray tree
            color_refraction = traceRay4D(refracted_ray, depth + 1);

            // Update color based on transmisiveness
            color_refraction = Color(
                color_refraction.r * material.t.r,
                color_refraction.g * material.t.g,
                color_refraction.b * material.t.b
            );
        }
    }

    // Overall color
    Color color = Color(
        color_here.r + color_reflection.r + color_refraction.r,
        color_here.g + color_reflection.g + color_refraction.g,
        color_here.b + color_reflection.b + color_refraction.b
    );

    // Clamp color
    color.r = std::min(color.r, 1.0f);
    color.g = std::min(color.g, 1.0f);
    color.b = std::min(color.b, 1.0f);

    return color;
}

int main(int argc, char** argv){
  
  //Read command line parameters to get scene file
  if (argc != 2){
     std::cout << "Usage: ./a.out scenefile\n";
     return(0);
  }
  std::string sceneFileName = argv[1];

  //Parse Scene File
  parseSceneFile(sceneFileName);
  bool use4D = (num_hyperspheres > 0);

  float imgW = img_width, imgH = img_height;
  float halfW = imgW/2, halfH = imgH/2;
  float d = halfH / tanf(halfAngleVFOV * (M_PI / 180.0f));

  Image outputImg = Image(img_width,img_height);
  auto t_start = std::chrono::high_resolution_clock::now();

  // Parellelize
  #pragma omp parallel for num_threads(12) collapse(2) schedule(dynamic, 4)
  for (int i = 0; i < img_width; i++){
    for (int j = 0; j < img_height; j++){
      float u = (halfW - (imgW) * ((i + 0.5) / imgW));
      float v = (halfH - (imgH) * ((j + 0.5) / imgH));
      Color color;
      Point3D p = eye - d * forward_dir + u * right_dir + v * up_dir;
      Dir3D rayDir = (p - eye).normalized(); 

      // Add 3D effect
      if (!anaglyph) {
        Ray4D ray;

        // Convert origin to 4D coordinates
        ray.origin = ga4d::make_vec4(eye.x(), eye.y(), eye.z(), camera_w);
        ray.dir = ga4d::normalize(ga4d::make_vec4(rayDir.dx, rayDir.dy, rayDir.dz, pga_legacy::Scalar(0.0)));

        // Ray trace
        color = traceRay4D(ray, 0);
      } else {
        float separation = 0.25f; // Difference between camera positions
        float separation_x = 0.0f; // x-axis offset

        // Left eye
        Point3D left_eye = eye - (separation_x * 0.5f) * right_dir;
        Point3D left_p = left_eye - d * forward_dir + u * right_dir + v * up_dir;
        Dir3D left_dir = (left_p - left_eye).normalized();

        Ray4D left_ray;

        left_ray.origin = ga4d::make_vec4(left_eye.x(), left_eye.y(), left_eye.z(), camera_w - separation);
        left_ray.dir = ga4d::normalize(ga4d::make_vec4(left_dir.dx, left_dir.dy, left_dir.dz, 0.0));

        Color left_color = traceRay4D(left_ray, 0);
        color.r = left_color.r; // Only red channels here

        // Right eye
        Point3D right_eye = eye + (separation_x * 0.5f) * right_dir;
        Point3D right_p = right_eye - d * forward_dir + u * right_dir + v * up_dir;
        Dir3D right_dir = (right_p - right_eye).normalized();

        Ray4D right_ray;

        right_ray.origin = ga4d::make_vec4(right_eye.x(), right_eye.y(), right_eye.z(), camera_w + separation);
        right_ray.dir = ga4d::normalize(ga4d::make_vec4(right_dir.dx, right_dir.dy, right_dir.dz, 0.0));

        Color right_color = traceRay4D(right_ray, 0);
        color.g = right_color.g; // Only cyan here
        color.b = right_color.b;
      }

      outputImg.setPixel(i,j, color);
    }
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  printf("Rendering took %.2f ms\n",std::chrono::duration<double, std::milli>(t_end-t_start).count());

  outputImg.write(imgName.c_str());
  return 0;
}
