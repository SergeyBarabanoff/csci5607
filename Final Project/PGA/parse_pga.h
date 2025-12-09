// Completed
// Camera Placement, background colors, BMP or PNG output, Spheres, Ambient lights, point lights, Multiple lights, Directional, spot, Color/specularity
// Recursion, basic sampling, reflection, shadows, refraction
//Set the global scene parameter variables
//Set the scene parameters based on the values in the scene file

#ifndef PARSE_PGA_H
#define PARSE_PGA_H

#include <cstdio>
#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <algorithm>

//Camera & Scene Parameters (Global Variables)
//Here we set default values, override them in parseSceneFile()

//Image Parameters
int img_width = 640, img_height = 480;
std::string imgName = "raytraced.bmp";

//Camera Parameters
Point3D eye = Point3D(0,0,0); 
Dir3D forward_dir = Dir3D(0,0,-1).normalized();
Dir3D up_dir = Dir3D(0,1,0).normalized();
Dir3D right_dir = Dir3D(-1,0,0).normalized();
float halfAngleVFOV = 45; 
Scalar camera_w = 0.0; // 4D camera coordinate
bool anaglyph = false; // Whether to have 3D effect or not

// Different epsilons for different situations
Scalar epsilon = 1e-3;
Scalar denominator_epsilon = 1e-8f;
Scalar inside_epsilon = 1e-8;

// Recursion
int max_depth = 5;
int max_objects = 250000;

// Handle multiple spheres
std::vector<Point3D> spherePos;
std::vector<Scalar> sphereRadius; 

// Material
struct Material {
  Color a, d, s, t; // Ambient, diffuse, specular, and transmissive colors
  Scalar ns; // Phong power
  Scalar ior; // Index of refraction
};

// 4D sphere (hypersphere)
struct Hypersphere {
  ga4d::Vec4 center;
  Scalar radius;
  Material material;
};

Hypersphere hyperspheres[250000];
int num_hyperspheres = 0;

// 4D hyperplane: a x + b y + c z + d w + e = 0
struct HyperPlane {
  ga4d::Vec4 normal;
  pga_legacy::Scalar offset;
  Material material;
};

// Storage for hyperplanes
HyperPlane hyperplanes[250000];
int num_hyperplanes = 0;

Material default_material() {
  Material material;
  material.a = Color(0, 0, 0);
  material.d = Color(1, 1, 1);
  material.s = Color(0, 0, 0);
  material.t = Color(0, 0, 0);
  material.ns = 5.0f;
  material.ior = 1.0f;
  return material;
}

Material current_material = default_material();

// Background
Color background = Color(0, 0, 0);

// Lighting
Color ambient = Color(0, 0, 0);

struct Point_Light {
  Point3D pos;
  Color color;
};

Point_Light point_lights[250000]; // Limit point lights
int num_point_lights = 0;

struct Dir_Light {
  Dir3D dir;
  Color color;
};

Dir_Light dir_lights[250000]; // Limit directional lights
int num_dir_lights = 0;

struct Spot_Light {
  Point3D pos;
  Dir3D dir;
  Color color;
  Scalar angle1;
  Scalar angle2;
};

Spot_Light spot_lights[250000]; // Limit spot lights
int num_spot_lights = 0;

void parseSceneFile(std::string fileName){
  // Override the default values with new data from the file "fileName"

  FILE* fp = fopen(fileName.c_str(), "r");
  char line[1024];

  // Check if file opened
  if (fp == NULL) {
    printf("File %s failed to open", fileName.c_str());
  } else {
    
    // Loop over lines
    while (fgets(line, 1024, fp)) {
      // Skip comments
      if (line[0] == '#'){
        printf("Skipping comment: %s", line);
        continue;
      }

      // Get command
      char command[100];
      int fieldsRead = sscanf(line, "%s ", command);
      std::string commandStr = command;

      // Skip blank lines
      if (fieldsRead < 1) {
        continue;
      }

      // Image resolution
      if (commandStr == "film_resolution:") {
        int w, h;
        
        sscanf(line, "film_resolution: %d %d", &w, &h);
        img_width = w;
        img_height = h;
      }

      // Output image
      else if (commandStr == "output_image:") {
        char output[1024];
        
        sscanf(line, "output_image: %1023s", output);
        imgName = std::string(output);
      }

      // Camera position
      else if (commandStr == "camera_pos:") {
        Scalar x, y, z;
        
        sscanf(line, "camera_pos: %lf %lf %lf", &x, &y, &z);
        eye = Point3D(x, y, z);
      }

      // Camera forward
      else if (commandStr == "camera_fwd:") {
        Scalar fx, fy, fz;
        
        sscanf(line, "camera_fwd: %lf %lf %lf", &fx, &fy, &fz);
        forward_dir = Dir3D(fx, fy, fz);
      }

      // Camera up
      else if (commandStr == "camera_up:") {
        Scalar ux, uy, uz;
        
        sscanf(line, "camera_up: %lf %lf %lf", &ux, &uy, &uz);
        up_dir = Dir3D(ux, uy, uz);
      }

      // Field of view
      else if (commandStr == "camera_fov_ha:") {
        Scalar ha;
        
        sscanf(line, "camera_fov_ha: %lf", &ha);
        halfAngleVFOV = ha;
      }

      // Background
      else if (commandStr == "background:") {
        Scalar r, g, b;

        sscanf(line, "background: %lf %lf %lf", &r, &g, &b);
        background = Color(r, g, b);
      }

      // Ambient lighting
      else if (commandStr == "ambient_light:") {
        Scalar r, g, b;

        sscanf(line, "ambient_light: %lf %lf %lf", &r, &g, &b);
        ambient = Color(r, g, b);
      }

      // Material
      else if (commandStr == "material:") {
        Scalar ar, ag, ab, dr, dg, db, sr, sg, sb, ns, tr, tg, tb, ior;

        sscanf(line, "material: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &ar, &ag, &ab, &dr, &dg, &db, &sr, &sg, &sb, &ns, &tr, &tg, &tb, &ior);
        Material material = default_material();
        material.a = Color(ar, ag, ab);
        material.d = Color(dr, dg, db);
        material.s = Color(sr, sg, sb);
        material.ns = ns;
        material.t = Color(tr, tg, tb);
        material.ior = ior;
        current_material = material;
      }

      // Point light
      else if (commandStr == "point_light:") {
        Scalar r, g, b, x, y, z;

        sscanf(line, "point_light: %lf %lf %lf %lf %lf %lf", &r, &g, &b, &x, &y, &z);
        if (num_point_lights < max_objects) { // Add new point light unless there are 100
          point_lights[num_point_lights].pos = Point3D(x, y, z);
          point_lights[num_point_lights].color = Color(r, g, b);
          ++num_point_lights;
        }
      }

      // Direction light
      else if (commandStr == "directional_light:") {
        Scalar r, g, b, x, y, z;

        sscanf(line, "directional_light: %lf %lf %lf %lf %lf %lf", &r, &g, &b, &x, &y, &z);
        if (num_dir_lights < max_objects) { // Only add if there are less than 100
          dir_lights[num_dir_lights].dir = Dir3D(x, y, z);
          dir_lights[num_dir_lights].color = Color(r, g, b);
          ++num_dir_lights;
        }
      }

      // Spot light
      else if (commandStr == "spot_light:") {
        Scalar r, g, b, px, py, pz, dx, dy, dz, angle1, angle2;

        sscanf(line, "spot_light: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &r, &g, &b, &px, &py, &pz, &dx, &dy, &dz, &angle1, &angle2);
        if (num_spot_lights < max_objects) { // Only add if there are less than 100
          spot_lights[num_spot_lights].pos = Point3D(px, py, pz);
          spot_lights[num_spot_lights].dir = Dir3D(dx, dy, dz);
          spot_lights[num_spot_lights].color = Color(r, g, b);
          spot_lights[num_spot_lights].angle1 = angle1;
          spot_lights[num_spot_lights].angle2 = angle2;
          ++num_spot_lights;
        }
      }

      // Set max recursion depth
      else if (commandStr == "max_depth:") {
        int n;

        sscanf(line, "max_depth: %d", &n);
        max_depth = n;
      }

      // Hypersphere: cx cy cz cw r
      else if (commandStr == "hypersphere:") {
        Scalar cx, cy, cz, cw, r;

        sscanf(line, "hypersphere: %lf %lf %lf %lf %lf", &cx, &cy, &cz, &cw, &r);

        if (num_hyperspheres < max_objects) {
          Hypersphere hs;
          hs.center = ga4d::make_vec4(cx, cy, cz, cw);
          hs.radius = r;
          hs.material = current_material;

          hyperspheres[num_hyperspheres] = hs;
          ++num_hyperspheres;
        }
      }

      // 4D hyperplane: a x + b y + c z + d w + e = 0
      else if (commandStr == "hyperplane:") {
        pga_legacy::Scalar a, b, c, d, e;

        sscanf(line, "hyperplane: %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e);

        if (num_hyperplanes < max_objects) {
          HyperPlane hp;
          hp.normal = ga4d::make_vec4(a, b, c, d);
          hp.offset = e;
          hp.material = current_material;

          hyperplanes[num_hyperplanes] = hp;
          ++num_hyperplanes;
        }
      }

      else if (commandStr == "camera_w:") {
        Scalar w;
        sscanf(line, "camera_w: %lf", &w);
        camera_w = w;
      }

      else if (commandStr == "3D:") {
        char mode[32];
        sscanf(line, "3D: %31s", mode);

        if (strcmp(mode, "true") == 0) {
          anaglyph = true;
        } else {
          anaglyph = false;
        }
      }

      // Ignore other commands
      else {}
    }

    fclose(fp);
  }
  //Create an orthogonal camera basis, based on the provided up and right vectors
  // Normalize forward
  forward_dir = forward_dir.normalized();

  // Rejection of upward and forward to get orthogonal upward
  up_dir = (up_dir - dot(up_dir, forward_dir) * forward_dir).normalized();

  // Cross product for right direction to get orthogonal right vector
  right_dir = cross(up_dir, forward_dir).normalized();

  printf("Orthogonal Camera Basis:\n");
  forward_dir.print("forward");
  right_dir.print("right");
  up_dir.print("up");
}

  #endif
