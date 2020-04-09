#ifndef RAYTRACING_H_1
#define RAYTRACING_H_1
#include <vector>
#include "mesh.h"

//Welcome to your MAIN PROJECT...
//THIS IS THE MOST relevant code for you!
//this is an important file, raytracing.cpp is what you need to fill out
//In principle, you can do the entire project ONLY by working in these two files

extern Mesh MyMesh; //Main mesh
extern std::vector<Vec3Df> MyLightPositions;
extern std::vector<Vec3Df> virtualLightPositions;
extern Vec3Df MyCameraPosition; //currCamera
extern unsigned int WindowSize_X;//window resolution width
extern unsigned int WindowSize_Y;//window resolution height
extern unsigned int RayTracingResolutionX;  // largeur fenetre
extern unsigned int RayTracingResolutionY;  // largeur fenetre
extern float lightIntensity;
extern Vec3Df lightColor;
extern bool isSpecularEnabled;
extern bool isReflectiveEnabled;
extern bool isRefractedEnabled;
extern unsigned long AmountOfIntersect;

//use this function for any preprocessing of the mesh.
void init();

//function to calculate the normal vector of a triangle
Vec3Df calcNormal(std::vector<Vertex> &vertices, const Triangle &hitTriangle);

//you can use this function to transform a click to an origin and destination
//the last two values will be changed. There is no need to define this function.
//it is defined elsewhere
void produceRay(int x_I, int y_I, Vec3Df & origin, Vec3Df & dest);

//your main function to rewrite
Vec3Df performRayTracing(const Vec3Df & camera, const Vec3Df & destination, const int &depth);

//function for refraction
Vec3Df refraction(const Vec3Df &hitPoint, const Vec3Df &direction, const Vec3Df &normal, const int &depth, const float &ior) ;

// The Fresnel equation: ration of reflected light.
void fresnel(const Vec3Df &direction, const Vec3Df &normal, const float &ior, float *kr);

//function for reflection
Vec3Df reflection(const Vec3Df &hitPoint, const Vec3Df &direction, const Vec3Df &normal, const int &depth);

//function for blinn phong specularities
float blinnPhongSpecular(const std::vector<Vertex> &vertices, std::vector<Vec3Df> *directlyVisibleLights, const Vec3Df &hitPoint, const Vec3Df &normal, Material &material);

//fucntion for diffuse lighting
float diffusion(const std::vector<Vertex> &vertices, std::vector<Vec3Df> *directlyVisibleLights, const Vec3Df &hitPoint, const Vec3Df &normal) ;

//function for getting hard shadows
float hardShadows(const std::vector<Vertex> &vertices, std::vector<Vec3Df> *directlyVisibleLights, const Vec3Df &hitPoint);

//a function to debug --- you can draw in OpenGL here
void yourDebugDraw();

//want keyboard interaction? Here it is...
void yourKeyboardFunc(char t, int x, int y, const Vec3Df & rayOrigin, const Vec3Df & rayDestination);

#endif