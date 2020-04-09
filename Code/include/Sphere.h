#ifndef ASSIGNMENT_5_PROJECT_SPHERE_H
#define ASSIGNMENT_5_PROJECT_SPHERE_H

#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <utility>
#include <vector>
#include <math.h>

#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <glut.h>
#else

#include <GL/glut.h>

#endif

#include "Vec3D.h"
#include "mesh.h"
#include "raytracing.h"

class Sphere {
protected:
    Material material;
    Vec3Df center;
    float radius;

public:
    /*
    * empty constructor for sphere
    */
    inline Sphere(void) {
    }

    /*
    * a default constructor to just make a sphere.
    * makes the color grey by default.
    */
    inline Sphere(Vec3Df newCenter, float newRadius) {
        center = newCenter;
        radius = newRadius;
        Material defaultMat;
        defaultMat.set_Kd(0.5f, 0.5f, 0.5f);
        material = defaultMat;
    }

    /*
    * a default constructor to just make a sphere with a certain color.
    */
    inline Sphere(Vec3Df newCenter, float newRadius, Vec3Df color) {
        center = newCenter;
        radius = newRadius;
        Material defaultMat;
        defaultMat.set_Kd(color[0], color[1], color[2]);
        material = defaultMat;
    }

    /*
    * a default constructor to just make a sphere along with a certain material.
    */
    inline Sphere(Vec3Df newCenter, float newRadius, Material newMaterial) {
        center = newCenter;
        radius = newRadius;
        material = newMaterial;
    }

    /*
    * Method to have OpenGL draw the Sphere at a different location.
    */
    void draw(GLfloat x, GLfloat y, GLfloat z) {
        setCenter(Vec3Df(x, y, z));
        draw();
    }

    /*
    * Method to have OpenGL draw the Sphere.
    */
    void draw() {
        Vec3Df color = material.Kd();
        if(material.illum() >= 3)
            color = material.Ka();

        glPushMatrix();
        glColor3f(color[0], color[1], color[2]);
        glTranslatef(center[0], center[1], center[2]);
        glRotatef(90, 1, 0, 0);
        glutWireSphere(radius, 12, 24);
        glPopMatrix();
    }

    /**
    * Standard getter for the coordinates of the center.
    */
    inline Vec3Df getCenter() {
        return center;
    }

    /**
    * Standard getter for the coordinates of the center.
    */
    inline void setCenter(Vec3Df newCenter) {
        center = newCenter;
    }

    /**
    * Standard getter for the radius.
    */
    inline float getRadius() {
        return radius;
    }

    /**
    * Standard setter for the radius.
    */
    inline void setRadius(float newRadius) {
        radius = newRadius;
    }

    /*
    * Standard getter for the material.
    */
    inline Material getMaterial() {
        return material;
    }

    /*
    * Standard setter for the material.
    */
    inline void setMaterial(Material newMaterial) {
        material = newMaterial;
    }

    /**
    * Method which checks whether a ray intersects with the sphere.
    *
    * implementation taken form https://www.scratchapixel.com/code.php?id=10&origin=/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes
    * and edited to better serve the implementation of our raytracer
    */
    inline bool intersect(const Vec3Df &orig, const Vec3Df &dir, float *pointer, Vec3Df *normal) const {
        float t0, t1, r = radius * radius;
        Vec3Df L = center - orig; //distance to center from origin
        float tca = Vec3Df::dotProduct(L, dir);    //distance to middle of intersectionpoints
        if (tca < 0) return false;    //if this distance is negative, it does not exist(or is behind ray)
        float d = Vec3Df::dotProduct(L, L) - (tca * tca);    //distance from center to tca*dir
        if (d > r) return false;        //if this distance is bigger than the radius, ray does not hit
        float thc = sqrt(r - d);        //distance from d to intersection points
        t0 = tca - thc;            //intersection point first hit by the ray
        t1 = tca + thc;            //intersection point hit through the sphere
        if (t0 > t1) {
            float temp = t0;
            t0 = t1;
            t1 = temp;
        }

        if (t0 < 0.001f) {
            t0 = t1;
            if (t0 < 0.001f) return false;
        }
        Vec3Df P = orig + t0 * dir;
        Vec3Df N = (P - center);
        N.normalize();


        (*normal) = N;
        (*pointer) = t0;
        return true;
    }

    /*
    * a method to return the normal at a certain point.
    */
    inline Vec3Df getNormal(Vec3Df orig, Vec3Df des) {
        float pointer = 0;
        Vec3Df normal = 0;
        if (intersect(orig, des, &pointer, &normal)) {
            return normal;
        } else
            return Vec3Df(0, 0, 0);
    }
};

#endif //ASSIGNMENT_5_PROJECT_SPHERE_H