
#ifndef ASSIGNMENT_5_PROJECT_BOX_H
#define ASSIGNMENT_5_PROJECT_BOX_H

#include <cmath>
#include <iostream>
#include <utility>
#include <algorithm>

#ifdef __APPLE__

#include <OpenGL/OpenGL.h>
#include <glut.h>

#else
#include <GL/glut.h>
#endif

#include "Vec3D.h"
#include "mesh.h"
#include "raytracing.h"


/**
 * An AABB Box class that is used for the k.d tree.
 */
class Box {
public:


    Vec3Df max;
    Vec3Df min;
    std::vector<Vertex> vertices = MyMesh.vertices;

    /**
     * Empty constructor
     */
    inline Box() = default;

    /**
     * Constructor to create an empty AABB.
     * @param newMax - The max of the AABB
     * @param newMin - The min of the AABB
     */
    inline Box(const Vec3Df &newMax, const Vec3Df &newMin) {
        max = newMax * 1, 00001;
        min = newMin * 0, 99999;

    }

    /**
     * Constructor to create a AABB around all triangles in 'triangles'.
     * @param triangles - The triangles that need to be surrounded
     */
    inline explicit Box(std::vector<Triangle> triangles) {

        if (triangles.empty()) {
            min = max = Vec3Df(0, 0, 0);
        }

        //Initialize max and min vectors
        float xMax = MyMesh.vertices.at(triangles[0].v[0]).p[0];
        float yMax = MyMesh.vertices.at(triangles[0].v[0]).p[1];
        float zMax = MyMesh.vertices.at(triangles[0].v[0]).p[2];

        float xMin = MyMesh.vertices.at(triangles[0].v[0]).p[0];
        float yMin = MyMesh.vertices.at(triangles[0].v[0]).p[1];
        float zMin = MyMesh.vertices.at(triangles[0].v[0]).p[2];

        for (Triangle triangle: triangles) {
            //Get corners of triangles
            Vec3Df A = MyMesh.vertices.at(triangle.v[0]).p;
            Vec3Df B = MyMesh.vertices.at(triangle.v[1]).p;
            Vec3Df C = MyMesh.vertices.at(triangle.v[2]).p;

            //If one of the coordinates is larger or smaller, update the max and min vector
            if (A.p[0] > xMax) xMax = A.p[0];
            if (A.p[1] > yMax) yMax = A.p[1];
            if (A.p[2] > zMax) zMax = A.p[2];

            if (A.p[0] < xMin) xMin = A.p[0];
            if (A.p[1] < yMin) yMin = A.p[1];
            if (A.p[2] < zMin) zMin = A.p[2];

            if (B.p[0] > xMax) xMax = B.p[0];
            if (B.p[1] > yMax) yMax = B.p[1];
            if (B.p[2] > zMax) zMax = B.p[2];

            if (B.p[0] < xMin) xMin = B.p[0];
            if (B.p[1] < yMin) yMin = B.p[1];
            if (B.p[2] < zMin) zMin = B.p[2];

            if (C.p[0] > xMax) xMax = C.p[0];
            if (C.p[1] > yMax) yMax = C.p[1];
            if (C.p[2] > zMax) zMax = C.p[2];

            if (C.p[0] < xMin) xMin = C.p[0];
            if (C.p[1] < yMin) yMin = C.p[1];
            if (C.p[2] < zMin) zMin = C.p[2];
        }

        //Initialize max and min vectors
        max = Vec3Df(xMax, yMax, zMax);
        min = Vec3Df(xMin, yMin, zMin);

    }

    /**
     * Constructor to create a AABB around all triangles in 'triangles'.
     * @param triangles - The triangles that need to be surrounded
     */
    inline explicit Box(std::vector<std::pair<Triangle, int>> triangles) {

        if (triangles.empty()) {
            min = max = Vec3Df(0, 0, 0);
        }

        float xMax = MyMesh.vertices.at(triangles[0].first.v[0]).p[0];
        float yMax = MyMesh.vertices.at(triangles[0].first.v[0]).p[1];
        float zMax = MyMesh.vertices.at(triangles[0].first.v[0]).p[2];

        float xMin = MyMesh.vertices.at(triangles[0].first.v[0]).p[0];
        float yMin = MyMesh.vertices.at(triangles[0].first.v[0]).p[1];
        float zMin = MyMesh.vertices.at(triangles[0].first.v[0]).p[2];

        for (auto &triangle: triangles) {
            Vec3Df A = MyMesh.vertices.at(triangle.first.v[0]).p;
            Vec3Df B = MyMesh.vertices.at(triangle.first.v[1]).p;
            Vec3Df C = MyMesh.vertices.at(triangle.first.v[2]).p;

            if (A.p[0] > xMax) xMax = A.p[0];
            if (A.p[1] > yMax) yMax = A.p[1];
            if (A.p[2] > zMax) zMax = A.p[2];

            if (A.p[0] < xMin) xMin = A.p[0];
            if (A.p[1] < yMin) yMin = A.p[1];
            if (A.p[2] < zMin) zMin = A.p[2];

            if (B.p[0] > xMax) xMax = B.p[0];
            if (B.p[1] > yMax) yMax = B.p[1];
            if (B.p[2] > zMax) zMax = B.p[2];

            if (B.p[0] < xMin) xMin = B.p[0];
            if (B.p[1] < yMin) yMin = B.p[1];
            if (B.p[2] < zMin) zMin = B.p[2];

            if (C.p[0] > xMax) xMax = C.p[0];
            if (C.p[1] > yMax) yMax = C.p[1];
            if (C.p[2] > zMax) zMax = C.p[2];

            if (C.p[0] < xMin) xMin = C.p[0];
            if (C.p[1] < yMin) yMin = C.p[1];
            if (C.p[2] < zMin) zMin = C.p[2];
        }

        max = Vec3Df(xMax, yMax, zMax);
        min = Vec3Df(xMin, yMin, zMin);


    }

    /**
     * Checks if the ray intersects with the box by checking if the ray.
     * intersects with two two-dimensional planes (XY and XZ).
     * @param orig: origin vector of the ray.
     * @param dir:  direction vector of the ray.
     * @return returns true if the box intersects with the ray.
     */
    inline bool intersect(Vec3Df orig, Vec3Df dir) const {

        Vec3Df inverse_direction;
        inverse_direction.normalize();
        inverse_direction[0] = 1 / dir[0];
        inverse_direction[1] = 1 / dir[1];
        inverse_direction[2] = 1 / dir[2];
        //if there is no direction return false
        if (dir.getLength() == 0)
            return false;

        Vec3Df min = getMin();
        Vec3Df max = getMax();
        //txmin & txmax for x axis
        float txmin = (min[0] - orig[0]) * inverse_direction[0];
        float txmax = (max[0] - orig[0]) * inverse_direction[0];

        //if txmin is bigger, swap them around

        if (txmin > txmax) {
            float temp = txmin;
            txmin = txmax;
            txmax = temp;
        }


        float tymin = (min[1] - orig[1]) * inverse_direction[1];
        float tymax = (max[1] - orig[1]) * inverse_direction[1];

        //if tymin is bigger, swap them around

        if (tymin > tymax) {
            float temp = tymin;
            tymin = tymax;
            tymax = temp;
        }

        //return false if the ray doesnt intersect through the xy plane
        if ((txmin > tymax || tymin > txmax))
            return false;

        float tzmin = (min[2] - orig[2]) * inverse_direction[2];
        float tzmax = (max[2] - orig[2]) * inverse_direction[2];

        //if tzmin is bigger, swap them around

        if (tzmin > tzmax) {
            float temp = tzmin;
            tzmin = tzmax;
            tzmax = temp;
        }

        //return false if the ray doesnt intersect through the xz plane
        return !((tzmax < txmin) || (txmax < tzmin));
        //if all checks pass, the box intersects

    }


    /**
     * Calculates the maximum float of three inputs
     */
    inline float Max(float a, float b, float c) {
        float res = a;
        if (b > a) { res = b; }
        if (c > res) { res = c; }
        return res;
    }

    /**
     * Calculates the minimum float of three inputs
     */
    inline float Min(float a, float b, float c) {
        float res = a;
        if (b < a) { res = b; }
        if (c < res) { res = c; }
        return res;
    }


    /**
     * Checks if the box intersects with a point using min-max testing.
     * @param point the point that has to be checked.
     * @return returns true if the point intersects the box.
     */
    inline bool intersect(Vec3Df point) {
        if (point[0] < getMin()[0] && point[0] > getMax()[0])
            return false;

        if (point[1] < getMin()[1] && point[1] > getMax()[1])
            return false;

        return !(point[2] < getMin()[2] && point[2] > getMax()[2]);

    }


    /**
     * Draws an outline along the edges of the box
     */
    inline void draw(int r, int g, int b) {
        Vec3Df center = getCenter();

        glPushMatrix();
        glColor3f(r, g, b);
        glTranslatef(center[0], center[1], center[2]);
        glScalef(getPointH()[0] - getPointA()[0], getPointH()[1] - getPointA()[1], getPointH()[2] - getPointA()[2]);
        glutWireCube(1);
        glPopMatrix();
    }




    //Orientation of all points is based on that the minimum is point a and the maximum is point h
    /*
           e-------f
          /|      /|
         / |     / |
        g--|----h  |
        |  a----|--b
        | /     | /
        d-------c
     */

    /* These are the basic functions that are used
    *  for the math associated with the box.
    *
    */
    inline Vec3Df getPointA() {
        return getMin();

    }

    inline Vec3Df getPointB() {
        Vec3Df result = getMin();
        result[0] = result[0] + getLengthX();
        return result;


    }

    inline Vec3Df getPointC() {
        Vec3Df result = getMin();
        result[0] = result[0] + getLengthX();
        result[2] = result[2] + getLengthZ();
        return result;
    }

    inline Vec3Df getPointD() {
        Vec3Df result = getMin();
        result[2] = result[2] + getLengthZ();
        return result;
    }

    inline Vec3Df getPointE() {
        Vec3Df result = getMin();
        result[1] = result[1] + getLengthY();
        return result;
    }

    inline Vec3Df getPointF() {
        Vec3Df result = getMin();
        result[0] = result[0] + getLengthX();
        result[1] = result[1] + getLengthY();
        return result;
    }

    inline Vec3Df getPointG() {
        Vec3Df result = getMin();
        result[2] = result[2] + getLengthZ();
        result[1] = result[1] + getLengthY();
        return result;
    }

    inline Vec3Df getPointH() {
        return getMax();
    }

    inline float getLengthX() {
        return abs(max[0] - min[0]);

    }

    inline float getLengthY() {
        return abs(max[1] - min[1]);
    }

    inline float getLengthZ() {
        return abs(max[2] - min[2]);
    }

    inline Vec3Df getCenter() {
        Vec3Df center((min + max) / 2);
        return center;
    }

    const Vec3Df getMax() const {
        return max;
    }

    const Vec3Df getMin() const {
        return min;
    }

    const float getVolume() {
        return getLengthX() * getLengthY() * getLengthZ();
    }
};


#endif //ASSIGNMENT_5_PROJECT_BOX_H
