//
// Created by Huib sprangers on 14-06-18.
//

#ifndef ASSIGNMENT_5_PROJECT_TREE_H
#define ASSIGNMENT_5_PROJECT_TREE_H

#include <utility>
#include <cmath>
#include <iostream>
#ifdef __APPLE__

#include <OpenGL/OpenGL.h>
#include <glut.h>

#else
#include <GL/glut.h>
#endif

#include "Vec3D.h"
#include "mesh.h"
#include "raytracing.h"
#include "Box.h"
#include "Node.h"


//
// A tree for acceleration
//
class Tree {
private:
    //get Triangles left of an axis aligned line, which can be simpliefied to a point
    inline static std::vector<std::pair<Triangle, int>>
    getTrianglesLeftofPoint(float point, int axis, std::vector<std::pair<Triangle, int>> triangles) {
        std::vector<std::pair<Triangle, int>> result;
        for (int i = 0; i < triangles.size(); i++) {
            Triangle triangle = triangles[i].first;
            if (isTriangleLeftofPoint(point, axis, triangle))
                result.push_back(triangles[i]);
        }
        return result;
    }
    //get Triangles right of an axis aligned line, which can be simpliefied to a point
    inline static std::vector<std::pair<Triangle, int>>
    getTrianglesRightofPoint(float point, int axis, std::vector<std::pair<Triangle, int>> triangles) {
        std::vector<std::pair<Triangle, int>> result;
        for (int i = 0; i < triangles.size(); i++) {
            Triangle triangle = triangles[i].first;
            if (isTriangleRightofPoint(point, axis, triangle))
                result.push_back(triangles[i]);
        }
        return result;
    }

    inline static bool isTriangleLeftofPoint(float point, int axis, Triangle triangle) {
        return getMinInAxis(axis, triangle) <= point;
    }

    inline static bool isTriangleRightofPoint(float point, int axis, Triangle triangle) {
        return getMaxInAxis(axis, triangle) >= point;
    }

    //get max float in an axis from a triangle
    inline static float getMaxInAxis(int axis, Triangle triangle) {
        Vec3Df A = MyMesh.vertices.at(triangle.v[0]).p;
        Vec3Df B = MyMesh.vertices.at(triangle.v[1]).p;
        Vec3Df C = MyMesh.vertices.at(triangle.v[2]).p;
        float max = A[axis];
        if (B[axis] > max)
            max = B[axis];
        if (C[axis] > max)
            max = C[axis];
        return max;


    }
    //get min float in an axis from a triangle
    inline static float getMinInAxis(int axis, Triangle triangle) {
        Vec3Df A = MyMesh.vertices.at(triangle.v[0]).p;
        Vec3Df B = MyMesh.vertices.at(triangle.v[1]).p;
        Vec3Df C = MyMesh.vertices.at(triangle.v[2]).p;
        float min = A[axis];
        if (B[axis] < min)
            min = B[axis];
        if (C[axis] < min)
            min = C[axis];
        return min;

    }

    //recursive call to add potentional intersecting triangles to a list. potential triangles are triangles that are in leaves
    inline static void
    getLeaveTriangles(std::vector<std::pair<Triangle, int>> &targetTriangle, Node &node, Vec3Df origin, Vec3Df dir) {

        if (node.hasLeft() && node.getLeft()->box.intersect(origin, dir)) {
            getLeaveTriangles(targetTriangle, *node.getLeft(), origin, dir);

        }
        if (node.hasRight() && node.getRight()->box.intersect(origin, dir)) {
            getLeaveTriangles(targetTriangle, *node.getRight(), origin, dir);

        }

        if (node.isLeaf()) {
            for (int i = 0; i < node.triangles.size(); i++) {
                targetTriangle.push_back(node.triangles[i]);
            }
        }
    }

    //construct kd tree
    inline void makeTree(Node &parent) {
        int axis = 0;
        if (viableToSplit(parent.depth))
            makeChildren(parent, parent.depth, axis);


    }

    //calculates point where to split the box in two
    inline float whereToSplit(int axis, Node node) {
        switch (axis) {
            case 0: {
                Triangle triangleX = node.xMedian();
                Vec3Df Ax = MyMesh.vertices.at(triangleX.v[0]).p;
                Vec3Df Bx = MyMesh.vertices.at(triangleX.v[1]).p;
                Vec3Df Cx = MyMesh.vertices.at(triangleX.v[2]).p;
                return node.xMedian().centroid(Ax, Bx, Cx)[0];

            }


            case 1: {
                Triangle triangleY = node.yMedian();
                Vec3Df Ay = MyMesh.vertices.at(triangleY.v[0]).p;
                Vec3Df By = MyMesh.vertices.at(triangleY.v[1]).p;
                Vec3Df Cy = MyMesh.vertices.at(triangleY.v[2]).p;
                return node.yMedian().centroid(Ay, By, Cy)[1];
            }

            case 2: {

                Triangle triangleZ = node.zMedian();
                Vec3Df Az = MyMesh.vertices.at(triangleZ.v[0]).p;
                Vec3Df Bz = MyMesh.vertices.at(triangleZ.v[1]).p;
                Vec3Df Cz = MyMesh.vertices.at(triangleZ.v[2]).p;
                return node.zMedian().centroid(Az, Bz, Cz)[2];
            }
        }

    }

    //checks if it is viable to split boxes based on depth
    inline bool viableToSplit(int depth) {

        return depth < 5;
    }

    inline void drawTreeRecursive(Node node) {
        if(node.isLeaf()) {
            node.box.draw(1, 1, 1);
        }
            if (node.hasLeft())
                drawTreeRecursive(*node.getLeft());
            if (node.hasRight())
                drawTreeRecursive(*node.getRight());
        }



public:
    Node root;

    Tree() = default;


    explicit Tree(Node &root) {
        this->root = root;

        makeTree(this->root);
    }


    //draw outlines for debugging purposes
    inline void drawOutline() {
        drawTreeRecursive(root);
    }


    //make childs of parent node
    inline void makeChildren(Node &parent, int depth, int axis) {
        Box parentBox = parent.box;
        float point = whereToSplit(axis, parent);

        std::vector<std::pair<Triangle, int>> leftTriangles =getTrianglesLeftofPoint(point,axis,parent.triangles);
        Box LeftBox = Box(leftTriangles);
        Node *left = new Node(LeftBox, depth + 1, leftTriangles);



        std::vector<std::pair<Triangle, int>> rightTriangles =getTrianglesRightofPoint(point,axis,parent.triangles);
        Box RightBox = Box(rightTriangles);
        Node *right = new Node(RightBox, depth + 1, rightTriangles);




        int splitAxis = depth % 3;

        if (right->triangles.size() > 0 && viableToSplit(parent.depth)) {
            parent.setRight(right);
            makeChildren(*right, right->depth, splitAxis);
        }
        if (left->triangles.size() > 0 && viableToSplit(parent.depth)) {
            parent.setLeft(left);
            makeChildren(*left, left->depth, splitAxis);
        }




    }
    //base call to calculate potential triangles. See the private recursive call for more info
    inline std::vector<std::pair<Triangle, int>> calculatePossibleTriangles(const Vec3Df &origin, const Vec3Df &dir) {
        std::vector<std::pair<Triangle, int>> possibleTriangles;
        getLeaveTriangles(possibleTriangles, root, origin, dir);
        return possibleTriangles;

    }


};
#endif