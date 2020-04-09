//
// Created by javd on 26-6-18.
//

#ifndef ASSIGNMENT_5_PROJECT_NODE_H
#define ASSIGNMENT_5_PROJECT_NODE_H

#include <algorithm>
#include "mesh.h"
#include "Box.h"


class Node {

public:


    Box box;
    int depth=-1;

    //childs of the node
    std::vector<Node *> Nodes;

    Node *getRight() const {
        return Nodes[1];
    }

    Node *getLeft() const {
        return Nodes[0];
    }




    //bool for isLeaf function
    bool leftSet =false;
    bool rightSet = false;


    void setLeft(Node* left) {
        leftSet=true;
        Nodes.push_back(left);
    }

    void setRight(Node* right) {
        rightSet=true;
        Nodes.push_back(right);
    }


    //triangles in the box, sorted in every axis
    std::vector<std::pair<Triangle, int>> triangles;
    std::vector<std::pair<Triangle, int>> Xaxissorted;
    std::vector<std::pair<Triangle, int>> Yaxissorted;
    std::vector<std::pair<Triangle, int>> Zaxissorted;


    Node(){
        leftSet = false;
        rightSet = false;
    };
    Node(Box box, int depth) {

        leftSet = false;
        rightSet = false;
        this->box = box;
        this->depth = depth;

    }

    Node(Box box, int depth, std::vector<Triangle> &triangles) {

        leftSet = false;
        rightSet = false;
        this->box = box;
        this->depth = depth;
        for(int i=0; i< triangles.size();i++) {
            std::pair<Triangle, int> pair = std::pair<Triangle, int>(triangles[i], i);
            this->triangles.push_back(pair);
        }
            xSortVector(this->triangles);
            ySortVector(this->triangles);
            zSortVector(this->triangles);

    }

    Node(Box box, int depth, std::vector<std::pair<Triangle,int>> &triangles) {

        leftSet = false;
        rightSet = false;
        this->box = box;
        this->depth = depth;
        for(int i=0; i< triangles.size();i++) {
            this->triangles.push_back(triangles[i]);
        }
        xSortVector(this->triangles);
        ySortVector(this->triangles);
        zSortVector(this->triangles);

    }
    //sort comparator for sorting triangle lists on every axis
    inline static bool xSortTriangle(std::pair<Triangle, int> &t1, std::pair<Triangle, int> &t2) {

        Vec3Df t1A = MyMesh.vertices.at(t1.first.v[0]).p;
        Vec3Df t1B = MyMesh.vertices.at(t1.first.v[1]).p;
        Vec3Df t1C = MyMesh.vertices.at(t1.first.v[2]).p;


        Vec3Df t2A = MyMesh.vertices.at(t2.first.v[0]).p;
        Vec3Df t2B = MyMesh.vertices.at(t2.first.v[1]).p;
        Vec3Df t2C = MyMesh.vertices.at(t2.first.v[2]).p;

        Vec3Df t1Centroid = t1.first.centroid(t1A,t1B, t1C);
        Vec3Df t2Centroid = t2.first.centroid(t2A, t2B, t2C);
        return t1Centroid[0]<t2Centroid[0];
    }
    
    inline static bool ySortTriangle(std::pair<Triangle, int> &t1, std::pair<Triangle, int> &t2) {
        Vec3Df t1A = MyMesh.vertices.at(t1.first.v[0]).p;
        Vec3Df t1B = MyMesh.vertices.at(t1.first.v[1]).p;
        Vec3Df t1C = MyMesh.vertices.at(t1.first.v[2]).p;


        Vec3Df t2A = MyMesh.vertices.at(t2.first.v[0]).p;
        Vec3Df t2B = MyMesh.vertices.at(t2.first.v[1]).p;
        Vec3Df t2C = MyMesh.vertices.at(t2.first.v[2]).p;

        Vec3Df t1Centroid = t1.first.centroid(t1A,t1B, t1C);
        Vec3Df t2Centroid = t2.first.centroid(t2A, t2B, t2C);
        return t1Centroid[1]<t2Centroid[1];
    }

    inline static bool zSortTriangle(std::pair<Triangle, int> &t1, std::pair<Triangle, int> &t2) {
        Vec3Df t1A = MyMesh.vertices.at(t1.first.v[0]).p;
        Vec3Df t1B = MyMesh.vertices.at(t1.first.v[1]).p;
        Vec3Df t1C = MyMesh.vertices.at(t1.first.v[2]).p;


        Vec3Df t2A = MyMesh.vertices.at(t2.first.v[0]).p;
        Vec3Df t2B = MyMesh.vertices.at(t2.first.v[1]).p;
        Vec3Df t2C = MyMesh.vertices.at(t2.first.v[2]).p;

        Vec3Df t1Centroid = t1.first.centroid(t1A,t1B, t1C);
        Vec3Df t2Centroid = t2.first.centroid(t2A, t2B, t2C);
        return t1Centroid[2]<t2Centroid[2];
    }


    //actual sorting of vector on every axis
    inline void xSortVector(std::vector<std::pair<Triangle, int>> &triangles) {

        std::sort(triangles.begin(), triangles.end(), xSortTriangle);
        Xaxissorted = triangles;
    }



    inline void ySortVector(std::vector<std::pair<Triangle, int>> &triangles) {

        std::sort(triangles.begin(), triangles.end(), ySortTriangle);
        Yaxissorted = triangles;
    }

    inline void zSortVector(std::vector<std::pair<Triangle, int>> &triangles) {

        std::sort(triangles.begin(), triangles.end(), zSortTriangle);
        Zaxissorted = triangles;
    }


    //calculate medians for every axis
    inline Triangle xMedian() {
        long median =Xaxissorted.size()/2;
        return Xaxissorted[median].first;
    }
    inline Triangle yMedian() {
        long median =Yaxissorted.size()/2;
        return Yaxissorted[median].first;
    }
    inline Triangle zMedian() {
        long median =Zaxissorted.size()/2;
        return Zaxissorted[median].first;
    }






    inline bool hasLeft() {
        return leftSet;
    }

    inline bool hasRight() {
        return rightSet;
    }
    inline bool isLeaf() {
        return !hasLeft()&&!hasRight();
    }


    //check the best axis to split the box in two
    inline int bestSplitAxis() {
        float xLength = box.getLengthX();
        float yLength = box.getLengthY();
        float zLength = box.getLengthZ();
        if (xLength > yLength && xLength > zLength)
            return 0;
        if (yLength > zLength && yLength > xLength)
            return 1;
        return 2;
    }


};
#endif //ASSIGNMENT_5_PROJECT_NODE_H
