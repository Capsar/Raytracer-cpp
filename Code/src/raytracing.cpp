#include <stdio.h>
#include <Box.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <Node.h>
#include <Tree.h>

#else

#include <GL/glut.h>
#include <limits>
#include <Node.h>
#include <Tree.h>

#endif
#ifdef WIN32

#include <windows.h>
#include <Box.h>


#endif
#include "Sphere.h"
#include "raytracing.h"


//temporary variables
//these are only used to illustrate
//a simple debug drawing. A ray
Vec3Df testRayOrigin;
Vec3Df testRayDestination;
Box all;
Tree tree;
std::vector<Triangle> triangles;
std::vector<Sphere> spheres;
Node root;
int previousIndex = -1;


Vec3Df calcNormal(std::vector<Vertex> &vertices, const Triangle &hitTriangle);



const float ambient = 0.25;
const int reflectionDepth = 8;
const int reflectionIllum = 3;
const int refractionIllum = 6;



//use this function for any preprocessing of the mesh.
void init() {
    //load the mesh file
    //please realize that not all OBJ files will successfully load.
    //Nonetheless, if they come from Blender, they should, if they
    //are exported as WavefrontOBJ.
    //PLEASE ADAPT THE LINE BELOW TO THE FULL PATH OF THE dodgeColorTest.obj
    //model, e.g., "C:/temp/myData/GraphicsIsFun/dodgeColorTest.obj",
    //otherwise the application will not load properly

    MyMesh.loadMesh("newSimpleSceneRefraction.obj", true);

    MyMesh.computeVertexNormals();

    //one first move: initialize the first light source
    //at least ONE light source has to be in the scene!!!
    //here, we set it to the current location of the camera
    MyLightPositions.push_back(MyCameraPosition);
    all = Box(MyMesh.triangles);
    root = Node(all, 0, MyMesh.triangles);
    tree = Tree(root);





    //list of basic controls of scene
    std::cout<<"c button: simulate a Sphere Light"<<std::endl;
    std::cout<<"v button: simulate a Box Light"<<std::endl;
    std::cout<<"b button: go back to Point light"<<std::endl;
    std::cout<<"z button: decrease the amount of virtual lights by 1 for each light source,"<<std::endl;
    std::cout<<"x button: increase the amount of virtual lights by 1 for each light source,"<<std::endl;
    std::cout<<"1 button: decrease the light intensity by 0.1"<<std::endl;
    std::cout<<"2 button: increase the light intensity by 0.1"<<std::endl;
    std::cout<<"8 button: increase the red light color by 0.1"<<std::endl;
    std::cout<<"9 button: increase the green light color by 0.1"<<std::endl;
    std::cout<<"0 button: increase the blue light color by 0.1"<<std::endl;
    std::cout<<"s button: toggles specularity on and off"<<std::endl;
    std::cout<<"a button: toggles refraction on and off"<< std::endl;
    std::cout<<"d button: toggles reflection on and off"<<std::endl;


    Material sphereMaterial;
    sphereMaterial.set_Ka(1, 1, 1);
    sphereMaterial.set_Kd(0.05, 0.001, 0.001);
    sphereMaterial.set_Ks(0.005, 0.003, 0.004);
    sphereMaterial.set_Ni(1.5);
    sphereMaterial.set_Ns(94);
    sphereMaterial.set_illum(3);

    Sphere testSphere1 = Sphere(Vec3Df(0,4,0), 1.0f, sphereMaterial);
    Material sphereMaterial2;
    sphereMaterial2.set_Ka(1, 1, 1);
    sphereMaterial2.set_Kd(0.05, 0.001, 0.001);
    sphereMaterial2.set_Ks(0.005, 0.003, 0.004);
    sphereMaterial2.set_Ni(1.25);
    sphereMaterial2.set_Ns(94);
    sphereMaterial2.set_illum(6);
    Sphere testSphere2 = Sphere(Vec3Df(-2,1.2,-2), 0.6f, sphereMaterial2);


    Material sphereMaterial3;
    sphereMaterial3.set_Ka(1, 0.2, 0.5);
    sphereMaterial3.set_Kd(0.001, 0.001, 0.1);
    sphereMaterial3.set_Ks(0.005, 0.003, 0.004);
    sphereMaterial3.set_Ni(1.125);
    sphereMaterial3.set_Ns(94);
    sphereMaterial3.set_illum(6);
    Sphere testSphere3 = Sphere(Vec3Df(-1,1.4,1), 0.6f, sphereMaterial3);

    spheres.push_back(testSphere1);
    spheres.push_back(testSphere2);
    spheres.push_back(testSphere3);


}

//return the color of your pixel.
/**
 *
 * @param origin: origin Vector.
 * @param dir: directional vector.
 * @param A: A vertex of triangle ABC.
 * @param B: B vertex of triangle ABC.
 * @param C: C vertex of triangle ABC.
 * @param t; memory adress of the hit.
 * @return(bool):returns if a ray interects the triangle, else false.
 */
bool rayIntersect(const Vec3Df &origin, const Vec3Df &dir, const std::vector<Vertex> &vertices,
                  const Triangle &triangle,
                  float *t) {
    AmountOfIntersect+=1;
    Vec3Df A = vertices.at(triangle.v[0]).p;
    Vec3Df B = vertices.at(triangle.v[1]).p;
    Vec3Df C = vertices.at(triangle.v[2]).p;

    //calculating determent
    const Vec3Df AB = B - A;
    const Vec3Df AC = C - A;
    const Vec3Df q = Vec3Df::crossProduct(dir, AC);
    const float det = Vec3Df::dotProduct(AB, q);

    //checking error margin
    const float epsilon = 0.0001f;

    if (det > -epsilon && det < epsilon) {
        return false;
    }
    //calculating barycentric coordinates
    const float f = 1 / det;
    const Vec3Df s = origin - A;
    float u = f * Vec3Df::dotProduct(s, q);
    if (u < 0 || u > 1.0) {
        return false;
    }
    const Vec3Df r = Vec3Df::crossProduct(s, AB);
    const float v = f * Vec3Df::dotProduct(dir, r);
    if (v < 0 || u + v > 1) {
        return false;
    }
    //store hit in memory adress
    (*t) = f * Vec3Df::dotProduct(AC, r);
    if((*t) > 0.001f)
        return true;
    return false;
}

Vec3Df calcNormal(std::vector<Vertex> &vertices, const Triangle &hitTriangle) {
    Vec3Df A = vertices.at(hitTriangle.v[0]).p;
    Vec3Df B = vertices.at(hitTriangle.v[1]).p;
    Vec3Df C = vertices.at(hitTriangle.v[2]).p;
    Vec3Df AB = B - A;
    Vec3Df AC = C - A;
    Vec3Df normal = Vec3Df::crossProduct(AB, AC);
    normal.normalize(); // Normal of the triangle the hitPoint is on.
    return normal;
}

//Used to clip numbers off if they're too big/small (used in refraction)
float clamp(float lowerLimit, float upperLimit, float number) {
	if (number > upperLimit) {
		return upperLimit;
	}
	if (number < lowerLimit) {
		return lowerLimit;
	}
	return number;
}


Vec3Df performRayTracing(const Vec3Df &camera, const Vec3Df &destination, const int &depth) {

    auto &vertices = MyMesh.vertices;
    Vec3Df direction = destination - camera;
    direction.normalize();
    float last_t = std::numeric_limits<float>::max();
    Triangle hitTriangle;
    Vec3Df normal;
    Vec3Df hitPoint;
    Material material;

    std::vector<std::pair<Triangle,int>> possibleTriangles = tree.calculatePossibleTriangles(camera, direction);
    int sphereSize = spheres.size();
    int triangleSize = possibleTriangles.size();
    float t = 0;


    #pragma omp parallel for
    for (int i = -sphereSize; i < triangleSize; i++) {
        t = 0;

        if (i >= 0) {
            std::pair<Triangle, int> pair = possibleTriangles.at(i);
            //triangle points
            Triangle triangle = pair.first;
            bool intersect = rayIntersect(camera, direction, vertices, triangle, &t);
            if (intersect && t < last_t) {
                last_t = t;
                hitTriangle = triangle;
                normal = calcNormal(vertices, hitTriangle);
                int materialIndex = MyMesh.triangleMaterials.at(pair.second);
                material = MyMesh.materials.at(materialIndex);
            }
        } else if (i < 0) {
            int ii = (i + sphereSize);
            Sphere sphere = spheres.at(ii);
            bool sphereIntersect = sphere.intersect(camera, direction, &t, &normal);
            if (sphereIntersect && t < last_t) {
                last_t = t;
                material = sphere.getMaterial();
            }
        }
    }

    //if no intersect found, return a black vector
    if (last_t == std::numeric_limits<float>::max()) {
        return Vec3Df(0, 0, 0);
    }


    hitPoint = camera + last_t * direction; //The point that the rayTrace hits.

    Vec3Df usedlightColor = lightColor;

    usedlightColor *= lightIntensity;

    // Material index of vertex
    const unsigned illum = material.illum();
    Vec3Df color = material.Kd();
    Vec3Df ambientLight = ambient*usedlightColor;

    std::vector<Vec3Df> directlyVisibleLights = MyLightPositions;
    float shadow = hardShadows(vertices, &directlyVisibleLights, hitPoint);

    Vec3Df diffuseModifier = Vec3Df(0,0,0);
    Vec3Df specularModifier = Vec3Df(0,0,0);
    Vec3Df reflectionModifier = Vec3Df(0,0,0);
	Vec3Df refractionModifier = Vec3Df(0,0,0);
    float Kr = 1;

    if(shadow > 0 || illum >= reflectionIllum)   {
        if (isSpecularEnabled) {
            float specular = blinnPhongSpecular(vertices, &directlyVisibleLights, hitPoint, normal, material);
            specularModifier += lightColor*material.Ks()*specular;
        }
        float diff = diffusion(vertices, &directlyVisibleLights, hitPoint, normal);
        diffuseModifier += lightColor*diff;

        if(illum < reflectionIllum) {
            diffuseModifier *= shadow;
            specularModifier *= shadow;
        }
    }


    if(illum >= reflectionIllum && depth < reflectionDepth) {
        bool outside = Vec3Df::dotProduct(direction, normal) < 0;
        Vec3Df bias = 0.0001f * normal;


        // Check for refraction
        if(illum == refractionIllum && isRefractedEnabled) {
            float ior = material.Ni();
            fresnel(direction, normal, ior, &Kr);
            if (Kr < 1) {
                Vec3Df refractionRayOrig = outside ? hitPoint - bias : hitPoint + bias;
                Vec3Df refract = refraction(refractionRayOrig, direction, normal, depth, ior);
                refractionModifier += refract;
                refractionModifier *= material.Ka();
            }
        }
        if (isReflectiveEnabled && Kr > 0.0001) {
            // Do recursive reflection.
            Vec3Df reflectionRayOrig = outside ? hitPoint + bias : hitPoint - bias;
            Vec3Df reflect = reflection(reflectionRayOrig, direction, normal, depth);
            reflectionModifier += reflect;
            reflectionModifier *= material.Ka();
        }
    }

    return (reflectionModifier*Kr + refractionModifier*(1.0-Kr)) + specularModifier + (ambientLight + diffuseModifier) * color;
}


Vec3Df refraction(const Vec3Df &hitPoint, const Vec3Df &direction, const Vec3Df &normal, const int &depth, const float &ior) {
    float cosI = Vec3Df::dotProduct(direction, normal);
    float n1=1.0;
    float n2=ior;
    Vec3Df newNormal = normal;
    if (cosI > 0.0) {
        n1 = ior;
        n2 = 1.0;
        newNormal = -newNormal;
    } else {
        cosI = -cosI;
    }
    float n = n1 / n2;
    if(n == 1) // No change in direction according to Snell's law
        return performRayTracing(hitPoint, hitPoint+direction, depth + 1);


    float cosT = 1.0f - n*n * (1.0f - cosI*cosI);
    if(cosT <= 0.0f) { //Total internal reflection, no need to calculate further.
        return Vec3Df(0,0,0);
    }
    cosT = sqrt(cosT);

    Vec3Df refractionDirection = n * direction + (n * cosI - cosT) * normal;
    return performRayTracing(hitPoint, hitPoint+refractionDirection, depth + 1);
}


// The Fresnel equation: ration of reflected light.
void fresnel(const Vec3Df &direction, const Vec3Df &normal, const float &ior, float *kr) {
	float cosI = -Vec3Df::dotProduct(direction, normal);
	float n1 = 1.0;
	float n2 = ior;
	if (cosI > 0.0) {
	    n1 = ior;
	    n2 = 1.0;
	}

	float n = n1 / n2;
	if(n == 1) {
        (*kr) = 0;
        return;
    }

    float sinT = n * sqrt(std::fmax(0.0, 1-cosI*cosI));

    // TOTAL INTERNAL REFLECTION
	if(sinT > 1.0f) {
        (*kr) = 1;
    } else {
	    float cosT = sqrt(std::fmax(0.0f, 1 - sinT*sinT));
	    cosI = fabs(cosI);
        // Compute the parallel and perpendicular polarised light, we need to compute the ratio of reflected light for
        // These two waves using two different equations (one for each type of wave) and average the results to find the solution.
        float Rs = ((n2 * cosI) - (n1 * cosT)) / ((n2 * cosI) + (n1 * cosT));
        float Rp = ((n1 * cosI) - (n2 * cosT)) / ((n1 * cosI) + (n2 * cosT));
        (*kr) = (Rs * Rs + Rp * Rp) / 2.0f;
    }
}

Vec3Df reflection(const Vec3Df &hitPoint, const Vec3Df &direction, const Vec3Df &normal, const int &depth) {
    // Calculate outgoing vector on vertex from camera's point of view
    Vec3Df outGoingReflection = direction - 2*(Vec3Df::dotProduct(direction, normal)*normal);
    outGoingReflection.normalize();
    Vec3Df outGoingDest = hitPoint + outGoingReflection;
    // RayTrace in that direction
    return performRayTracing(hitPoint, outGoingDest, depth + 1);
}

float blinnPhongSpecular(const std::vector<Vertex> &vertices, std::vector<Vec3Df> *directlyVisibleLights, const Vec3Df &hitPoint, const Vec3Df &normal, Material &material){

    float specularity = 0;
    //Do it for every light source
    #pragma omp parallel for
    for (int i = 0; i < directlyVisibleLights->size(); i++) {
            Vec3Df lightPosition = directlyVisibleLights->at(i);
            Vec3Df lightDirection = lightPosition - hitPoint;
            lightDirection.normalize();


            Vec3Df viewDirection = MyCameraPosition - hitPoint;
            viewDirection.normalize();

            //Find the half vector.
            Vec3Df H = (lightDirection + viewDirection) / (lightDirection + viewDirection).getLength();
            H.normalize();

           //Follow the formula
            float innerProduct = std::fmax(0.0f, Vec3Df::dotProduct(normal, H));
        //Make sure it has shininess first.
        if (material.has_Ns()) {
            specularity += pow(innerProduct, material.Ns());
        }
    }
    if(specularity <= 0 || directlyVisibleLights->empty())
        return 0;

    return specularity / directlyVisibleLights->size();
}

float diffusion(const std::vector<Vertex> &vertices, std::vector<Vec3Df> *directlyVisibleLights, const Vec3Df &hitPoint,
                const Vec3Df &normal) {
    float diffusivity = 0;

    // Loop through all lights.
    #pragma omp parallel for
    for (int i = 0; i < directlyVisibleLights->size(); i++) {
        Vec3Df lightPos = directlyVisibleLights->at(i);
        Vec3Df lightDir = lightPos - hitPoint; // The directions of the light, (from hitPoint to light source)
        lightDir.normalize();


        float dotProduct = Vec3Df::dotProduct(normal, lightDir); // The angle between the lightDirection and the normal of the hitPoint.
        diffusivity += std::fmax(dotProduct, 0.1); //No Negative factors allowed.
    }

    if(diffusivity <= 0 || directlyVisibleLights->empty())
        return 0;


    return diffusivity / directlyVisibleLights->size();
}

/**
 * Add hard shadows as lighting to the scene per visible point
 * @param triangle Triangle in which the point we need to lit is.
 * @param vertices Vertices of triangle metioned above.
 * @param t float multiplier of direction to get point (from origin).
 * @param origin camerapoint.
 * @param dir direction of ray for camerapoint.
 * @param materialIndex index of triangle to get corresponding color.
 * @return color Vector.
 */
float hardShadows(const std::vector<Vertex> &vertices, std::vector<Vec3Df> *directlyVisibleLights, const Vec3Df &hitPoint) {
    float inShadow = 0;
    float inLight = 1;

    Triangle shadowTriangle;
    float totalShadow = 0;

    //loop through different lights

    #pragma omp parallel for
    for (int i = 0; i < virtualLightPositions.size(); i++) {
        Vec3Df lightPos = virtualLightPositions.at(i);
        Vec3Df shadowDir = lightPos - hitPoint;   //direction shadowRay
        float distanceToLight = shadowDir.getLength(); // length of ray between light and point.
        shadowDir.normalize();  //normalize direction
        bool shadowed = false;
        //loop through triangles
        for (int j = 0; j < MyMesh.triangles.size(); j++) {
            shadowTriangle = MyMesh.triangles[j];
            float distanceToIntersection = 0;
            bool shadowCheck = rayIntersect(hitPoint, shadowDir, vertices, shadowTriangle, &distanceToIntersection);
            //check distanceToIntersection with x & last_x
            if (shadowCheck && distanceToIntersection < distanceToLight && !shadowed) {
                //if we hit, dont add anything to the resulting vector
                totalShadow += inShadow;
                shadowed = true;
            } else if (j == MyMesh.triangles.size() - 1 && !shadowed) {
                (*directlyVisibleLights).push_back(lightPos);
                totalShadow += inLight;
            }
        }
    }

    //return result of all vectors
    return totalShadow / virtualLightPositions.size();
}


void yourDebugDraw() {
    //draw open gl debug stuff
    //this function is called every frame

    //let's draw the mesh
    MyMesh.draw();

    //let's draw the lights in the scene as points
    glPushAttrib(GL_ALL_ATTRIB_BITS); //store all GL attributes
    glDisable(GL_LIGHTING);
    glColor3f(1, 1, 1);
    glPointSize(10);
    glBegin(GL_POINTS);
    for (int i = 0; i < MyLightPositions.size(); ++i)
        glVertex3fv(MyLightPositions[i].pointer());
    glEnd();
    glPopAttrib();//restore all GL attributes
    //The Attrib commands maintain the state.
    //e.g., even though inside the two calls, we set
    //the color to white, it will be reset to the previous
    //state after the pop.
    tree.drawOutline();

    //as an example: we draw the test ray, which is set by the keyboard function
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glLineWidth(3);
    glColor3f(1, 1, 1);

    int index;
    Vec3Df dir = testRayDestination - testRayOrigin;
    Material material;

    if(testRayDestination.getLength() != 0 && testRayOrigin.getLength() != 0) {
        Vec3Df direction = testRayDestination - testRayOrigin;
        direction.normalize();

        auto &vertices = MyMesh.vertices;
        Vec3Df origin = testRayOrigin;
        Vec3Df destination = testRayDestination;
        for(int depth = 0; depth < reflectionDepth; depth++) {
            glBegin(GL_LINES);
            glVertex3f(origin[0], origin[1], origin[2]);

            float last_t = std::numeric_limits<float>::max()/100;
            Triangle hitTriangle;
            Vec3Df normal;
            float t;

            int sphereSize = spheres.size();
            int triangleSize = MyMesh.triangles.size();

            for (int i = -sphereSize; i < triangleSize; i++) {
                t = 0;

                if(i >= 0) {
                    //triangle points
                    Triangle triangle = MyMesh.triangles[i];
                    bool intersect = rayIntersect(origin, direction, vertices, triangle, &t);
                    if (intersect && t < last_t) {
                        last_t = t;
                        hitTriangle = triangle;
                        index = i;
                        normal = calcNormal(vertices, hitTriangle);
                        int materialIndex = MyMesh.triangleMaterials.at(i);
                        material = MyMesh.materials.at(materialIndex);
                    }
                } else {
                    if (i < 0) {
                        int ii = (i + sphereSize);
                        Sphere sphere = spheres.at(ii);
                        bool sphereIntersect = sphere.intersect(origin, direction, &t, &normal);
                        if (sphereIntersect && t < last_t) {
                            last_t = t;
                            material = sphere.getMaterial();
                        }
                    }
                }
            }

            if(material.illum() >= reflectionIllum) {
                glColor3f(material.Ka()[0], material.Ka()[1], material.Ka()[2]);
            } else {
                glColor3f(material.Kd()[0], material.Kd()[1], material.Kd()[2]);
            }

            destination = origin + direction * last_t;
            bool outside = Vec3Df::dotProduct(direction, normal) < 0;
            Vec3Df bias = 0.0001f * normal;

            glVertex3f(destination[0], destination[1], destination[2]);
            glEnd();

            Vec3Df reflectionRayOrig = outside ? destination + bias : destination - bias;
            Vec3Df refractionRayOrig = outside ? destination - bias : destination + bias;
            if(material.illum() == reflectionIllum) {
                Vec3Df outGoingReflection = direction - 2*(Vec3Df::dotProduct(direction, normal)*normal);
                outGoingReflection.normalize();
                direction = outGoingReflection;
                origin = reflectionRayOrig;
            } else if(material.illum() == refractionIllum) {
                float Kr = 1;
                float ior = material.Ni();
                fresnel(direction, normal, ior, &Kr);
                if(Kr >= 0.5) {
                    Vec3Df outGoingReflection = direction - 2*(Vec3Df::dotProduct(direction, normal)*normal);
                    outGoingReflection.normalize();
                    direction = outGoingReflection;
                    origin = reflectionRayOrig;
                } else {
                    float cosI = Vec3Df::dotProduct(direction, normal);
                    float n1=ior;
                    float n2=1.0;
                    Vec3Df newNormal = normal;
                    if (cosI > 0.0) {
                        n1 = 1.0;
                        n2 = ior;
                        newNormal = -newNormal;
                    } else {
                        cosI = -cosI;
                    }
                    float n = n1 / n2;
                    if(n == 1)  {// No change in direction according to Snell's law
                        direction = direction;
                        origin = destination;
                    } else {
                        float cosT = 1.0f - n*n * (1.0f - cosI*cosI);
                        if(cosT <= 0.0f) { //Total internal reflection, no need to calculate further.
                            Vec3Df outGoingReflection = direction - 2*(Vec3Df::dotProduct(direction, normal)*normal);
                            outGoingReflection.normalize();
                            direction = outGoingReflection;
                            origin = reflectionRayOrig;
                        } else {
                            cosT = sqrt(cosT);
                            Vec3Df outGoingRefraction = n * direction + (n * cosI - cosT) * normal;
                            outGoingRefraction.normalize();
                            direction = outGoingRefraction;
                            origin = refractionRayOrig;
                        }
                    }
                }
            } else {
                break;
            }
        }
    }

    glLineWidth(1);
    for(auto sphere : spheres)
        sphere.draw();

    if (index != previousIndex) {
        std::cout << "The triangle material properties of the Triangle at the end of the line are: " << std::endl;
        std::cout << "The Kd is : ";
        std::cout << material.Kd() << std::endl;
        std::cout << "The Ks is : ";
        std::cout << material.Ks() << std::endl;
        std::cout << "The illum is : ";
        std::cout << material.illum() << std::endl;
        previousIndex = index;
    }

    glPointSize(10);
    glBegin(GL_POINTS);
    glVertex3fv(MyLightPositions[0].pointer());
    glEnd();
    glPopAttrib();

}




//yourKeyboardFunc is used to deal with keyboard input.
//t is the character that was pressed
//x,y is the mouse position in pixels
//rayOrigin, rayDestination is the ray that is going in the view direction UNDERNEATH your mouse position.
//
//A few keys are already reserved:
//'L' adds a light positioned at the camera location to the MyLightPositions vector
//'l' modifies the last added light to the current
//    camera position (by default, there is only one light, so move it with l)
//    ATTENTION These lights do NOT affect the real-time rendering.
//    You should use them for the raytracing.
//'r' calls the function performRaytracing on EVERY pixel, using the correct associated ray.
//    It then stores the result in an image "result.bmp".
//    Initially, this function is fast (performRaytracing simply returns
//    the target of the ray - see the code above), but once you replaced
//    this function and raytracing is in place, it might take a
//    while to complete...
void yourKeyboardFunc(char t, int x, int y, const Vec3Df &rayOrigin, const Vec3Df &rayDestination) {

    //here, as an example, I use the ray to fill in the values for my upper global ray variable
    //I use these variables in the debugDraw function to draw the corresponding ray.
    //try it: Press a key, move the camera, see the ray that was launched as a line.
    testRayOrigin = rayOrigin;
    testRayDestination = rayDestination;
    Vec3Df color;
    color = performRayTracing(rayOrigin, rayDestination, 0);

    // do here, whatever you want with the keyboard input t.

    std::cout << t << " pressed! The mouse was in location " << x << "," << y << "  and the pixel color is "<< color << "!" << std::endl;
}