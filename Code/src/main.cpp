#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#endif
#ifdef WIN32
#include <windows.h>
#endif
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <ctime>
#include <Box.h>
#include <cstdlib>
#include "raytracing.h"
#include "mesh.h"
#include "mouse.h"
#include "imageWriter.h"
#include "Box.h"



//This is the main application
//Most of the code in here, does not need to be modified.
//It is enough to take a look at the function "drawFrame",
//in case you want to provide your own different drawing functions



Vec3Df MyCameraPosition;

//MyLightPositions stores all the light positions to use
//for the ray tracing. Please notice, the light that is 
//used for the real-time rendering is NOT one of these, 
//but following the camera instead.
std::vector<Vec3Df> MyLightPositions;
std::vector<Vec3Df> virtualLightPositions;
const float sphereSize = 0.2;
const float cubeSize = 0.25;
const double pi = 3.141592;
const int amountOfRays = 25;
bool isSphere = false;
bool isBox = false;
float lightIntensity = 1.5;
Vec3Df lightColor = Vec3Df(1,1,1);
Vec3Df localLightColor = Vec3Df(1,1,1);
bool isSpecularEnabled = true;
bool isReflectiveEnabled = true;
bool isRefractedEnabled = true;
//Main mesh 
Mesh MyMesh;

unsigned int WindowSize_X = 1000;  // resolution X
unsigned int WindowSize_Y = 1000;  // resolution Y
unsigned long AmountOfIntersect;




/**
 * Main function, which is drawing an image (frame) on the screen
*/
void drawFrame( )
{
	yourDebugDraw();
}

//animation is called for every image on the screen once
void animate()
{
	MyCameraPosition=getCameraPosition();
	glutPostRedisplay();
}



void display(void);
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);

/**
 * Main program
 */
int main(int argc, char** argv)
{
    glutInit(&argc, argv);

    //framebuffer setup
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );

    // positioning and size of window
    glutInitWindowPosition(200, 100);
    glutInitWindowSize(WindowSize_X,WindowSize_Y);
    glutCreateWindow(argv[0]);	

    //initialize viewpoint
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, -0.2,-4);
    tbInitTransform();     // This is for the mouse, please ignore
    tbHelp();             // idem
	MyCameraPosition=getCameraPosition();

	//activate the light following the camera
    glEnable( GL_LIGHTING );
    glEnable( GL_LIGHT0 );
    glEnable(GL_COLOR_MATERIAL);
    int LightPos[4] = {1,1,2,0};
    int MatSpec [4] = {1,1,1,1};
    glLightiv(GL_LIGHT0,GL_POSITION,LightPos);

	//normals will be normalized in the graphics pipeline
	glEnable(GL_NORMALIZE);
    //clear color of the background is black.
	glClearColor (0.0, 0.0, 0.0, 0.0);

	
	// Activate rendering modes
    //activate depth test
	glEnable( GL_DEPTH_TEST ); 
    //draw front-facing triangles filled
	//and back-facing triangles as wires
    glPolygonMode(GL_FRONT,GL_FILL);
    glPolygonMode(GL_BACK,GL_LINE);
    //interpolate vertex colors over the triangles
	glShadeModel(GL_SMOOTH);


	// glut setup... to ignore
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);    // keyboard callback
    glutDisplayFunc(display);      // draw callback
    glutMouseFunc(tbMouseFunc);    // mouse callback
    glutMotionFunc(tbMotionFunc);  // mouse callback
    glutIdleFunc( animate);        // animation callback


	init();

    
	//main loop for glut... this just runs your application
    glutMainLoop();
        
    return 0;  // execution never reaches this point
}











/**
 * OpenGL setup - functions do not need to be changed! 
 * you can SKIP AHEAD TO THE KEYBOARD FUNCTION
 */
//what to do before drawing an image
 void display(void)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);//store GL state
    // Clear everything
    glClear( GL_COLOR_BUFFER_BIT  | GL_DEPTH_BUFFER_BIT); // clear image
    
    glLoadIdentity();  

    tbVisuTransform(); // init mouse

    drawFrame( );    //actually draw

    glutSwapBuffers();//glut internal switch
	glPopAttrib();//return to old GL state
}
//Window changes size
void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //glOrtho (-1.1, 1.1, -1.1,1.1, -1000.0, 1000.0);
    //glFrustum(-1.0, 1.0, -1.0, 1.0, 1.5, 20.0);
    gluPerspective (50, (float)w/h, 0.001, 30);
    glMatrixMode(GL_MODELVIEW);
}


//transform the x, y position on the screen into the corresponding 3D world position
void produceRay(int x_I, int y_I, Vec3Df * origin, Vec3Df * dest)
{
		int viewport[4];
		double modelview[16];
		double projection[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, modelview); // get current matrices
		glGetDoublev(GL_PROJECTION_MATRIX, projection); // get current matrices
		glGetIntegerv(GL_VIEWPORT, viewport);//viewport
		int y_new = viewport[3] - y_I;

		double x, y, z;
		
		gluUnProject(x_I, y_new, 0, modelview, projection, viewport, &x, &y, &z);
		origin->p[0]=float(x);
		origin->p[1]=float(y);
		origin->p[2]=float(z);
		gluUnProject(x_I, y_new, 1, modelview, projection, viewport, &x, &y, &z);
		dest->p[0]=float(x);
		dest->p[1]=float(y);
		dest->p[2]=float(z);
}


void calcBoxPoint (Vec3Df & lightPos) {
    Vec3Df lightPosMax = lightPos + cubeSize * Vec3Df(1, 1, 1);
    Vec3Df lightPosMin = lightPos - cubeSize * Vec3Df(1, 1, 1);
    Box lightBox;
    lightBox = Box(lightPosMin, lightPosMax);

    int squareFinder = rand() % 5;
    //generates a random number between 0 and 1.
    float randomY_Square = ((float) rand()) / (float) RAND_MAX;
    float randomX_Square = ((float) rand()) / (float) RAND_MAX;

    //East Square of Box
    if (squareFinder == 0) {
        Vec3Df getBase = lightBox.getPointA();
        Vec3Df directionX = lightBox.getPointD() - lightBox.getPointA();
        Vec3Df directionY = lightBox.getPointE() - lightBox.getPointA();
        Vec3Df virtualLightPoint = getBase + directionX * randomX_Square + directionY * randomY_Square;
        virtualLightPositions.push_back(virtualLightPoint);

    }

    //North Square of Box
    if (squareFinder == 1) {
        Vec3Df getBase = lightBox.getPointB();
        Vec3Df directionX = lightBox.getPointA() - lightBox.getPointB();
        Vec3Df directionY = lightBox.getPointF() - lightBox.getPointB();
        Vec3Df virtualLightPoint = getBase + directionX * randomX_Square + directionY * randomY_Square;
        virtualLightPositions.push_back(virtualLightPoint);

    }

    //West Square of Box
    if (squareFinder == 2) {
        Vec3Df getBase = lightBox.getPointC();
        Vec3Df directionX = lightBox.getPointB() - lightBox.getPointC();
        Vec3Df directionY = lightBox.getPointH() - lightBox.getPointC();
        Vec3Df virtualLightPoint = getBase + directionX * randomX_Square + directionY * randomY_Square;
        virtualLightPositions.push_back(virtualLightPoint);

    }

    //South Square of Box
    if (squareFinder == 3) {
        Vec3Df getBase = lightBox.getPointD();
        Vec3Df directionX = lightBox.getPointC() - lightBox.getPointD();
        Vec3Df directionY = lightBox.getPointE() - lightBox.getPointD();
        Vec3Df virtualLightPoint = getBase + directionX * randomX_Square + directionY * randomY_Square;
        virtualLightPositions.push_back(virtualLightPoint);

    }

    //Upper Square of Box
    if (squareFinder == 4) {
        Vec3Df getBase = lightBox.getPointG();
        Vec3Df directionX = lightBox.getPointH() - lightBox.getPointG();
        Vec3Df directionY = lightBox.getPointE() - lightBox.getPointG();
        Vec3Df virtualLightPoint = getBase + directionX * randomX_Square + directionY * randomY_Square;
        virtualLightPositions.push_back(virtualLightPoint);

    }

    //Bottom Square of Box
    if (squareFinder == 5) {
        Vec3Df getBase = lightBox.getPointD();
        Vec3Df directionX = lightBox.getPointC() - lightBox.getPointD();
        Vec3Df directionY = lightBox.getPointA() - lightBox.getPointD();
        Vec3Df virtualLightPoint = getBase + directionX * randomX_Square + directionY * randomY_Square;
        virtualLightPositions.push_back(virtualLightPoint);

    }
 }

void calcSpherePoint (Vec3Df & lightPos) {
    //generates a random number between 0 and 1.
    float randomNumber = ((float) rand()) / (float) RAND_MAX;
    float polarAngle = randomNumber * pi;
    float equatorAngle = randomNumber * 2 * pi;

    //get x, y and z positions for a point on the sphere.
    float xpoint = sphereSize * sin(polarAngle) * cos(equatorAngle);
    float ypoint = sphereSize * sin(polarAngle) * sin(equatorAngle);
    float zpoint = sphereSize * cos(polarAngle);
    Vec3Df virtualPoint = Vec3Df(xpoint, ypoint, zpoint) + lightPos;
    virtualLightPositions.push_back(virtualPoint);
 }





// react to keyboard input
void keyboard(unsigned char key, int x, int y) {
    printf("key %d pressed at %d,%d\n", key, x, y);
    fflush(stdout);
    switch (key) {
        //add/update a light based on the camera position.
        case 'L':
            MyLightPositions.push_back(getCameraPosition());
            break;
        case 'l':
            MyLightPositions[MyLightPositions.size() - 1] = getCameraPosition();
            break;
            //create a box around every light position and add them to a vector of virtual light to preserve the real lights.
        case 'v':
            //clear out previous virtual lights
            if (virtualLightPositions.size() > 0) {
                virtualLightPositions.clear();
            }
            cout << "Beginning pushing virtualLights" << endl;
            for (int i = 0; i < MyLightPositions.size(); i++) {
                Vec3Df lightPos = MyLightPositions.at(i);
                for (int i = 0; i < amountOfRays; i++) {
                   calcBoxPoint(lightPos);
                }
            }
            if (virtualLightPositions.size() > 0) {
                std::cout << "Amount of virtual light Positions is now: ";
                cout << virtualLightPositions.size() << endl;
            }
            isBox = true;
            isSphere = false;
            break;

            //create a sphere around every light position and add them to a vector of virtual lights to preserve real lights.
        case 'c':
            //clear out previous virtual Lights
            if (virtualLightPositions.size() > 0) {
                virtualLightPositions.clear();
            }

            //Loop through ligth positions
            for (int i = 0; i < MyLightPositions.size(); i++) {
                Vec3Df lightPos = MyLightPositions.at(i);
                for (int s = 0; s < amountOfRays; s++) {
                   calcSpherePoint(lightPos);
                }
            }
            std::cout << "Amount of virtual light Positions is now: ";
            std::cout << virtualLightPositions.size() << endl;
            isBox = false;
            isSphere = true;
            break;

            //changes the lights to hard light points
        case 'b':
            if (virtualLightPositions.size() > 0) {
                virtualLightPositions.clear();
                std::cout << "Amount of virtual light Positions is now: ";
                std::cout << virtualLightPositions.size() << endl;
            }
            isBox = false;
            isSphere = false;
            break;

            //toggles Refraction
        case 'a':
            isRefractedEnabled = !isRefractedEnabled;
            cout<<"Refraction is now: ";
            if (isRefractedEnabled) {
                cout<<"on"<<endl;
            } else{
                cout<<"off"<<endl;
            }
            break;

            //toggles specularity
        case 's':
            isSpecularEnabled = !isSpecularEnabled;
            cout<<"Specularity is now: ";
            if (isSpecularEnabled) {
                cout<<"on"<<endl;
            } else {
                cout<<"off"<<endl;
            }
            break;

            //toggles reflection
        case 'd':
            isReflectiveEnabled = !isReflectiveEnabled;
            cout<<"Reflection is now: ";
            if (isReflectiveEnabled) {
                cout<<"on"<<endl;
            } else{
                cout<<"off"<<endl;

            }
            break;

            //remove one virtual light from each original light source
        case 'z': {
            if (virtualLightPositions.size() > 2*MyLightPositions.size()) {
                if (MyLightPositions.size() == 1) {
                    virtualLightPositions.erase(virtualLightPositions.begin()+virtualLightPositions.size()-1);
                }
                else {
                    int biggerThan = virtualLightPositions.size()/MyLightPositions.size();
                    biggerThan -= MyLightPositions.size();
                    cout<<biggerThan<<endl;
                    for (int i = MyLightPositions.size(); i< virtualLightPositions.size(); i++) {
                        if (i % biggerThan == 0) {
                        virtualLightPositions.erase(virtualLightPositions.begin() + i);
                        }
                    }
                }
            }
            std::cout << "Amount of virtual light Positions is now: ";
            std::cout << virtualLightPositions.size() << endl;
        }
            break;
            //add one virtual light from each original light source
        case 'x':
            for (int i = 0; i < MyLightPositions.size(); i++) {
                if (isSphere && !isBox) {
                    calcSpherePoint(MyLightPositions.at(i));
                }
                if (!isSphere && isBox) {
                    calcBoxPoint(MyLightPositions.at(i));
                } else {
                    cout<<"Something bad went wrong"<<endl;
                }
                //Rewrite the C and V case to not have the actual methods in it
            }
            std::cout << "Amount of virtual light Positions is now: ";
            std::cout << virtualLightPositions.size() << endl;
            break;

            //decrease the intensity of the light by 0.1
        case '1':
            if (lightIntensity >= 0.1) {
                lightIntensity -= 0.1;
            }
            cout<<"Light intensity is now: ";
            cout<<lightIntensity<<endl;
            break;

            //increase the intensity of the light by 0.1
        case '2':
            lightIntensity += 0.1;
            cout<<"Light intensity is now: ";
            cout<<lightIntensity<<endl;
            break;

            // Ups the red light color by 0.1, and loops to 0 after it crosses 1.
        case '8':
            if (localLightColor[0] >= 1) {
                localLightColor[0] = 0.0;
            } else {
                localLightColor[0] +=0.1;
            }
            lightColor = localLightColor;
            cout<<"Light Color is now: ";
            cout<<lightColor<<endl;

            break;

            // Ups the green light color by 0.1, and loops to 0 after it crosses 1.
        case '9':
            if (localLightColor[1] >= 1) {
                localLightColor[1] = 0.0;
            } else {
                localLightColor[1] +=0.1;
            }
            lightColor = localLightColor;
            cout<<"Light Color is now: ";
            cout<<lightColor<<endl;

            break;

            // Ups the blue light color by 0.1, and loops to 0 after it crosses 1.
        case '0':
            if (localLightColor[2] >= 1) {
                localLightColor[2] = 0.0;
            } else {
                localLightColor[2] +=0.1;
            }
            lightColor = localLightColor;
            cout<<"Light Color is now: ";
            cout<<lightColor<<endl;

            break;

        case 'r': {
            //Pressing r will launch the raytracing.
            cout << "Raytracing" << endl;
            if (virtualLightPositions.size() < MyLightPositions.size()) {
                virtualLightPositions = MyLightPositions;
            }


            //Setup an image with the size of the current image.
            Image result(WindowSize_X, WindowSize_Y);

            //produce the rays for each pixel, by first computing
            //the rays for the corners of the frustum.
            Vec3Df origin00, dest00;
            Vec3Df origin01, dest01;
            Vec3Df origin10, dest10;
            Vec3Df origin11, dest11;
            Vec3Df origin, dest;


            produceRay(0, 0, &origin00, &dest00);
            produceRay(0, WindowSize_Y - 1, &origin01, &dest01);
            produceRay(WindowSize_X - 1, 0, &origin10, &dest10);
            produceRay(WindowSize_X - 1, WindowSize_Y - 1, &origin11, &dest11);


            //statistics
            clock_t timeStart = clock();
            int amountOfRaytracing = 0;

            #pragma omp parallel for
            for (unsigned int y = 0; y < WindowSize_Y; ++y) {
                //print current progress
                std::cout << (float) y * 100 / WindowSize_Y << "% \n";
                #pragma omp parallel for
                for (unsigned int x = 0; x < WindowSize_X; ++x) {
                    //produce the rays for each pixel, by interpolating
                    //the four rays of the frustum corners.
                    float xscale = 1.0f - float(x) / (WindowSize_X - 1);
                    float yscale = 1.0f - float(y) / (WindowSize_Y - 1);

                    origin = yscale * (xscale * origin00 + (1 - xscale) * origin10) +
                             (1 - yscale) * (xscale * origin01 + (1 - xscale) * origin11);
                    dest = yscale * (xscale * dest00 + (1 - xscale) * dest10) +
                           (1 - yscale) * (xscale * dest01 + (1 - xscale) * dest11);

                    //launch raytracing for the given ray.
                    Vec3Df rgb = performRayTracing(origin, dest, 0);
                    amountOfRaytracing++;
                    //store the result in an image
                    result.setPixel(x, y, RGBValue(rgb[0], rgb[1], rgb[2]));
                }
            }
            result.writeImage("result.bmp");


		clock_t timeStop = clock();
		std::cout<<std::endl<<"Time elapsed (estimated): " << (float) (timeStop-timeStart) /CLOCKS_PER_SEC << "(sec)" << std::endl;
        std::cout<<"Time number of triangles: " << MyMesh.triangles.size() << std::endl;
        std::cout<<"Amount of rays: " << amountOfRaytracing << std::endl;
        std::cout<<"Amount of intersects: "<< AmountOfIntersect << std::endl;
        std::cout<<"Amount of intersects per ray(on average): "<<(float) AmountOfIntersect/amountOfRaytracing<<std::endl;
            break;

        }
        case 27:     // ESC pressed
            exit(0);
    }
            //produce the ray for the current mouse position
            Vec3Df testRayOrigin, testRayDestination;
            produceRay(x, y, &testRayOrigin, &testRayDestination);

            yourKeyboardFunc(key, x, y, testRayOrigin, testRayDestination);
}

