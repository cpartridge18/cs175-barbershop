//
//  main.cpp
//  cs175-barbershop
//
//  Created by Chris Partridge on 4/28/23.
//

// TODO:
// 1. Add clippers object
// 2. Remove extraneous code (animation? key controllers? robots?)
// 3. Make head
// 4. Add fur
// 5. profit $$$$++



////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <cstddef>
#include <cstring>
#include <fstream>
#include <list>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <math.h>

#define GLEW_STATIC
#include "GL/glew.h"
#include "GL/glfw3.h"

#include "arcball.h"
#include "cvec.h"
#include "geometry.h"
#include "geometrymaker.h"
#include "glsupport.h"
#include "matrix4.h"
#include "ppm.h"
#include "rigtform.h"
#include "scenegraph.h"
#include "sgutils.h"

#include "asstcommon.h"
#include "drawer.h"
#include "picker.h"
#include "mesh.h"


//#include <filesystem>
//namespace fs = std::filesystem;

using namespace std;

// G L O B A L S ///////////////////////////////////////////////////


static const float g_frustMinFov = 60.0; // A minimal of 60 degree field of view
static float g_frustFovY =
    g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;  // near plane
static const float g_frustFar = -50.0;  // far plane
static const float g_groundY = -2.0;    // y coordinate of the ground
static const float g_groundSize = 10.0; // half the ground length

enum SkyMode { WORLD_SKY = 0, SKY_SKY = 1 };

static GLFWwindow *g_window;

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static double g_wScale = 1;
static double g_hScale = 1;

static bool g_mouseClickDown = false; // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static bool g_spaceDown = false; // space state, for middle mouse emulation
static double g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

static SkyMode g_activeCameraFrame = WORLD_SKY;

static bool g_displayArcball = true;
static double g_arcballScreenRadius = 100; // number of pixels
static double g_arcballScale = 1;

static bool g_pickingMode = false;

static bool g_playingAnimation = false;

static bool g_won = false;

// Material
static shared_ptr<Material> g_bunnyMat; // for the bunny

static vector<shared_ptr<Material>> g_bunnyShellMats; // for bunny shells

// New Geometry
static const int g_numShells = 24; // constants defining how many layers of shells
static double g_furHeight = 0.4;
static double g_hairyness = 0.7;
static bool g_shellNeedsUpdate = false;

static shared_ptr<SimpleGeometryPN> g_bunnyGeometry, g_headGeometry, g_headNoHairGeometry, g_trimmerGeometry, g_chairGeometry;
static vector<shared_ptr<SimpleGeometryPNX>> g_bunnyShellGeometries, g_headShellGeometries;
static Mesh g_bunnyMesh, g_headMesh, g_headNoHairMesh, g_trimmerMesh, g_chairMesh;

// New Scene node
static shared_ptr<SgRbtNode> g_bunnyNode, g_chairNode;

// For Simulation
static double g_lastFrameClock;

// Global variables for used physical simulation
static const Cvec3 g_gravity(0, -0.5, 0); // gavity vector
static double g_timeStep = 0.02;
static double g_numStepsPerFrame = 10;
static double g_damping = 0.96;
static double g_stiffness = 4;
static int g_simulationsPerSecond = 60;

bool g_preRender = true;
float depthSpd = 0.05; 

static std::vector<Cvec3>
    g_ogTipPos,
    g_tipPos,      // should be hair tip pos in world-space coordinates
    g_tipVelocity, g_headTipPos, g_headTipVelocity; // should be hair tip velocity in world-space coordinates

vector<double> g_hairLengths;

// --------- Materials
static shared_ptr<Material> g_redDiffuseMat, g_blueDiffuseMat, g_bumpFloorMat,
    g_arcballMat, g_pickingMat, g_lightMat, g_shaverMat, g_mirrorMat, g_chairMat;

shared_ptr<Material> g_overridingMaterial;

// --------- Geometry
typedef SgGeometryShapeNode MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere, g_mirror;

// --------- Scene

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node,
    g_robot2Node, g_light1, g_light2, g_shaverNode, g_headNode, g_mirrorNode, g_cutNode;

static shared_ptr<SgRbtNode> g_currentCameraNode;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode;

static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_framesPerSecond = 60; // frames to render per second during animation playback
//
///////////////// END OF G L O B A L S /////////////////////////////////////////

// New function to initialize the dynamics simulation
static void initSimulation() {
    RigTForm obj = getPathAccumRbt(g_world, g_headNode);
    
    for (int i = 0; i < g_headMesh.getNumFaces(); i++) {
        g_hairLengths.push_back(g_furHeight);
        for (int j = 0; j < g_headMesh.getFace(i).getNumVertices(); j++) {
            Mesh::Vertex v = g_headMesh.getFace(i).getVertex(j);
            
            
            Cvec3 p = v.getPosition();
            Cvec3 norm = v.getNormal();
//            Cvec3 t = p + (norm*g_furHeight);
            Cvec3 t = p + (norm*g_hairLengths[i]);
//            Cvec3 t = p + (norm * g_headMesh.getFace(i).getHairHeight());
            
            g_headTipPos.push_back((obj * RigTForm(t)).getTranslation());
            g_headTipVelocity.push_back(Cvec3(0, 0, 0));
        }
    }
    
}

static void initGround() {
    int ibLen, vbLen;
    getPlaneVbIbLen(vbLen, ibLen);

    // Temporary storage for cube Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    makePlane(g_groundSize * 2, vtx.begin(), idx.begin());
    g_ground.reset(
        new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
    
    // MAKE MIRROR
}

static void initCubes() {
    int ibLen, vbLen;
    getCubeVbIbLen(vbLen, ibLen);

    // Temporary storage for cube Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    makeCube(1, vtx.begin(), idx.begin());
    g_cube.reset(
        new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
    int ibLen, vbLen;
    getSphereVbIbLen(20, 10, vbLen, ibLen);

    // Temporary storage for sphere Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);
    makeSphere(1, 20, 10, vtx.begin(), idx.begin());
    g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(),
                                                  idx.size()));
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(Uniforms &uniforms,
                                 const Matrix4 &projMatrix) {
    uniforms.put("uProjMatrix", projMatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
    if (g_windowWidth >= g_windowHeight)
        g_frustFovY = g_frustMinFov;
    else {
        const double RAD_PER_DEG = 0.5 * CS175_PI / 180;
        g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight /
                                g_windowWidth,
                            cos(g_frustMinFov * RAD_PER_DEG)) /
                      RAD_PER_DEG;
    }
}

static Matrix4 makeProjectionMatrix() {
    return Matrix4::makeProjection(
        g_frustFovY, g_windowWidth / static_cast<double>(g_windowHeight),
        g_frustNear, g_frustFar);
}

enum ManipMode { ARCBALL_ON_PICKED, ARCBALL_ON_SKY, EGO_MOTION };

static ManipMode getManipMode() {
    // if nothing is picked or the picked transform is the transfrom we are
    // viewing from
    if (g_currentPickedRbtNode == NULL ||
        g_currentPickedRbtNode == g_currentCameraNode) {
        if (g_currentCameraNode == g_skyNode &&
            g_activeCameraFrame == WORLD_SKY)
            return ARCBALL_ON_SKY;
        else
            return EGO_MOTION;
    } else
        return ARCBALL_ON_PICKED;
}

static bool shouldUseArcball() {
    return (g_currentPickedRbtNode != 0);
    //  return getManipMode() != EGO_MOTION;
}

// The translation part of the aux frame either comes from the current
// active object, or is the identity matrix when
static RigTForm getArcballRbt() {
    switch (getManipMode()) {
    case ARCBALL_ON_PICKED:
        return getPathAccumRbt(g_world, g_currentPickedRbtNode);
    case ARCBALL_ON_SKY:
        return RigTForm();
    case EGO_MOTION:
        return getPathAccumRbt(g_world, g_currentCameraNode);
    default:
        throw runtime_error("Invalid ManipMode");
    }
}

static void updateArcballScale() {
    RigTForm arcballEye =
        inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
    double depth = arcballEye.getTranslation()[2];
    if (depth > -CS175_EPS)
        g_arcballScale = 0.02;
    else
        g_arcballScale =
            getScreenToEyeScale(depth, g_frustFovY, g_windowHeight);
}

static void drawArcBall(Uniforms &uniforms) {
    RigTForm arcballEye =
        inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
    Matrix4 MVM = rigTFormToMatrix(arcballEye) *
                  Matrix4::makeScale(Cvec3(1, 1, 1) * g_arcballScale *
                                     g_arcballScreenRadius);

    sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));

    g_arcballMat->draw(*g_sphere, uniforms);
}

// Specifying shell geometries based on g_tipPos, g_furHeight, and g_numShells.
// You need to call this function whenver the shell needs to be updated
static void updateShellGeometry() {
    // TASK 1 and 3 TODO: finish this function as part of Task 1 and Task 3
        
    RigTForm obj = getPathAccumRbt(g_world, g_headNode);

    for (int k = 0; k < g_numShells; k++) {
        
        vector<VertexPNX> vec2;
        vec2.reserve(g_headMesh.getNumFaces() * 3);
        
        for (int i = 0; i < g_headMesh.getNumFaces(); i++) {
            for (int j = 0; j < g_headMesh.getFace(i).getNumVertices(); j++) {
                Mesh::Vertex v = g_headMesh.getFace(i).getVertex(j);
                Cvec2 texCoords;
                
                
                Cvec3 p = v.getPosition();
                Cvec3 norm = v.getNormal();
                Cvec3 t = g_headTipPos[(i*3)+j];
                Cvec3 s = p + (norm*g_hairLengths[i]);
//                Cvec3 s = p + (norm* g_headMesh.getFace(i).getHairHeight());
          
                // translate t into object coordinates
                Cvec3 newt = (inv(obj) * RigTForm(t)).getTranslation();
                
                Cvec3 n = (s - p) / g_numShells;
                // TODO: Calculate p
//                Cvec3 pi1 = p + (n * (k+1));
                
                Cvec3 d = ((newt - p - (n * g_numShells)) / (g_numShells*(g_numShells-1)))*2;
                
                double sum = ((k - 1)*(k)) / 2;
                Cvec3 p_i = p + (n * k) + (d * sum);
                
                if (j == 0) {
                    texCoords = Cvec2(0,0);
                } else if (j == 1) {
                    texCoords = Cvec2(g_hairyness, 0);
                } else {
                    texCoords = Cvec2(0, g_hairyness);
                }
                double sum_minus = ((k - 2)*(k-1)) / 2;
                if (k == 0 ) {
                    sum_minus = 0;
                }
                if (k == 1) {
                    sum_minus = 1;
                }
                Cvec3 p_i_minus = p + (n * (k-1)) + (d * sum_minus);
                Cvec3 normal = (p_i - p_i_minus);
                VertexPNX vertex = VertexPNX(p_i, normal, texCoords);
                // Make VertexPN
                vec2.push_back(vertex);
            }
        }
        VertexPNX* v2 = &vec2[0];
        g_headShellGeometries[k]->upload(v2, g_headMesh.getNumFaces() * 3);
    }
    
    g_shellNeedsUpdate = false;
    
}


static void drawStuff(bool picking, bool mirror=false) {
    // if we are not translating, update arcball scale
    if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton) ||
          (g_mouseLClickButton && !g_mouseRClickButton && g_spaceDown)))
        updateArcballScale();

    Uniforms uniforms;

    // build & send proj. matrix to vshader
    const Matrix4 projmat = makeProjectionMatrix();
    sendProjectionMatrix(uniforms, projmat);
    

    g_currentCameraNode = g_skyNode;

    const RigTForm eyeRbt = getPathAccumRbt(g_world, g_currentCameraNode);
    const RigTForm invEyeRbt = inv(eyeRbt);

    Cvec3 l1 = getPathAccumRbt(g_world, g_light1).getTranslation();
    Cvec3 l2 = getPathAccumRbt(g_world, g_light2).getTranslation();
    uniforms.put("uLight", Cvec3(invEyeRbt * Cvec4(l1, 1)));
    uniforms.put("uLight2", Cvec3(invEyeRbt * Cvec4(l2, 1)));

    if (!picking) {
        Drawer drawer(invEyeRbt, uniforms);
        g_world->accept(drawer);

        if (g_displayArcball && shouldUseArcball())
            drawArcBall(uniforms);
    } else {
        Picker picker(invEyeRbt, uniforms);

        g_overridingMaterial = g_pickingMat;
        g_world->accept(picker);
        g_overridingMaterial.reset();

        glFlush();
        g_currentPickedRbtNode =
            picker.getRbtNodeAtXY(g_mouseClickX * g_wScale,
                                  g_mouseClickY * g_hScale);
        if (g_currentPickedRbtNode == g_groundNode or g_currentPickedRbtNode == g_mirrorNode)
            g_currentPickedRbtNode.reset(); // set to NULL

        cout << (g_currentPickedRbtNode ? "Part picked" : "No part picked")
             << endl;
    }
    
    if (g_shellNeedsUpdate) {
        updateShellGeometry();
    }
}

// New function to update the simulation every frame
static void hairsSimulationUpdate() {
    // TASK 2 TODO: write dynamics simulation code here
    // bunny coordinates

    RigTForm obj = getPathAccumRbt(g_world, g_headNode);

    
    for (int j = 0; j < g_numStepsPerFrame; j++) {
        for (int i = 0; i < g_headTipPos.size(); i++) {
            // 3i, 3i + 1, 3i + 2
            Mesh::Vertex v = g_headMesh.getFace(floor(i / 3)).getVertex(i % 3);
            Cvec2 texCoords;

            // take p in world coords
            Cvec3 p = v.getPosition();
            Cvec3 nor = v.getNormal();

            Cvec3 s = p + (nor * g_hairLengths[floor(i / 3)]);
            
//            Cvec3 s = p + (nor * g_headMesh.getFace(floor(i / 3)).getHairHeight());
            
            s = (obj * RigTForm(s)).getTranslation();
            p = (obj * RigTForm(p)).getTranslation();

            Cvec3 total_force = g_gravity + ((s  - g_headTipPos[i])*g_stiffness);
    //
            g_headTipPos[i] = g_headTipPos[i] + (g_headTipVelocity[i]*g_timeStep);

            Cvec3 first = ((g_headTipPos[i] - p) / norm(g_headTipPos[i] - p))* g_hairLengths[floor(i / 3)];
            g_headTipPos[i] = p + first;

            g_headTipVelocity[i] = (g_headTipVelocity[i] + (total_force*g_timeStep)) * g_damping;
        }
    }
    
    g_shellNeedsUpdate = true;
        
}

static void checkCutLength() {
    // find the distance b/e center of face and clipper object

    double maxHairLen = 0;

    // shaver in world coords
    RigTForm shavObj = getPathAccumRbt(g_world, g_cutNode);
//    shavObj = RigTForm(shavObj.getTranslation() + Cvec3(0, 0, 0), shavObj.getRotation());
    RigTForm headObj = getPathAccumRbt(g_world, g_headNode);
    
    double sum_hairlens = 0;
    
    for (int i = 0; i < g_headMesh.getNumFaces(); i++) {

        double cntr_x = 0;
        double cntr_y = 0;
        double cntr_z = 0;
        for (int j = 0; j < g_headMesh.getFace(i).getNumVertices(); j++) {

            // convert center of face to world cordinates
            Cvec3 p = (headObj * RigTForm(g_headMesh.getFace(i).getVertex(j).getPosition())).getTranslation();
            cntr_x += p[0];
            cntr_y += p[1];
            cntr_z += p[2];

        }
        Cvec3 centroid = Cvec3((cntr_x / 3), (cntr_y / 3), (cntr_z / 3));
    
        
        float dist = sqrt(pow((centroid[0] - shavObj.getTranslation()[0]), 2) + pow((centroid[1] - shavObj.getTranslation()[1]), 2) + pow((centroid[2] - shavObj.getTranslation()[2]), 2));
        
        if (dist < g_hairLengths[i]) {
            g_hairLengths[i] = dist;

        }
        
        if (dist < .1) {
            RigTForm cur = g_shaverNode->getRbt();
            g_shaverNode->setRbt(RigTForm(cur.getTranslation() + (g_headMesh.getFace(i).getNormal()*.005), cur.getRotation()));
        } 
        
        // after updating hair length, update maximum hair length
        if (maxHairLen < g_hairLengths[i]) {
            maxHairLen = g_hairLengths[i];
        }
        
        sum_hairlens += g_hairLengths[i];
    }
    
    if (((sum_hairlens / double(g_hairLengths.size())) < 0.25) || g_won == true) {
        g_won = true;
        cout << "YOU WIN" << endl;
        cout << "YOU WIN" << endl;
        cout << "YOU WIN" << endl;
    }
    
    if (maxHairLen < 0.4) {
        
    }
    
    
    // prevent clipping on nohair mesh
    for (int i = 0; i < g_headNoHairMesh.getNumFaces(); i++) {
        double cntr_x = 0;
        double cntr_y = 0;
        double cntr_z = 0;
        for (int j = 0; j < g_headNoHairMesh.getFace(i).getNumVertices(); j++) {
            
            // convert center of face to world cordinates
            Cvec3 p = (RigTForm(headObj.getTranslation() + Cvec3(0, -.8, .4), headObj.getRotation()) * RigTForm(g_headNoHairMesh.getFace(i).getVertex(j).getPosition())).getTranslation();
            cntr_x += p[0];
            cntr_y += p[1];
            cntr_z += p[2];
            
        }
        Cvec3 centroid = Cvec3((cntr_x / 3), (cntr_y / 3), (cntr_z / 3));
        
        
        float noHair_dist = sqrt(pow((centroid[0] - shavObj.getTranslation()[0]), 2) + pow((centroid[1] - shavObj.getTranslation()[1]), 2) + pow((centroid[2] - shavObj.getTranslation()[2]), 2));
        
        if (noHair_dist < .1) {

            RigTForm cur = g_shaverNode->getRbt();
            g_shaverNode->setRbt(RigTForm(cur.getTranslation() + (g_headNoHairMesh.getFace(i).getNormal()*.001), cur.getRotation()));
        }
    }
    
    
    
    
    
}

static void preRender() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
    GLuint mirrorFBO = 0;
    glGenFramebuffers(1, &mirrorFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, mirrorFBO);
    
    // The texture we're going to render to
    GLuint renderedTexture;
    glGenTextures(1, &renderedTexture);

    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_2D, renderedTexture);

    // Give an empty image to OpenGL ( the last "0" )
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, g_windowWidth, g_windowHeight, 0,GL_RGB, GL_UNSIGNED_BYTE, 0);

    // Poor filtering. Needed !
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    
    // The depth buffer
    GLuint depthrenderbuffer;
    glGenRenderbuffers(1, &depthrenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, g_windowWidth, g_windowHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);
    
    // Set "renderedTexture" as our colour attachement #0
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);

    // Set the list of draw buffers.
    GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers
    
    // Always check that our framebuffer is ok
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        throw runtime_error(string("Cannot load frame buffer correctly "));
    }
    
    // Render to our framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, mirrorFBO);
    glViewport(0,0,g_windowWidth,g_windowHeight);

    // actually draw the things onto the frame buffer
    drawStuff(false, true);

    //glfwSwapBuffers(g_window);
    // set texture to be equal to buffer

    checkGlErrors();

}

static void display() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawStuff(false);

    glfwSwapBuffers(g_window);

    checkGlErrors();
}

static void pick() {
    // We need to set the clear color to black, for pick rendering.
    // so let's save the clear color
    GLdouble clearColor[4];
    glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

    glClearColor(0, 0, 0, 0);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawStuff(true);

    // Uncomment below to see result of the pick rendering pass
    //glfwSwapBuffers(g_window);

    // Now set back the clear color
    glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

    checkGlErrors();
}

static void reshape(GLFWwindow * window, const int w, const int h) {
    int width, height;
    glfwGetFramebufferSize(g_window, &width, &height);
    glViewport(0, 0, width, height);
    
    g_windowWidth = w;
    g_windowHeight = h;
    cerr << "Size of window is now " << g_windowWidth << "x" << g_windowHeight << endl;
    g_arcballScreenRadius = max(1.0, min(h, w) * 0.25);
    updateFrustFovY();
}

static Cvec3 getArcballDirection(const Cvec2 &p, const double r) {
    double n2 = norm2(p);
    if (n2 >= r * r)
        return normalize(Cvec3(p, 0));
    else
        return normalize(Cvec3(p, sqrt(r * r - n2)));
}

static RigTForm moveArcball(const Cvec2 &p0, const Cvec2 &p1) {
    const Matrix4 projMatrix = makeProjectionMatrix();
    const RigTForm eyeInverse =
        inv(getPathAccumRbt(g_world, g_currentCameraNode));
    const Cvec3 arcballCenter = getArcballRbt().getTranslation();
    const Cvec3 arcballCenter_ec = Cvec3(eyeInverse * Cvec4(arcballCenter, 1));

    if (arcballCenter_ec[2] > -CS175_EPS)
        return RigTForm();

    Cvec2 ballScreenCenter =
        getScreenSpaceCoord(arcballCenter_ec, projMatrix, g_frustNear,
                            g_frustFovY, g_windowWidth, g_windowHeight);
    const Cvec3 v0 =
        getArcballDirection(p0 - ballScreenCenter, g_arcballScreenRadius);
    const Cvec3 v1 =
        getArcballDirection(p1 - ballScreenCenter, g_arcballScreenRadius);

    return RigTForm(Quat(0.0, v1[0], v1[1], v1[2]) *
                    Quat(0.0, -v0[0], -v0[1], -v0[2]));
}

static RigTForm doMtoOwrtA(const RigTForm &M, const RigTForm &O,
                           const RigTForm &A) {
    return A * M * inv(A) * O;
}

static RigTForm getMRbt(const double dx, const double dy) {
    RigTForm M;

    
    
    if (g_mouseLClickButton && !g_mouseRClickButton && !g_spaceDown) {
        if (shouldUseArcball()) {
                M = moveArcball(Cvec2(g_mouseClickX, g_mouseClickY),
                                Cvec2(g_mouseClickX + dx, g_mouseClickY + dy));
            }
        else
            M = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
    } else {
        double movementScale =
            getManipMode() == EGO_MOTION ? 0.02 : g_arcballScale;
        if (g_mouseRClickButton && !g_mouseLClickButton) {
            M = RigTForm(Cvec3(dx, dy, 0) * movementScale);
        } else if (g_mouseMClickButton ||
                   (g_mouseLClickButton && g_mouseRClickButton) ||
                   (g_mouseLClickButton && g_spaceDown && g_currentPickedRbtNode != g_shaverNode)) {
            M = RigTForm(Cvec3(0, 0, -dy) * movementScale);
        }
    }

    switch (getManipMode()) {
    case ARCBALL_ON_PICKED:
        break;
    case ARCBALL_ON_SKY:
        M = inv(M);
        break;
    case EGO_MOTION:
        if (g_mouseLClickButton && !g_mouseRClickButton &&
            !g_spaceDown) // only invert rotation
            M = inv(M);
        break;
    }
    return M;
}

static RigTForm makeMixedFrame(const RigTForm &objRbt, const RigTForm &eyeRbt) {
    return transFact(objRbt) * linFact(eyeRbt);
}

// l = w X Y Z
// o = l O
// a = w A = l (Z Y X)^1 A = l A'
// o = a (A')^-1 O
//   => a M (A')^-1 O = l A' M (A')^-1 O

static void motion(GLFWwindow *window, double x, double y) {
    if (!g_mouseClickDown)
        return;

    const double dx = x - g_mouseClickX;
    const double dy = g_windowHeight - y - 1 - g_mouseClickY;

    const RigTForm M = getMRbt(dx, dy); // the "action" matrix

    // the matrix for the auxiliary frame (the w.r.t.)
    RigTForm A = makeMixedFrame(getArcballRbt(),
                                getPathAccumRbt(g_world, g_currentCameraNode));

    shared_ptr<SgRbtNode> target;
    switch (getManipMode()) {
    case ARCBALL_ON_PICKED:
        target = g_currentPickedRbtNode;
        break;
    case ARCBALL_ON_SKY:
        target = g_skyNode;
        break;
    case EGO_MOTION:
        target = g_currentCameraNode;
        break;
    }

    A = inv(getPathAccumRbt(g_world, target, 1)) * A;

    if ((g_mouseLClickButton && !g_mouseRClickButton &&
         !g_spaceDown) // rotating
        && target == g_skyNode) {
        RigTForm My = getMRbt(dx, 0);
        RigTForm Mx = getMRbt(0, dy);
        RigTForm B = makeMixedFrame(getArcballRbt(), RigTForm());
        RigTForm O = doMtoOwrtA(Mx, target->getRbt(), A);
        O = doMtoOwrtA(My, O, B);
        target->setRbt(O);
    } else {
        target->setRbt(doMtoOwrtA(M, target->getRbt(), A));
    }

    g_mouseClickX += dx;
    g_mouseClickY += dy;
}

static void mouse(GLFWwindow *window, int button, int state, int mods) {
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    g_mouseClickX = x;
    g_mouseClickY =
        g_windowHeight - y - 1; // conversion from GLFW window-coordinate-system
                                // to OpenGL window-coordinate-system

    g_mouseLClickButton |= (button == GLFW_MOUSE_BUTTON_LEFT && state == GLFW_PRESS);
    g_mouseRClickButton |= (button == GLFW_MOUSE_BUTTON_RIGHT && state == GLFW_PRESS);
    g_mouseMClickButton |= (button == GLFW_MOUSE_BUTTON_MIDDLE && state == GLFW_PRESS);

    g_mouseLClickButton &= !(button == GLFW_MOUSE_BUTTON_LEFT && state == GLFW_RELEASE);
    g_mouseRClickButton &= !(button == GLFW_MOUSE_BUTTON_RIGHT && state == GLFW_RELEASE);
    g_mouseMClickButton &= !(button == GLFW_MOUSE_BUTTON_MIDDLE && state == GLFW_RELEASE);

    g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

    if (g_pickingMode && button == GLFW_MOUSE_BUTTON_LEFT && state == GLFW_PRESS) {
        pick();
        g_pickingMode = false;
        cerr << "Picking mode is off" << endl;
    }
}

static void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
            case GLFW_KEY_SPACE:
                g_spaceDown = true;
                break;
            case GLFW_KEY_ESCAPE:
                exit(0);
            case GLFW_KEY_H:
                cout << " ============== H E L P ==============\n"
                << "h   \t\thelp menu\n\n"
                << "Welcome to the barbershop! Your task is to give our client, Tyrion Lannister, the hair cut of his life as he prepares for the battle to defend his countrymen. Use the controls below and the mouse to save the kingdom. \n\n"
                << "p\t\tUse mouse to select the clippers to  move or rotate them. \n"
                << "w,a,s,d \t Move the clippers up and down and side to side. \n"
                << "z,x \t\t Move the clippers in and out. \n"
                << "r   \t\t Reset Tyrion's hair to have fun all over again!"
                << endl;
                break;

            case GLFW_KEY_R: {
                for (int i = 0; i < g_headMesh.getNumFaces(); i++) {
                    g_hairLengths[i] = g_furHeight;
                }
                g_won = false;
            } break;
            case GLFW_KEY_P:
                g_pickingMode = !g_pickingMode;
                cerr << "Picking mode is " << (g_pickingMode ? "on" : "off") << endl;
                break;
                
            case GLFW_KEY_RIGHT:
                for (int i = 0; i < g_headMesh.getNumFaces(); i++) {
                    g_hairLengths[i] *= 1.05;
                }
                
                updateShellGeometry();
                break;
            case GLFW_KEY_LEFT:
                for (int i = 0; i < g_headMesh.getNumFaces(); i++) {
                    g_hairLengths[i] /= 1.05;
                }
                
                updateShellGeometry();
                break;
            case GLFW_KEY_UP:
                g_hairyness *= 1.05;
                cerr << "hairyness = " << g_hairyness << std::endl;
                updateShellGeometry();
                break;
            case GLFW_KEY_DOWN:
                g_hairyness /= 1.05;
                cerr << "hairyness = " << g_hairyness << std::endl;
                updateShellGeometry();
                break;
            case GLFW_KEY_W: {
                // move clipper deeper
                RigTForm M = RigTForm(Cvec3(0, depthSpd, 0));
                RigTForm A = makeMixedFrame(getArcballRbt(),
                                            getPathAccumRbt(g_world, g_currentCameraNode));
                A = inv(getPathAccumRbt(g_world, g_shaverNode, 1)) * A;
                g_shaverNode->setRbt(doMtoOwrtA(M, g_shaverNode->getRbt(), A));
                
            }
                break;
            case GLFW_KEY_S: {
                RigTForm M = RigTForm(Cvec3(0, -depthSpd, 0));
                RigTForm A = makeMixedFrame(getArcballRbt(),
                                            getPathAccumRbt(g_world, g_currentCameraNode));
                A = inv(getPathAccumRbt(g_world, g_shaverNode, 1)) * A;
                g_shaverNode->setRbt(doMtoOwrtA(M, g_shaverNode->getRbt(), A));
                
            }
                break;
            case GLFW_KEY_A: {
                // move clipper deeper
                // We created a translation RigTForm M which was responsible for movement in the x,y,z directions.
                // Then, we calculated an auxiliary mixed frame between the arcball Rbt and the accumulated rbt from world
                // to our current perspective. We redefine our auxiliary mixed frame A by inverting the accumulated rbt from the world to the shave node and left-multiplying
                // by its previous value (defined above). Finally, we apply our translation M to our trimmer rbt with respect to our auxiliary mixed frame A. In plain english, we ensure sure that our keyboard movements are intuitive by defining them in terms of the current perspective.
                
                RigTForm M = RigTForm(Cvec3(-depthSpd, 0, 0));
                RigTForm A = makeMixedFrame(getArcballRbt(),
                                            getPathAccumRbt(g_world, g_currentCameraNode));
                A = inv(getPathAccumRbt(g_world, g_shaverNode, 1)) * A;
                g_shaverNode->setRbt(doMtoOwrtA(M, g_shaverNode->getRbt(), A));
                
            }
                break;
            case GLFW_KEY_D: {
                // move clipper deeper
                RigTForm M = RigTForm(Cvec3(depthSpd, 0, 0));
                RigTForm A = makeMixedFrame(getArcballRbt(),
                                            getPathAccumRbt(g_world, g_currentCameraNode));
                A = inv(getPathAccumRbt(g_world, g_shaverNode, 1)) * A;
                g_shaverNode->setRbt(doMtoOwrtA(M, g_shaverNode->getRbt(), A));
                
            }
                break;
            case GLFW_KEY_Z: {
                // move clipper deeper
                RigTForm M = RigTForm(Cvec3(0, 0, -depthSpd));
                RigTForm A = makeMixedFrame(getArcballRbt(),
                                            getPathAccumRbt(g_world, g_currentCameraNode));
                A = inv(getPathAccumRbt(g_world, g_shaverNode, 1)) * A;
                g_shaverNode->setRbt(doMtoOwrtA(M, g_shaverNode->getRbt(), A));
                
            }
                break;
            case GLFW_KEY_X: {
                // pull clipper out
                
                RigTForm M = RigTForm(Cvec3(0, 0, depthSpd));
                RigTForm A = makeMixedFrame(getArcballRbt(),
                                            getPathAccumRbt(g_world, g_currentCameraNode));
                A = inv(getPathAccumRbt(g_world, g_shaverNode, 1)) * A;
                g_shaverNode->setRbt(doMtoOwrtA(M, g_shaverNode->getRbt(), A));
                
            }
                break;
            case GLFW_KEY_K: {
                // cheat code to win
                g_won = true;
            }break;
        }
    } else {
        switch(key) {
        case GLFW_KEY_SPACE:
            g_spaceDown = false;
            break;
        }
    }

}

void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}

static void initGlfwState() {
    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_SRGB_CAPABLE, GL_TRUE);

    g_window = glfwCreateWindow(g_windowWidth, g_windowHeight,
                                "Assignment 8", NULL, NULL);
    if (!g_window) {
        fprintf(stderr, "Failed to create GLFW window or OpenGL context\n");
        exit(1);
    }
    glfwMakeContextCurrent(g_window);
    glewInit();

    glfwSwapInterval(1);

    glfwSetErrorCallback(error_callback);
    glfwSetMouseButtonCallback(g_window, mouse);
    glfwSetCursorPosCallback(g_window, motion);
    glfwSetWindowSizeCallback(g_window, reshape);
    glfwSetKeyCallback(g_window, keyboard);

    int screen_width, screen_height;
    glfwGetWindowSize(g_window, &screen_width, &screen_height);
    int pixel_width, pixel_height;
    glfwGetFramebufferSize(g_window, &pixel_width, &pixel_height);

    cout << screen_width << " " << screen_height << endl;
    cout << pixel_width << " " << pixel_width << endl;

    g_wScale = pixel_width / screen_width;
    g_hScale = pixel_height / screen_height;
}

static void initGLState() {
    glClearColor(173. / 255., 216. / 255., 230. / 255., 0.);
    glClearDepth(0.);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_GREATER);
    glReadBuffer(GL_BACK);
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initMaterials() {
    // Create some prototype materials
    Material diffuse("./shaders/basic-gl3.vshader",
                     "./shaders/diffuse-gl3.fshader");
    Material solid("./shaders/basic-gl3.vshader",
                   "./shaders/solid-gl3.fshader");

    // copy diffuse prototype and set red color
    g_redDiffuseMat.reset(new Material(diffuse));
    g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

    // copy diffuse prototype and set blue color
    g_blueDiffuseMat.reset(new Material(diffuse));
    g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

    // normal mapping
    g_bumpFloorMat.reset(new Material("./shaders/floor.vshader",
                                      "./shaders/floor.fshader"));
    g_bumpFloorMat->getUniforms().put(
        "uTexColor",
        shared_ptr<ImageTexture>(new ImageTexture("wood.ppm", true)));

    // copy solid prototype, and set to wireframed rendering
    g_arcballMat.reset(new Material(solid));
    g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
    g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // copy solid prototype, and set to color white
    g_lightMat.reset(new Material(solid));
    g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));
    
    // pick shader
    g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader",
                                    "./shaders/pick-gl3.fshader"));
    
    // bunny material
    g_bunnyMat.reset(new Material("./shaders/basic-gl3.vshader",
                                  "./shaders/bunny-gl3.fshader"));
    g_bunnyMat->getUniforms().put("uColorAmbient", Cvec3f(0.45f, 0.3f, 0.3f))
                             .put("uColorDiffuse", Cvec3f(0.2f, 0.2f, 0.2f));
    
    // allocate array of materials
    g_bunnyShellMats.resize(g_numShells);
    g_bunnyShellMats[0].reset(new Material("./shaders/basic-gl3.vshader",
                                           "./shaders/bunny-gl3.fshader"));
    g_bunnyShellMats[0]->getUniforms().put("uColorAmbient", Cvec3f(0.45f, 0.3f, 0.3f))
                             .put("uColorDiffuse", Cvec3f(0.2f, 0.2f, 0.2f));
    
    // bunny material
    g_shaverMat.reset(new Material(diffuse));
    g_shaverMat->getUniforms().put("uColor", Cvec3f(255. / 255, 105. / 255, 180. / 255));
    
    // copy solid prototype, and set to color white
    g_chairMat.reset(new Material(diffuse));
    g_chairMat->getUniforms().put("uColor", Cvec3f(0.2, 0.2, 0.2));
    
    
    // bunny shell materials;
    // common shell texture:
    shared_ptr<ImageTexture> shellTexture(new ImageTexture("shell.ppm", false));

    // enable repeating of texture coordinates
    shellTexture->bind();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // eachy layer of the shell uses a different material, though the materials
    // will share the same shader files and some common uniforms. hence we
    // create a prototype here, and will copy from the prototype later
    Material bunnyShellMatPrototype("./shaders/bunny-shell-gl3.vshader",
                                    "./shaders/bunny-shell-gl3.fshader");
    bunnyShellMatPrototype.getUniforms().put("uTexShell", shellTexture);
    bunnyShellMatPrototype.getRenderStates()
        .blendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) // set blending mode
        .enable(GL_BLEND)                           // enable blending
        .disable(GL_CULL_FACE);                          // disable culling

    for (int i = 1; i < g_numShells; ++i) {
        // copy prototype
        g_bunnyShellMats[i].reset(new Material(bunnyShellMatPrototype));
        // but set a different exponent for blending transparency
        g_bunnyShellMats[i]->getUniforms().put(
            "uAlphaExponent", 2.f + 5.f * float(i + 1) / g_numShells);
    }
    
};

static void initNormals(Mesh &m) {
    for (int i = 0; i < m.getNumVertices(); ++i) {
        m.getVertex(i).setNormal(Cvec3(0));
    }
    for (int i = 0; i < m.getNumFaces(); ++i) {
        const Mesh::Face f = m.getFace(i);
        const Cvec3 n = f.getNormal();
        for (int j = 0; j < f.getNumVertices(); ++j) {
            f.getVertex(j).setNormal(f.getVertex(j).getNormal() + n);
        }
    }
    for (int i = 0; i < m.getNumVertices(); ++i) {
        Cvec3 n = m.getVertex(i).getNormal();
        if (norm2(n) > CS175_EPS2)
            m.getVertex(i).setNormal(normalize(n));
    }
}

static void initGeo(Mesh &m, shared_ptr<SimpleGeometryPN> &g) {
    vector<VertexPN> vec;
    vec.reserve(m.getNumFaces() * 3);
    
    for (int i = 0; i < m.getNumFaces(); i++) {
        for (int j = 0; j < m.getFace(i).getNumVertices(); j++) {
            Mesh::Vertex v = m.getFace(i).getVertex(j);
            VertexPN vertex = VertexPN(v.getPosition(), v.getNormal());
            // Make VertexPN
            vec.push_back(vertex);
        }
    }
    g.reset(new SimpleGeometryPN);
    
    VertexPN* vertices = &vec[0];
    g->upload(vertices, m.getNumFaces() * 3);
}

// New function that loads the bunny mesh and initializes the bunny shell meshes
static void initMeshes() {
    g_headMesh.load("tyrionhead.mesh");
    g_headNoHairMesh.load("tyrion_nohair.mesh");
    g_trimmerMesh.load("trimmer2.mesh");
    g_chairMesh.load("chair.mesh");

    //g_headMesh.subdivide();  
    // TODO: Init the per vertex normal of g_bunnyMesh; see "calculating
    // normals" section of spec
    // ...
    
    initNormals(g_headMesh);
    initNormals(g_headNoHairMesh);
    initNormals(g_trimmerMesh);
    initNormals(g_chairMesh);
    
    // TODO: Initialize g_bunnyGeometry from g_bunnyMesh; see "mesh preparation"
    // section of spec
    // ...
    
    initGeo(g_headMesh, g_headGeometry);
    initGeo(g_headNoHairMesh, g_headNoHairGeometry);
    initGeo(g_trimmerMesh, g_trimmerGeometry);
    initGeo(g_chairMesh, g_chairGeometry);

    // Now allocate array of SimpleGeometryPNX to for shells, one per layer
    g_headShellGeometries.resize(g_numShells);
    for (int i = 0; i < g_numShells; ++i) {
        g_headShellGeometries[i].reset(new SimpleGeometryPNX());
    }
    
    
}

static void initGeometry() {
    initGround();
    initCubes();
    initSphere();
    initMeshes();
}

static void initScene() {
    g_world.reset(new SgRootNode());

    g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 4, 6.0))));

    g_groundNode.reset(new SgRbtNode(RigTForm(Cvec3(0, g_groundY, 0))));
    g_groundNode->addChild(
        shared_ptr<MyShapeNode>(new MyShapeNode(g_ground, g_bumpFloorMat)));


    g_headNode.reset(new SgRbtNode(RigTForm(Cvec3(0, 4.4, 1))));

    // add bunny as a shape nodes
    g_headNode->addChild(
        shared_ptr<MyShapeNode>(new MyShapeNode(g_headGeometry, g_bunnyMat, Cvec3(0, 0, 0), Cvec3(0, 0, 0), Cvec3(1))));
  
    // add each shell as shape node
    g_headNode->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_headNoHairGeometry, g_bunnyMat, Cvec3(0, -.8, .4), Cvec3(0, 0, 0), Cvec3(1.1))));
    
    // helllooooo
    for (int i = 0; i < g_numShells; ++i) {
        g_headNode->addChild(shared_ptr<MyShapeNode>(
            new MyShapeNode(g_headShellGeometries[i], g_bunnyShellMats[i], Cvec3(0), Cvec3(0, 0, 0), Cvec3(1))));
    }
    
    g_chairNode.reset(new SgRbtNode(RigTForm(Cvec3(0, 0, 0))));
    g_chairNode->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_chairGeometry, g_chairMat, Cvec3(0, 1.6, 0.6), Cvec3(0, 0, 0), Cvec3(2))));
     
    g_shaverNode.reset(new SgRbtNode(RigTForm(Cvec3(-3, 5, 0))));

    g_cutNode.reset(new SgRbtNode(RigTForm(Cvec3(0, 0, 0.48))));
    g_cutNode->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_cube, g_bunnyMat, Cvec3(0), Cvec3(0), Cvec3(.01, .01, .01))));
    g_shaverNode->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_trimmerGeometry, g_shaverMat, Cvec3(0), Cvec3(0), Cvec3(.5, .5, .5))));
    g_shaverNode->addChild(g_cutNode);
    g_light1.reset(new SgRbtNode(RigTForm(Cvec3(4.0, 3.0, 5.0))));
    g_light2.reset(new SgRbtNode(RigTForm(Cvec3(-6, 10, -4.0))));

    g_light1->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_sphere, g_lightMat, Cvec3(0), Cvec3(0), Cvec3(0.5))));

    g_light2->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_sphere, g_lightMat, Cvec3(0), Cvec3(0), Cvec3(0.5))));
    
    // Remember to move the subdiv mesh node away a bit so it doesn't block the
    // bunny.

    // from this point, calling g_bunnyShellGeometries[i]->upload(...) will
    // change the geometry of the ith layer of shell that gets drawn



    g_world->addChild(g_skyNode);
    g_world->addChild(g_groundNode);
    //g_world->addChild(g_robot1Node);
   // g_world->addChild(g_robot2Node);
    g_world->addChild(g_light1);
    g_world->addChild(g_light2);
    g_world->addChild(g_shaverNode);
    g_world->addChild(g_chairNode);
    g_world->addChild(g_headNode);

    
//    if (!g_preRender) {
//        g_world->addChild(g_mirrorNode);
//    }
    
    g_currentCameraNode = g_skyNode;
}

//static void initAnimation() {
//    g_animator.attachSceneGraph(g_world);
//    g_curKeyFrame = g_animator.keyFramesBegin();
//}


static void glfwLoop() {
    g_lastFrameClock = glfwGetTime();

    while (!glfwWindowShouldClose(g_window)) {
        double thisTime = glfwGetTime();
        if( thisTime - g_lastFrameClock >= 1. / g_framesPerSecond) {

//            animationUpdate();
            g_lastFrameClock = glfwGetTime();

            checkCutLength();
            hairsSimulationUpdate();
        
            //preRender();

            display();
            g_lastFrameClock = thisTime;
        }

        glfwPollEvents();
    }
}

int main(int argc, char *argv[]) {
    
    try {

        initGlfwState();

        // on Mac, we shouldn't use GLEW.
#ifndef __MAC__
        glewInit(); // load the OpenGL extensions

#endif


        initGLState();
        initMaterials();
        initGeometry();
        initScene();
//        initAnimation();
        initSimulation();
        updateShellGeometry();

        glfwLoop();
        return 0;
    } catch (const runtime_error &e) {
        cout << "Exception caught: " << e.what() << endl;
        return -1;
    }
}
