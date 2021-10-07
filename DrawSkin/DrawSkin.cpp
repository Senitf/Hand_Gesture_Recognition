#pragma once
#include <cstdlib>
#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>

// Windows.h must be included before GLU:
#if defined(_WIN32)
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#endif
#  include <Windows.h>
#endif

#include <GL/glfw3.h>

//For Keyboard event
#include <gl/glut.h>

#include "../HandTrackingClient/HandTrackingListener.h"
#include "../HandTrackingClient/HandTrackingClient.h"
#include "../HandTrackingClient/Threads.h"

#if defined(__APPLE__)
#    include <OpenGL/GLU.h>
#else
#    include <gl/GLU.h>
#endif

using namespace HandTrackingClient;

#include <glm/glm.hpp>
#include <process.h>

//#define N_HANDS 1
Vector3f m_r_th_ft_pos, m_r_th_jnt_pos; //thumb
Vector3f m_r_id_ft_pos, m_r_id_jnt_pos; //index
Vector3f m_r_ro_ft_pos, m_r_ro_jnt_pos;//root
Vector3f m_r_id_jnt_rot[3], m_r_th_jnt_rot[3], m_r_ro_jnt_rot[3];
Vector3f m_r_id_jnt_cache, m_r_id_jnt_call;

Vector3f m_r_id_5_jnt_pos, m_r_id_6_jnt_pos, m_r_id_7_jnt_pos;

float tmpAngle;
/*
void display(GLFWwindow *pWindow, int viewportWidth, int viewportHeight);
unsigned __stdcall hapticRenderingThread(void* arg);

bool m_run_Athread = false;
bool m_run_haptic = true;
using namespace HandTrackingClient;
class DrawSkin;
DrawSkin *m_pDrawSkin;
HandTrackingClient::Client *m_pClient;
Vector3f m_r_ft_pos, m_r_th_ft_pos;
Vector3f m_r_jnt_pos, m_r_th_jnt_pos;
Vector3f m_r_jnt_rot[3], m_r_th_jnt_rot[3];
bool m_show_sphere = true;//false;
bool m_show_hand = false;
bool m_show_fingertip = true;//false;
float m_ft_r = 15.0;
GLUquadricObj *m_pSphrQuad = NULL;
GLUquadricObj *m_pQuadric = NULL;
float m_sphr_pos[3] = {0, 100, 0.0};
float m_sphr_r = 70.0;//100.0;
float m_top_pos = 200.0;
bool m_prev_coll[2] = {false, false};
*/
float prevPositions[3];
float prevNormals[3];
float prevTriangles[3];

void drawGroundPlane() 
{
    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 1.0f, 1.0f);

    const float spacing = 50.0f;
    const int numLines_z = 10;
    const int numLines_x = 10;
    const float zmax = static_cast<float>(numLines_z) * spacing;
    const float xmax = static_cast<float>(numLines_x) * spacing;
    
    glBegin(GL_LINES);
    for (int i = -numLines_z; i <= numLines_z; ++i)
    {
        const float z = static_cast<float>(i) * spacing;
        glVertex3f(-xmax, 0.0f, z);
        glVertex3f(xmax, 0.0f, z);
    }

    for (int i = -numLines_x; i <= numLines_x; ++i)
    {
        const float x = static_cast<float>(i) * spacing;
        glVertex3f(x, 0.0f, -zmax);
        glVertex3f(x, 0.0f, zmax);
    }
    glEnd();
}

class DrawSkin : public HandTrackingClient::HandTrackingListener
{
public:
    DrawSkin();
    ~DrawSkin();

    virtual void handleEvent(const HandTrackingClient::HandTrackingMessage& message);
    virtual void handleConnectionClosed() { }

    void renderSkin();
    void paint(int viewportWidth, int viewportHeight);

private:
    typedef UserMessage::Triangle Triangle;
    typedef UserMessage::IndicesVector IndicesVector;
    typedef UserMessage::WeightsVector WeightsVector;

    Mutex _dataMutex;
    std::vector<Vector3f> _restPositions[N_HANDS];
    std::vector<Triangle> _triangles[N_HANDS];

    // This is not an especially efficient way to store the skinning weights;
    // in a real application we'd pack them into a tighter array for cache
    // locality (or even better, move the skinning onto the GPU):
    std::vector<WeightsVector> _skinningWeights[N_HANDS];
    std::vector<IndicesVector> _skinningIndices[N_HANDS];

    std::array<Transformf, N_JOINTS> _restJointTransforms[N_HANDS];

    std::vector<Vector3f> _skinnedPositions[N_HANDS];
    std::vector<Vector3f> _skinnedVertexNormals[N_HANDS];

    void updateVertexNormals(); // Computes per-vertex normals from the positions

    HandTrackingClient::Client _client;
};


DrawSkin::DrawSkin()
{
}

DrawSkin::~DrawSkin()
{
}

void 
DrawSkin::handleEvent(const HandTrackingMessage& message)
{
    if (message.getType() == HandTrackingMessage::USER)
    {
        const UserMessage& userMessage = dynamic_cast<const UserMessage&> (message);
        SimpleLock lock (_dataMutex);
        for (int iHand = 1; iHand < N_HANDS; ++iHand)
        {
            _restPositions[iHand] = userMessage.getRestPositions(iHand);
            _triangles[iHand] = userMessage.getTriangles(iHand);
            _skinningIndices[iHand] = userMessage.getSkinningIndices(iHand);
            _skinningWeights[iHand] = userMessage.getSkinningWeights(iHand);
            _restJointTransforms[iHand] = userMessage.getRestJointTransforms(iHand);

            assert (_skinningIndices[iHand].size() == _restPositions[iHand].size());
            assert (_skinningWeights[iHand].size() == _restPositions[iHand].size());
        }

        for (int iHand = 1; iHand < N_HANDS; ++iHand)
            _skinnedPositions[iHand] = _restPositions[iHand];
        updateVertexNormals();
        //Refresh(false);
    }
    else if (message.getType() == HandTrackingMessage::POSE)
    {
        const PoseMessage& poseMessage = dynamic_cast<const PoseMessage&>(message);

        SimpleLock lock (_dataMutex);
        for (int iHand = 1; iHand < N_HANDS; ++iHand)
        {
            const std::array<Transformf, N_JOINTS>& restJoints = _restJointTransforms[iHand];
            std::array<Transformf, N_JOINTS> jointTransforms = poseMessage.getJointTransforms(iHand);
            std::array<Transformf, N_JOINTS> relativeJointTransforms;
            for (size_t jJoint = 0; jJoint < N_JOINTS; ++jJoint)
                relativeJointTransforms[jJoint] = jointTransforms[jJoint] * restJoints[jJoint].inverse();

            std::vector<Vector3f>& skinnedPos = _skinnedPositions[iHand];
            const std::vector<Vector3f>& restPos = _restPositions[iHand];
            skinnedPos.resize (_restPositions[iHand].size());
            for (size_t jVertex = 0; jVertex < skinnedPos.size(); ++jVertex)
            {
                const IndicesVector& boneIndices = _skinningIndices[iHand][jVertex];
                const WeightsVector& boneWeights = _skinningWeights[iHand][jVertex];
                const size_t nBones = boneIndices.size();
                assert (boneWeights.size() == nBones);

                Vector3f p (0, 0, 0);
                for (size_t kBone = 0; kBone < nBones; ++kBone)
                    p += boneWeights[kBone] * (relativeJointTransforms[boneIndices[kBone]] * restPos[jVertex]);
                skinnedPos[jVertex] = p;
            }
			//  LEFT_HAND = 0, RIGHT_HAND = 1
			//  THUMB = 0, INDEX FINGER = 1, MIDDLE FINGER = 2, RING FINGER = 3, PINKY FINGER = 4
			const std::array< Vector3f, N_FINGERS >& fingerTips = poseMessage.getFingerTips(1);
			Vector3f qv; float qw;
			//root

			//thumb
			m_r_th_jnt_pos = jointTransforms[4].translation;
			qv = jointTransforms[4].rotation.v;
			qw = jointTransforms[4].rotation.w;
			m_r_th_jnt_rot[0].x = 1-2*qv.y*qv.y-2*qv.z*qv.z; 
			m_r_th_jnt_rot[0].y = 2*qv.x*qv.y+2*qv.z*qw;
			m_r_th_jnt_rot[0].z = 2*qv.x*qv.z-2*qv.y*qw;
			//
			m_r_th_jnt_rot[1].x  = 2*qv.x*qv.y-2*qv.z*qw;
			m_r_th_jnt_rot[1].y = 1-2*qv.x*qv.x - 2*qv.z*qv.z;
			m_r_th_jnt_rot[1].z = 2*qv.y*qv.z + 2*qv.x*qw;
			//
			m_r_th_jnt_rot[2].x = 2*qv.x*qv.z + 2*qv.y*qw;
			m_r_th_jnt_rot[2].y = 2*qv.y*qv.z - 2*qv.x*qw;
			m_r_th_jnt_rot[2].z = 1- 2*qv.x*qv.x - 2*qv.y*qv.y;
			m_r_th_ft_pos.x = fingerTips[0].x;
			m_r_th_ft_pos.y = fingerTips[0].y;
			m_r_th_ft_pos.z = fingerTips[0].z;
			// index
			m_r_id_jnt_pos = jointTransforms[7].translation;
			//m_r_jnt_rot = jointTransforms[7].rotation.v;
			qv = jointTransforms[7].rotation.v;
			qw = jointTransforms[7].rotation.w;
			m_r_id_jnt_rot[0].x = 1-2*qv.y*qv.y-2*qv.z*qv.z; 
			m_r_id_jnt_rot[0].y = 2*qv.x*qv.y+2*qv.z*qw;
			m_r_id_jnt_rot[0].z = 2*qv.x*qv.z-2*qv.y*qw;
			//
			m_r_id_jnt_rot[1].x  = 2*qv.x*qv.y-2*qv.z*qw;
			m_r_id_jnt_rot[1].y = 1-2*qv.x*qv.x - 2*qv.z*qv.z;
			m_r_id_jnt_rot[1].z = 2*qv.y*qv.z + 2*qv.x*qw;
			//
			m_r_id_jnt_rot[2].x = 2*qv.x*qv.z + 2*qv.y*qw;
			m_r_id_jnt_rot[2].y = 2*qv.y*qv.z - 2*qv.x*qw;
			m_r_id_jnt_rot[2].z = 1- 2*qv.x*qv.x - 2*qv.y*qv.y;
			m_r_id_ft_pos.x = fingerTips[1].x;
			m_r_id_ft_pos.y = fingerTips[1].y;
			m_r_id_ft_pos.z = fingerTips[1].z;

			tmpAngle = acos((m_r_id_jnt_rot[0].x + m_r_id_jnt_rot[1].y + m_r_id_jnt_rot[2].z - 1) / 2);

			//root
			m_r_ro_jnt_pos = jointTransforms[0].translation;
			//m_r_jnt_rot = jointTransforms[7].rotation.v;
			qv = jointTransforms[0].rotation.v;
			qw = jointTransforms[0].rotation.w;
			m_r_ro_jnt_rot[0].x = 1-2*qv.y*qv.y-2*qv.z*qv.z; 
			m_r_ro_jnt_rot[0].y = 2*qv.x*qv.y+2*qv.z*qw;
			m_r_ro_jnt_rot[0].z = 2*qv.x*qv.z-2*qv.y*qw;
			//
			m_r_ro_jnt_rot[1].x  = 2*qv.x*qv.y-2*qv.z*qw;
			m_r_ro_jnt_rot[1].y = 1-2*qv.x*qv.x - 2*qv.z*qv.z;
			m_r_ro_jnt_rot[1].z = 2*qv.y*qv.z + 2*qv.x*qw;
			//
			m_r_ro_jnt_rot[2].x = 2*qv.x*qv.z + 2*qv.y*qw;
			m_r_ro_jnt_rot[2].y = 2*qv.y*qv.z - 2*qv.x*qw;
			m_r_ro_jnt_rot[2].z = 1- 2*qv.x*qv.x - 2*qv.y*qv.y;

			//Test 1
			/*
			std::cout << " Hand #:" << 1 << " fingerTip #:" << 0 << "(" <<
			m_r_ro_jnt_pos.x << ", " << m_r_ro_jnt_pos.y << ", " << m_r_ro_jnt_pos.z << ")" << std::endl;
			std::cout << " Hand #:" << 1 << " fingerTip #:" << 1 << "(" <<
			m_r_id_jnt_pos.x << ", " << m_r_id_jnt_pos.y << ", " << m_r_id_jnt_pos.z << ")" << std::endl;

			m_r_id_jnt_cache = m_r_id_jnt_pos;
			if(m_r_id_jnt_pos.x < m_r_id_jnt_call.x - 60.){
				printf("Left\n");
			}
			else if(m_r_id_jnt_pos.x > m_r_id_jnt_call.x + 60.){
				printf("Right\n");
			}
			*/

			//Test2
			m_r_id_5_jnt_pos = jointTransforms[5].translation;
			m_r_id_6_jnt_pos = jointTransforms[6].translation;
			m_r_id_7_jnt_pos = jointTransforms[7].translation;

			/*
			std::cout << " Hand #:" << 1 << " id jnt #:" << 5 << "(" <<
			m_r_id_5_jnt_pos.x << ", " << m_r_id_5_jnt_pos.y << ", " << m_r_id_5_jnt_pos.z << ")" << std::endl;

			std::cout << " Hand #:" << 1 << " id jnt #:" << 6 << "(" <<
			m_r_id_6_jnt_pos.x << ", " << m_r_id_6_jnt_pos.y << ", " << m_r_id_6_jnt_pos.z << ")" << std::endl;

			std::cout << " Hand #:" << 1 << " id jnt #:" << 7 << "(" <<
			m_r_id_7_jnt_pos.x << ", " << m_r_id_7_jnt_pos.y << ", " << m_r_id_7_jnt_pos.z << ")" << std::endl;
			*/

			std::cout << " Hand #:" << 1 << " id jnt #:" << 7 << "(" <<
			tmpAngle << ")" << std::endl;

			/*
			for (int iFinger = 0; iFinger < N_FINGERS; ++iFinger)
			{
			std::cout << " Hand #:" << iHand << " fingerTip #:" << iFinger << "(" <<
			fingerTips[iFinger].x << ", " << fingerTips[iFinger].y << ", " << fingerTips[iFinger].z << ")" << std::endl;
			}
			*/

        }
        updateVertexNormals();
        //Refresh(false);
    }
}

void 
DrawSkin::renderSkin()
{
    SimpleLock lock (_dataMutex);
	/*
	edit something
	int iHand = 1;
	*/
    for (int iHand = 1; iHand < N_HANDS; ++iHand)
    {
        const std::vector<Vector3f>& positions = _skinnedPositions[iHand];
        const std::vector<Vector3f>& normals = _skinnedVertexNormals[iHand];
        const std::vector<Triangle>& triangles = _triangles[iHand];

        if (positions.empty() || normals.empty() || triangles.empty())
            continue;
		/*
		for(int i = 0; i < 3; i++){
			prevPositions[i] = positions[i].x;
			printf("%lf\n", prevPositions[i]);
		}
		*/

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, &positions[0]);
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, 0, &normals[0]);

        glEnable(GL_LIGHTING);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, (GLsizei) (3*triangles.size()), GL_UNSIGNED_INT, &triangles[0]);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
    }
}


void
DrawSkin::updateVertexNormals()
{
    for (int iHand = 1; iHand < N_HANDS; ++iHand)
    {
        const std::vector<Vector3f>& positions = _skinnedPositions[iHand];
        std::vector<Vector3f>& normals = _skinnedVertexNormals[iHand];
        normals.resize (positions.size());
        std::fill (normals.begin(), normals.end(), Vector3f(0,0,0));

        for (size_t jTriangle = 0; jTriangle < _triangles[iHand].size(); ++jTriangle)
        {
            const Triangle& tri = _triangles[iHand][jTriangle];
            const Vector3f crossProd = 
                (positions[tri[1]] - positions[tri[0]]).cross (positions[tri[2]] - positions[tri[0]]);

            for (int k = 0; k < 3; ++k)
                normals[tri[k]] += crossProd;
        }

        for (size_t jVertex = 0; jVertex < normals.size(); ++jVertex)
            normals[jVertex].normalize();
    }
}


void DrawSkin::paint(int viewportWidth, int viewportHeight)
{
    // Special case: avoid division by zero below
    viewportHeight = viewportHeight > 0 ? viewportHeight : 1;

    // Setup the OpenGL state.  Our world is measured in millimeters 
    glViewport(0, 0, viewportWidth, viewportHeight);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Select and setup the projection matrix
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective (45.0f, 
                    static_cast<GLfloat>(viewportWidth) / static_cast<GLfloat>(viewportHeight), 
                    10.0f, 
                    10000.0f );

    // Select and setup the modelview matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    gluLookAt(0, 450, 550,  // eye position
              0, 125,   0,  // viewpoint
              0,    1,  0); // up vector

    // General OpenGL setup:
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glDisable(GL_AUTO_NORMAL);
    glDisable(GL_NORMALIZE);
    glDisable(GL_TEXTURE_2D);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);

    // Lighting setup:
     {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
        glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);

        float lightAmbient0[] = {0.2f, 0.2f, 0.2f, 1.0f};
        float lightPosition0[] = {1.0f, -2.0f, 1.0f, 0.0f};
        float lightDiffuse0[] = {0.5f, 0.5f, 0.5f, 1.0f};
        float lightPosition1[] = {-2.0f, -1.0f, 5.0f, 0.0f};
        float lightDiffuse1[] = {0.5f, 0.5f, 0.5f, 1.0f};

        {
            float sqrLen = 0.0f;
            for (int i = 0; i < 4; ++i)
                sqrLen += lightPosition0[i];
            for (int i = 0; i < 4; ++i)
                lightPosition0[i] /= std::sqrt(sqrLen);
        }

        {
            float sqrLen = 0.0f;
            for (int i = 0; i < 4; ++i)
                sqrLen += lightPosition1[i];
            for (int i = 0; i < 4; ++i)
                lightPosition1[i] /= std::sqrt(sqrLen);
        }

        glLightfv( GL_LIGHT0, GL_POSITION, lightPosition0 );
        glLightfv( GL_LIGHT0, GL_DIFFUSE, lightDiffuse0 );
        glLightfv( GL_LIGHT0, GL_AMBIENT, lightAmbient0 );
        glLightfv( GL_LIGHT1, GL_POSITION, lightPosition1 );
        glLightfv( GL_LIGHT1, GL_DIFFUSE, lightDiffuse1 );

        glEnable( GL_LIGHT0 );
        glEnable( GL_LIGHT1 );

        glPopMatrix();
    }
/* something edit
{
		GLfloat lightZeroPosition[]  = {0.0, 3.0, -3.5, 0.0};//{10.0, 4.0, 100.0, 0.0};// 
		GLfloat lightZeroColor[]     = {0.8, 0.8, 0.8, 1.0}; 
		GLfloat lightOnePosition[]   = {0.0, -3.0, 3.5, 0.0};//{-1.0, -2.0, -100.0, 0.0};//
		GLfloat lightOneColor[]      = {0.8, 0.8, 0.8, 1.0};
		glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
		glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHT1);
		glEnable(GL_COLOR_MATERIAL);

		glEnable(GL_LIGHTING);
		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);
		glEnable(GL_DEPTH_TEST);

	 }
	 */
    drawGroundPlane();

    {
        const float material[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, material);
        renderSkin();
    }
}


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        exit(0);//printf("KangGeon Kim\n");//activate_airship();
	else if(key == GLFW_KEY_C){
		//printf("C pressed\n");
		m_r_id_jnt_call = m_r_id_jnt_cache;
	}
}
/*
void display(GLFWwindow *pWindow, int viewportWidth, int viewportHeight)
{
	const float material[4] = {1.0f, 0.0f, 0.0f, 0.0f};
//	const float material[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	const float sphr_material[4] = {0.5, 0.5, 1.0, 0.0f};
	const float sphr_coll_material[4] = {1, 0, 0, 0};
	const float fingertip_material[4] = {0, 1, 0, 1};

   // Special case: avoid division by zero below
    viewportHeight = viewportHeight > 0 ? viewportHeight : 1;

    // Setup the OpenGL state.  Our world is measured in millimeters 
    glViewport(0, 0, viewportWidth, viewportHeight);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Select and setup the projection matrix
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective (45.0f, 
                    static_cast<GLfloat>(viewportWidth) / static_cast<GLfloat>(viewportHeight), 
                    10.0f, 
                    10000.0f );

    // Select and setup the modelview matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    gluLookAt(0, 450, 550,  // eye position
              0, 125,   0,  // viewpoint
              0,    1,  0); // up vector

    // General OpenGL setup:
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glDisable(GL_AUTO_NORMAL);
    glDisable(GL_NORMALIZE);
    glDisable(GL_TEXTURE_2D);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);

    // Lighting setup:
    // {
    //    glMatrixMode(GL_MODELVIEW);
    //    glPushMatrix();
    //    glLoadIdentity();

    //    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
    //    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);

    //    float lightAmbient0[] = {0.2f, 0.2f, 0.2f, 0.0f};
    //    float lightPosition0[] = {1.0f, -2.0f, 1.0f, 0.0f};
    //    float lightDiffuse0[] = {0.5f, 0.5f, 0.5f, 1.0f};
    //    float lightPosition1[] = {-2.0f, -1.0f, 5.0f, 0.0f};
    //    float lightDiffuse1[] = {0.5f, 0.5f, 0.5f, 1.0f};
    //    {
    //        float sqrLen = 0.0f;
    //        for (int i = 0; i < 4; ++i)
    //            sqrLen += lightPosition0[i];
    //        for (int i = 0; i < 4; ++i)
    //            lightPosition0[i] /= std::sqrt(sqrLen);
    //    }

    //    {
    //        float sqrLen = 0.0f;
    //        for (int i = 0; i < 4; ++i)
    //            sqrLen += lightPosition1[i];
    //        for (int i = 0; i < 4; ++i)
    //            lightPosition1[i] /= std::sqrt(sqrLen);
    //    }

    //    glLightfv( GL_LIGHT0, GL_POSITION, lightPosition0 );
    //    glLightfv( GL_LIGHT0, GL_DIFFUSE, lightDiffuse0 );
    //    glLightfv( GL_LIGHT0, GL_AMBIENT, lightAmbient0 );
    //    glLightfv( GL_LIGHT1, GL_POSITION, lightPosition1 );
    //    glLightfv( GL_LIGHT1, GL_DIFFUSE, lightDiffuse1 );

    //    glEnable( GL_LIGHT0 );
    //    glEnable( GL_LIGHT1 );

    //    glPopMatrix();
    //}
	 {
		GLfloat lightZeroPosition[]  = {0.0, 3.0, -3.5, 0.0};//{10.0, 4.0, 100.0, 0.0};// 
		GLfloat lightZeroColor[]     = {0.8, 0.8, 0.8, 1.0}; 
		GLfloat lightOnePosition[]   = {0.0, -3.0, 3.5, 0.0};//{-1.0, -2.0, -100.0, 0.0};//
		GLfloat lightOneColor[]      = {0.8, 0.8, 0.8, 1.0};
		glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
		glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
		glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHT1);
		glEnable(GL_COLOR_MATERIAL);

		glEnable(GL_LIGHTING);
		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);
		glEnable(GL_DEPTH_TEST);
		if(m_show_sphere) {
			glPushMatrix();
			glColor3f(0, 0, 1);
			//	if(m_prev_coll) glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, sphr_coll_material);
			//	else glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, sphr_material);
			glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, sphr_material);
			glTranslatef(m_sphr_pos[0], m_sphr_pos[1], m_sphr_pos[2]);
			gluSphere(m_pSphrQuad, m_sphr_r, 20, 20);
			glPopMatrix();
		}
		if(m_show_fingertip){
			if(!m_prev_coll[0]) glColor3f(0, 1, 0);
			else glColor3f(1, 0, 0);
			glPushMatrix();
			//	glEnable(GL_LIGHTING);
			    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, fingertip_material);
				glTranslatef(m_r_th_ft_pos.x, m_r_th_ft_pos.y, m_r_th_ft_pos.z);
				gluSphere(m_pQuadric, m_ft_r, 20, 20);
			glPopMatrix();
			if(!m_prev_coll[1]) glColor3f(0, 1, 0);
			else glColor3f(1, 0, 0);
			glPushMatrix();
			//	glEnable(GL_LIGHTING);
			    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, fingertip_material);
				glTranslatef(m_r_ft_pos.x, m_r_ft_pos.y, m_r_ft_pos.z);
				gluSphere(m_pQuadric, m_ft_r, 20, 20);
			glPopMatrix();
		}
	 }
	 glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, material);
	 glColor3f(0.9, 0.6, 0.7);
	 m_pDrawSkin->renderSkin();
    glFlush();
	glfwSwapBuffers(pWindow);
}
*/



int main(void)
{
	// something edit this statement deleted
    DrawSkin drawSkin;

	/*
	se
	m_pDrawSkin = new DrawSkin();
	m_pClient = new HandTrackingClient::Client();
	m_pClient->addHandTrackingListener(m_pDrawSkin);
	*/
	
    HandTrackingClient::Client client; //se deleted
    client.addHandTrackingListener(&drawSkin); //deleted
    std::pair<bool, std::string> result = client.connect(); //replace to m_pClient->connect();
    if (!result.first)
    {
        printf("Unable to connect to hand tracking server: %s\n", result.second.c_str());
        return -1;
    }

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    struct GLFWTerminator
    {
        ~GLFWTerminator() { glfwTerminate(); }
    } terminator;

    /* Create a windowed mode window and its OpenGL context */
    GLFWwindow* window = 
        glfwCreateWindow( 800, 600, "Draw Skin Demo", NULL, NULL);
    if (window == NULL)
        return EXIT_FAILURE;

	glfwSetKeyCallback(window, key_callback);

	/*
	se
	glfwSetKeyCallback(pWindow, key_callback);
	m_pQuadric = gluNewQuadric();			// Create A Pointer To The Quadric Object  
	gluQuadricTexture(m_pQuadric, GL_TRUE);		// Create Texture Coords   
	gluQuadricNormals(m_pQuadric, GLU_SMOOTH);	// Create Smooth Normals 
	m_pSphrQuad = gluNewQuadric();		// Test sphere quadric
	gluQuadricTexture(m_pSphrQuad, GL_TRUE);		// Create Texture Coords   
	gluQuadricNormals(m_pSphrQuad, GLU_SMOOTH);	// Create Smooth Normals 

	_beginthreadex(NULL, 0, hapticRenderingThread, NULL, 0, NULL);
	*/

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        int viewportWidth, viewportHeight;
        glfwGetWindowSize(window, &viewportWidth, &viewportHeight);

		/*se replaced
		glfwMakeContextCurrent(pWindow);
		display(pWindow, viewportWidth, viewportHeight);
		*/
        /* Render here */
        glfwMakeContextCurrent(window);
        drawSkin.paint(viewportWidth, viewportHeight);
        glFlush();
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    client.stop(); //replaced to m_pClient->stop();
    return 0;
}

/*
se
unsigned __stdcall hapticRenderingThread(void* arg)
{
	bool curr_coll[2] = {false, false};
	glm::vec3 coll_vec[2], coll_vec2[2];
	float d_c2c[2], coll_depth[2] = {0.0, 0.0};
	int i;
	do {
		coll_vec[0] = glm::vec3(m_r_th_ft_pos.x - m_sphr_pos[0], m_r_th_ft_pos.y - m_sphr_pos[1], m_r_th_ft_pos.z - m_sphr_pos[2]);
		coll_vec[1] = glm::vec3(m_r_ft_pos.x - m_sphr_pos[0], m_r_ft_pos.y - m_sphr_pos[1], m_r_ft_pos.z - m_sphr_pos[2]);
		for(i=0;i<2;i++) {
			d_c2c[i] = sqrt(coll_vec[i][0]*coll_vec[i][0] + coll_vec[i][1]*coll_vec[i][1] + coll_vec[i][2]*coll_vec[i][2]);
			coll_vec2[i] = -coll_vec[i];
			if(d_c2c[i] <= (m_ft_r+m_sphr_r)) {
				curr_coll[i] = true;

			}
			else curr_coll[i] = false;
			m_prev_coll[i] = curr_coll[i];
			coll_depth[i] = m_ft_r + m_sphr_r - d_c2c[i];

		}
	} while(m_run_haptic);
	return 0;
}
*/