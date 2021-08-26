#include <cstdlib>
#include <vector>
#include <cassert>
#include <iostream>

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

#include "../HandTrackingClient/HandTrackingListener.h"
#include "../HandTrackingClient/HandTrackingClient.h"
#include "../HandTrackingClient/Threads.h"

#if defined(__APPLE__)
#    include <OpenGL/GLU.h>
#else
#    include <gl/GLU.h>
#endif
#include <glm\glm.hpp>
#include <process.h>
#include <826api.h>
#include <math.h>
#include <time.h>
#include <ft2build.h>
#include <string>
#include FT_FREETYPE_H
#include "stat_helper.h"
#include "CMCI_sound.h"

enum EXP_PHASE {
	INFO_INPUT = 0,				// 
	TRAINING_PHASE,	
	EXP_PHASE1,				// Experiment initialization
	EXP_PHASE2,				// Checks if the contact point is close to the contact surface
//	EXP_PHASE3,				// A participant feels the 1st stimulus of curren tirla.
	EXP_ANSWER,				// Type the anser and hit 'Enter'
	EXP_ANSWER_CORRECTNESS,	// 
	DATA_ANALYSIS,
	WRITE_RESULT,
	EXP_DONE,
	QUIT_EXP
};

enum RECORD_TYPE {
	REC_INIT = 0,
	REC_TRIAL,
	REC_END
};

enum AUDIO_PHASE {
	AUDIO_INIT = 0,
	PLAY,
	PAUSE,
	STOP,
	COMPLETE
};

const char m_instruction_msg[][128] = {
	{"Type subject ID hit 'Enter' to begin the training session"},	// INFO_INPUT
	{"Virtual sphere size (1: reference; 2: comparison). Press 'e' to begin the experiment."},	// TRAINING_PHASE
	{"Place your hand close to the red sphere."}, // EXP_PHASE1
	{"Touch the virtual sphere to feel its size. Hit 'Enter' to type your answer."},	// EXP_PHASE2
	{"Was the object as big as the reference size? Type your answer (1: yes (reference); 2: no (comarison))"},	// EXP_ANSWER
	{"Hit 'Enter' to move to the next trial."}, // EXP_ANSWER_CORRECTNESS
	{""},	// DATA_ANALYSIS
	{""},	// WRITE_RESULT
	{"Experiment Complete. Thank you for your participation."}	// EXP_DONE	
};

struct exp_result {
	int trial_no;
	time_t trial_begin_time;
	time_t trial_end_time;
	int trial_stimulus;		// 0: reference 1: comparison
	int trial_user_ans;
	double mean_coll_depth;
};

void display(GLFWwindow *pWindow, int viewportWidth, int viewportHeight);
unsigned __stdcall AnalogOutThread(void* arg);
unsigned __stdcall hapticRenderingThread(void* arg);
unsigned __stdcall audioThread(void* arg);
void init_s826();
int moveToNextPhase();
void getErrorString(int errcode, char* ret_string, int board_no);
bool check_directory(char *directory_path);
void recordResult(int type);
void setAudioPhase(int audio_phase);

using namespace HandTrackingClient;
class DrawSkin;
DrawSkin *m_pDrawSkin;
HandTrackingClient::Client *m_pClient;
Vector3f m_r_ft_pos, m_r_th_ft_pos;
Vector3f m_r_jnt_pos, m_r_th_jnt_pos;
Vector3f m_r_jnt_rot[3], m_r_th_jnt_rot[3];
bool m_show_sphere = true;//false;
bool m_show_hand = false;
bool m_show_fingertip = false;
bool m_show_sphr_top = false;
float m_ft_r = 15.0;
GLUquadricObj *m_pSphrQuad = NULL;
GLUquadricObj *m_pQuadric = NULL;
GLUquadricObj *m_pTopQuad = NULL;
float m_sphr_pos[3] = {0, 100, 0.0};
double m_sphr_top_pos[3] = {0, 100+100, 0.0};
///////

bool m_prev_coll[2] = {false, false};
////////////////////////
// sensoray 826 related variables
////////////////////////////
int m_board[2];		// s826 boards index
bool m_run_Athread = false;
bool m_run_haptic = true;
CRITICAL_SECTION m_cs;
float m_piezo_env[2][5] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
float m_coll_depth[2] = {0.0, 0.0};
bool m_enable_rendering = false;
bool m_dbg_flag[10] = {false, false, false, false, false, false, false, false, false, false};
glm::vec3 m_act_pt[10];
////////////////////////////////////
///// exeperiment related variables
/////////////////////////////
float m_ref_size = 100.0;
float m_comp_size =150; //125, 150
////////////////////////////
int m_exp_cond =0;	// 0: lateral(4)+normal(1, bottom); 1: lateral(4) 1
/////
float m_sphr_r = 100.0;//100.0;
float m_top_pos = 200.0;
int m_exp_phase = EXP_PHASE::INFO_INPUT;
int m_audio_phase;
static HANDLE m_hAudioEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
char m_txtBuf[65] = {NULL};
char m_subjectID[65] = { NULL };
int m_textBuf_len = 0;
bool m_testSubject = false;
std::vector<exp_result> m_expResult;
char m_rec_filename[128];
int m_curr_trial_no = -1;
time_t m_expBeginTime;
time_t m_trialBeginTime;
time_t m_trialEndTime;
int m_tot_trial_number = 20;
int m_curr_stimulus = -1;
int m_curr_answer = -1;
int m_n_ref = 0, m_n_comp = 0;
int m_n_H = 0, m_n_F = 0, m_correct_ans = 0;
double m_H, m_F, m_d, m_c;
FT_Library m_ft;
FT_Face m_face;
FT_GlyphSlot m_g;
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

void DrawSkin::handleEvent(const HandTrackingMessage& message)
{
    if (message.getType() == HandTrackingMessage::USER)
    {
        const UserMessage& userMessage = dyna22mic_cast<const UserMessage&> (message);
        SimpleLock lock (_dataMutex);
        for (int iHand = 0; iHand < N_HANDS; ++iHand)
        {
            _restPositions[iHand] = userMessage.getRestPositions(iHand);
            _triangles[iHand] = userMessage.getTriangles(iHand);
            _skinningIndices[iHand] = userMessage.getSkinningIndices(iHand);
            _skinningWeights[iHand] = userMessage.getSkinningWeights(iHand);
            _restJointTransforms[iHand] = userMessage.getRestJointTransforms(iHand);

            assert (_skinningIndices[iHand].size() == _restPositions[iHand].size());
            assert (_skinningWeights[iHand].size() == _restPositions[iHand].size());
        }

        for (int iHand = 0; iHand < N_HANDS; ++iHand)
            _skinnedPositions[iHand] = _restPositions[iHand];
        updateVertexNormals();
        //Refresh(false);
    }
    else if (message.getType() == HandTrackingMessage::POSE)
    {
        const PoseMessage& poseMessage = dynamic_cast<const PoseMessage&>(message);

        SimpleLock lock (_dataMutex);
        for (int iHand = 0; iHand < N_HANDS; ++iHand)
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
			const std::array< Vector3f, N_FINGERS >& fingerTips = poseMessage.getFingerTips(iHand);
			Vector3f qv; float qw;
			/// thumb
			m_r_th_jnt_pos = jointTransforms[4].translation;
			qv = jointTransforms[4].rotation.v;
			qw = jointTransforms[4].rotation.w;
			m_r_th_jnt_rot[0].x = 1-2*qv.y*qv.y-2*qv.z*qv.z; 
			m_r_th_jnt_rot[0].y = 2*qv.x*qv.y+2*qv.z*qw;
			m_r_th_jnt_rot[0].z = 2*qv.x*qv.z-2*qv.y*qw;
			///
			m_r_th_jnt_rot[1].x  = 2*qv.x*qv.y-2*qv.z*qw;
			m_r_th_jnt_rot[1].y = 1-2*qv.x*qv.x - 2*qv.z*qv.z;
			m_r_th_jnt_rot[1].z = 2*qv.y*qv.z + 2*qv.x*qw;
			///
			m_r_th_jnt_rot[2].x = 2*qv.x*qv.z + 2*qv.y*qw;
			m_r_th_jnt_rot[2].y = 2*qv.y*qv.z - 2*qv.x*qw;
			m_r_th_jnt_rot[2].z = 1- 2*qv.x*qv.x - 2*qv.y*qv.y;
			m_r_th_ft_pos.x = fingerTips[0].x;
			m_r_th_ft_pos.y = fingerTips[0].y;
			m_r_th_ft_pos.z = fingerTips[0].z;
			// index
			m_r_jnt_pos = jointTransforms[7].translation;
		//		m_r_jnt_rot = jointTransforms[7].rotation.v;
			qv = jointTransforms[7].rotation.v;
			qw = jointTransforms[7].rotation.w;
			m_r_jnt_rot[0].x = 1-2*qv.y*qv.y-2*qv.z*qv.z; 
			m_r_jnt_rot[0].y = 2*qv.x*qv.y+2*qv.z*qw;
			m_r_jnt_rot[0].z = 2*qv.x*qv.z-2*qv.y*qw;
			///
			m_r_jnt_rot[1].x  = 2*qv.x*qv.y-2*qv.z*qw;
			m_r_jnt_rot[1].y = 1-2*qv.x*qv.x - 2*qv.z*qv.z;
			m_r_jnt_rot[1].z = 2*qv.y*qv.z + 2*qv.x*qw;
			///
			m_r_jnt_rot[2].x = 2*qv.x*qv.z + 2*qv.y*qw;
			m_r_jnt_rot[2].y = 2*qv.y*qv.z - 2*qv.x*qw;
			m_r_jnt_rot[2].z = 1- 2*qv.x*qv.x - 2*qv.y*qv.y;
			m_r_ft_pos.x = fingerTips[1].x;
			m_r_ft_pos.y = fingerTips[1].y;
			m_r_ft_pos.z = fingerTips[1].z;
		//	for (int iFinger = 0; iFinger < N_FINGERS; ++iFinger)
		//	{
		////		std::cout << " Hand #:" << iHand << " fingerTip #:" << iFinger << "(" <<
  //       //       fingerTips[iFinger].x << ", " << fingerTips[iFinger].y << ", " << fingerTips[iFinger].z << ")" << std::endl;
		//	}

        }

        updateVertexNormals();
        //Refresh(false);
    }
}

void DrawSkin::renderSkin()
{
    SimpleLock lock (_dataMutex);
  //  for (int iHand = 0; iHand < N_HANDS; ++iHand)
	int iHand = 1;
    {
        const std::vector<Vector3f>& positions = _skinnedPositions[iHand];
        const std::vector<Vector3f>& normals = _skinnedVertexNormals[iHand];
        const std::vector<Triangle>& triangles = _triangles[iHand];

        if (positions.empty() || normals.empty() || triangles.empty())
            return; //continue;

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


void DrawSkin::updateVertexNormals()
{
    for (int iHand = 0; iHand < N_HANDS; ++iHand)
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

	 }
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
        exit(0);
	if (key == GLFW_KEY_F7 && action == GLFW_PRESS) {
		if(!m_show_sphere) m_show_sphere = true;
		else m_show_sphere = false;
	}
	if(key == GLFW_KEY_F8 && action == GLFW_PRESS) {
		if(!m_show_fingertip) m_show_fingertip = true;
		else m_show_fingertip = false;
	}
	if(key == GLFW_KEY_F9 && action == GLFW_PRESS) {
		if(!m_enable_rendering) m_enable_rendering = true;
		else m_enable_rendering = false;
	}
	if(m_exp_phase == EXP_PHASE::INFO_INPUT) {

		if(action == GLFW_PRESS) {
			if(key == GLFW_KEY_ENTER) {
				if(m_textBuf_len == 0) MessageBoxA(NULL, "Type a valid subject ID", "Error", MB_OK | MB_ICONERROR);
				else {
					memcpy((void*)m_subjectID, (void*)m_txtBuf, sizeof(char)*m_textBuf_len);
					m_subjectID[m_textBuf_len] = NULL;
					if (strcmp(m_txtBuf, "0000") == 0) {
						printf("Test subject\n");
						m_testSubject = true;
						moveToNextPhase();
					}
					else {
						printf("Subject ID: %s\n", m_subjectID);
						char directory_path[128];
						bool directory_check = true;
						sprintf(directory_path, "./z_%s", m_subjectID);
						if (!check_directory(directory_path)) directory_check = false;
						if (!directory_check) {
							MessageBoxA(NULL, "Failed to locate the logging folder", "Error", MB_OK | MB_ICONERROR);
							m_exp_phase = EXP_PHASE::QUIT_EXP;
							return;
						}
						else {
							sprintf(m_rec_filename, "%s/%s_%02d_%02d.txt", directory_path, m_subjectID, (int)(m_ref_size), (int)(m_comp_size));	// subject ID_cutaneous stiffness_kinesthetic stiffness.txt
						}
						m_textBuf_len = 0;
						moveToNextPhase();
					}
				}
			}
			else if(key >= GLFW_KEY_0 && key <= GLFW_KEY_9){
				m_txtBuf[m_textBuf_len++] = '0'+ key - GLFW_KEY_0;
				m_txtBuf[m_textBuf_len] = NULL;
			}
			else if(key >= GLFW_KEY_A && key <= GLFW_KEY_Z) {
				m_txtBuf[m_textBuf_len++] = 'A'+ key - GLFW_KEY_A;
				m_txtBuf[m_textBuf_len] = NULL;
			}
			else MessageBoxA(NULL, "Type a valid subject ID", "Error", MB_OK | MB_ICONERROR);
		}
	}
	else if(m_exp_phase == EXP_PHASE::TRAINING_PHASE) {
		if(action == GLFW_PRESS) {
			if(key == GLFW_KEY_E) {
				moveToNextPhase();
			}
			else if(key == GLFW_KEY_1) {
				m_sphr_r = m_ref_size;
				m_sphr_pos[1] = m_top_pos-m_sphr_r;
			}
			else if(key == GLFW_KEY_2) {
				m_sphr_r = m_comp_size;
				m_sphr_pos[1] = m_top_pos-m_sphr_r;
			}
		}
	}
	else if(m_exp_phase == EXP_PHASE::EXP_PHASE2) {
		if(action == GLFW_PRESS && key == GLFW_KEY_ENTER) {
			moveToNextPhase();
		}
	}
	else if(m_exp_phase == EXP_PHASE::EXP_ANSWER) {
		if(action == GLFW_PRESS) {
			if(key == GLFW_KEY_1) {
				m_curr_answer = 0;
				moveToNextPhase();
			}
			else if(key == GLFW_KEY_2) {
				m_curr_answer = 1;
				moveToNextPhase();
			}
			else MessageBoxA(NULL, "Type a valid answer (1 or 2)", "Error", MB_OK | MB_ICONERROR);
		}
	}
	else if(m_exp_phase == EXP_PHASE::EXP_ANSWER_CORRECTNESS) {
		if(action == GLFW_PRESS && key == GLFW_KEY_ENTER) moveToNextPhase();
	}
	//// actuator debugging
/*	if(key == GLFW_KEY_1 && action == GLFW_PRESS) {
		if(!m_dbg_flag[0]) {
			m_dbg_flag[0] = true;
			printf("1\n");
		}
		else m_dbg_flag[0] = false;
	}
	if(key == GLFW_KEY_2 && action == GLFW_PRESS) {
		if(!m_dbg_flag[1]) {
			m_dbg_flag[1] = true;
			printf("2\n");
		}
		else m_dbg_flag[1] = false;
	}
	if(key == GLFW_KEY_3 && action == GLFW_PRESS) {
		if(!m_dbg_flag[2]) {
			m_dbg_flag[2] = true;
			printf("3\n");
		}
		else m_dbg_flag[2] = false;
	}
	if(key == GLFW_KEY_4 && action == GLFW_PRESS) {
		if(!m_dbg_flag[3]) {
			m_dbg_flag[3] = true;
			printf("4\n");
		}
		else m_dbg_flag[3] = false;
	}
	if(key == GLFW_KEY_5 && action == GLFW_PRESS) {
		if(!m_dbg_flag[4]) {
			m_dbg_flag[4] = true;
			printf("5\n");
		}
		else m_dbg_flag[4] = false;
	}
	if(key == GLFW_KEY_6 && action == GLFW_PRESS) {
		if(!m_dbg_flag[5]) {
			m_dbg_flag[5] = true;
			printf("6\n");
		}
		else m_dbg_flag[5] = false;
	}
	if(key == GLFW_KEY_7 && action == GLFW_PRESS) {
		if(!m_dbg_flag[6]) {
			m_dbg_flag[6] = true;
			printf("7\n");
		}
		else m_dbg_flag[6] = false;
	}
	if(key == GLFW_KEY_8 && action == GLFW_PRESS) {
		if(!m_dbg_flag[7]) {
			m_dbg_flag[7] = true;
			printf("8\n");
		}
		else m_dbg_flag[7] = false;
	}
	if(key == GLFW_KEY_9 && action == GLFW_PRESS) {
		if(!m_dbg_flag[8]) {
			m_dbg_flag[8] = true;
			printf("9\n");
		}
		else m_dbg_flag[8] = false;
	}
	if(key == GLFW_KEY_0 && action == GLFW_PRESS) {
		if(!m_dbg_flag[9]) {
			m_dbg_flag[9] = true;
			printf("0\n");
		}
		else m_dbg_flag[9] = false;
	}*/
	////
}

void display(GLFWwindow *pWindow, int viewportWidth, int viewportHeight)
{
	const float material[4] = {1.0f, 0.0f, 0.0f, 0.0f};
//	const float material[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	const float sphr_material[4] = {0.5, 0.5, 1.0, 0.0f};
	const float sphr_coll_material[4] = {1, 0, 0, 0};
	const float fingertip_material[4] = {0, 1, 0, 1};
	int i;
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
				glTranslatef(m_r_ft_pos.x, m_r_ft_pos.y, m_r_ft_pos.z);
				gluSphere(m_pQuadric, m_ft_r, 20, 20);
			glPopMatrix();
			if(!m_prev_coll[1]) glColor3f(0, 1, 0);
			else glColor3f(1, 0, 0);
			glPushMatrix();
			//	glEnable(GL_LIGHTING);
			    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, fingertip_material);
				glTranslatef(m_r_th_ft_pos.x, m_r_th_ft_pos.y, m_r_th_ft_pos.z);
				gluSphere(m_pQuadric, m_ft_r, 20, 20);
			glPopMatrix();
			/// actuator position drawing
			//glColor3f(0, 0, 1);
			//for(i=0;i<10;i++) {
			//	glPushMatrix();
			//		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, fingertip_material);
			//		glTranslatef(m_act_pt[i].x, m_act_pt[i].y, m_act_pt[i].z);
			//		gluSphere(m_pQuadric, m_ft_r*0.3, 20, 20);
			//	glPopMatrix();
			//}
		}
		if(m_show_sphr_top) {
			glColor3f(1, 0, 0);
			glPushMatrix();
			//	glEnable(GL_LIGHTING);
			    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, sphr_coll_material);
				glTranslatef(m_sphr_top_pos[0], m_sphr_top_pos[1], m_sphr_top_pos[2]);
				gluSphere(m_pTopQuad, 10, 20, 20);
			glPopMatrix();
		}
	 }
	if(m_exp_phase == EXP_PHASE::TRAINING_PHASE) drawGroundPlane();

	if(m_show_hand) {
		 glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, material);
		 glColor3f(0.9, 0.6, 0.7);
		 m_pDrawSkin->renderSkin();
	}
    glFlush();
	glfwSwapBuffers(pWindow);
}
//// 826 related routines
#define X826(FUNC)	if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode; }
// Counter mode: count down at 1MHz, preload upon start or when 0 reached, assert ExtOut when not 0
#define TMR_MODE  (S826_CM_K_1MHZ | S826_CM_UD_REVERSE | S826_CM_PX_ZERO | S826_CM_PX_START | S826_CM_OM_NOTZERO)
void init_s826()
{
	int errcode = S826_ERR_OK;
	int boardflags = 0;
	int ret, i, j;
	boardflags = S826_SystemOpen();
	if (boardflags < 0)
	{
		ret = boardflags;
		Sleep(500);
		exit(-1);
	}
	else {
		m_board[0] = 0;//(int)(boardflags & 1);
		m_board[1] = 1;//(int)(boardflags & 2);
		printf("board no: %d, %d\n", m_board[0], m_board[1]);
	}
	for(i=0;i<4;i++) {
		errcode = S826_DacRangeWrite(m_board[0], i, S826_DAC_SPAN_10_10, 0);
		errcode = S826_DacRangeWrite(m_board[1], i, S826_DAC_SPAN_10_10, 0);
	}
}


int periodicTimerStart(int board, uint counter, uint period)
{
	int errcode;
	X826(S826_CounterStateWrite(board, counter, 0)			);	// halt channel if it is running
	X826(S826_CounterModeWrite(board, counter, TMR_MODE)	);	// configure counter as a periodic timer
	X826(S826_CounterSnapshotConfigWrite(board, counter, 4, S826_BITWRITE));	// capture snapshots at zero counts
	X826(S826_CounterPreloadWrite(board, counter, 0, period));	// timer period in microseconds
	X826(S826_CounterStateWrite(board, counter, 1)			);	// start timer
	return errcode;
}
int periodicTimerStop(int board, uint counter)
{
	return S826_CounterStateWrite(board, counter, 0);
}

int periodicTimerWait(int board, uint counter, uint *timestamp)
{
	uint counts, tstamp, reason;		// counter snapsho
	int errcode = S826_CounterSnapshotRead(board, counter, &counts, &tstamp, &reason, S826_WAIT_INFINITE);	// wait for timer snapshot
	if(timestamp != NULL)
		*timestamp = tstamp;
	return errcode;
}

void getErrorString(int errcode, char* ret_string, int board_no)
{
	switch(errcode) {
    case S826_ERR_OK:    
		break;
    case S826_ERR_BOARD:
		sprintf(ret_string, "Illegal board number");
		break;
    case S826_ERR_VALUE:
		sprintf(ret_string, "Illegal argument");
		break;
    case S826_ERR_NOTREADY:
		sprintf(ret_string, "Device not ready or timeout");
		break;
    case S826_ERR_CANCELLED:
		sprintf(ret_string, "Wait cancelled");
		break;
    case S826_ERR_DRIVER:
		sprintf(ret_string, "Driver call failed");
		break;
    case S826_ERR_MISSEDTRIG:
		sprintf(ret_string, "Missed adc trigger");
		break;
    case S826_ERR_DUPADDR:
		sprintf(ret_string, "Two boards have same number");
		S826_SafeWrenWrite(board_no, 0x02);
		break;
    case S826_ERR_BOARDCLOSED:
		sprintf(ret_string, "Board not open");
		break;
	case S826_ERR_CREATEMUTEX:
		sprintf(ret_string, "Can't create mutex");
		break;
    case S826_ERR_MEMORYMAP:
		sprintf(ret_string, "Can't map board");
		break;
    default:
		sprintf(ret_string, "Unknown error");
		break;
	}
}

int main(void)
{
//    DrawSkin drawSkin;

	m_pDrawSkin = new DrawSkin();
	m_pClient = new HandTrackingClient::Client();
	m_pClient->addHandTrackingListener(m_pDrawSkin);
//    HandTrackingClient::Client client;
//    client.addHandTrackingListener(m_pDrawSkin);
    std::pair<bool, std::string> result = m_pClient->connect();//client.connect();
    if (!result.first)
    {
        printf("Unable to connect to hand tracking server: %s\n", result.second.c_str());
        return -1;
    }
    // Initialize the library 
    if (!glfwInit())
        return -1;

    struct GLFWTerminator
    {
        ~GLFWTerminator() { glfwTerminate(); }
    } terminator;

    // Create a windowed mode window and its OpenGL context
    GLFWwindow* pWindow = 
        glfwCreateWindow( 800, 600, "Draw Skin Demo", NULL, NULL);
    if (pWindow == NULL)
        return EXIT_FAILURE;

	//if(FT_Init_FreeType(&m_ft)) {
	//	fprintf(stderr, "Could not init freetype library\n");
	//	return -1;
	//}
	//if(FT_New_Face(m_ft, "FreeSans.ttf", 0, &m_face)) {
	//	fprintf(stderr, "Could not open font\n");
	//	return 1;
	//}
	//FT_Set_Pixel_Sizes(m_face, 0, 48);  

	glfwSetKeyCallback(pWindow, key_callback);
	m_pQuadric = gluNewQuadric();			// Create A Pointer To The Quadric Object  
	gluQuadricTexture(m_pQuadric, GL_TRUE);		// Create Texture Coords   
	gluQuadricNormals(m_pQuadric, GLU_SMOOTH);	// Create Smooth Normals 
	////
	m_pSphrQuad = gluNewQuadric();		// Test sphere quadric
	gluQuadricTexture(m_pSphrQuad, GL_TRUE);		// Create Texture Coords   
	gluQuadricNormals(m_pSphrQuad, GLU_SMOOTH);	// Create Smooth Normals 
	////
	m_pTopQuad = gluNewQuadric();		// top position indicator's quaric
	gluQuadricTexture(m_pTopQuad, GL_TRUE);		// Create Texture Coords   
	gluQuadricNormals(m_pTopQuad, GLU_SMOOTH);	// Create Smooth Normals 
	init_s826();
	m_run_Athread = true;
	InitializeCriticalSection(&m_cs);
	_beginthreadex(NULL, 0, hapticRenderingThread, NULL, 0, NULL);
	_beginthreadex(NULL, 0, AnalogOutThread, NULL, 0, NULL);
	_beginthreadex(NULL, 0, audioThread, NULL, 0, NULL);
	printf("Type subject ID and hit 'Enter' (000 is a test ID which does not record the log.)\n");
    // Loop until the user closes the window
    while (!glfwWindowShouldClose(pWindow))
    {
        int viewportWidth, viewportHeight;
        glfwGetWindowSize(pWindow, &viewportWidth, &viewportHeight);

        // Render here
        glfwMakeContextCurrent(pWindow);
		display(pWindow, viewportWidth, viewportHeight);
  //    drawSkin.paint(viewportWidth, viewportHeight);

  //      glFlush();
  //      glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }
    m_pClient->stop();
	SetEvent(m_hAudioEvent);
	gluDeleteQuadric(m_pQuadric);
	gluDeleteQuadric(m_pSphrQuad);
	gluDeleteQuadric(m_pTopQuad);
	S826_SystemClose();
	DeleteCriticalSection(&m_cs);
    return 0;
}

int moveToNextPhase()
{
	if(m_exp_phase == EXP_PHASE::INFO_INPUT) {
		m_show_sphere = true;
		m_show_hand = true;
		m_enable_rendering = true;
		printf("------------------------------------------------\n");
		printf("%s\n", m_instruction_msg[EXP_PHASE::TRAINING_PHASE]);
		m_exp_phase = EXP_PHASE::TRAINING_PHASE;
	}
	else if(m_exp_phase == EXP_PHASE::TRAINING_PHASE) {
		printf("------------------------------------------------\n");
		/// initialize the experiment
		m_curr_trial_no = 1;
		m_show_sphere = false;
		m_enable_rendering = false;
		m_show_hand = false;
		m_show_fingertip = true;
		m_show_sphr_top = true;
		time(&m_expBeginTime);
		time(&m_trialBeginTime);
		m_curr_stimulus = rand() % 2;
		if(m_curr_stimulus == 0) {
			m_sphr_r = m_ref_size;
			m_n_ref++;
		}
		else {
			m_sphr_r = m_comp_size;
			m_n_comp++;
		}
		m_sphr_pos[1] = m_top_pos-m_sphr_r;
		////
		setAudioPhase(AUDIO_PHASE::PLAY);
		recordResult(RECORD_TYPE::REC_INIT);
		////
		m_exp_phase = EXP_PHASE::EXP_PHASE1;
		printf("Trial No.: 1\n");
		printf(" %s\n",  m_instruction_msg[EXP_PHASE::EXP_PHASE1]);
	}
	else if(m_exp_phase == EXP_PHASE::EXP_PHASE1) {
		m_show_fingertip = false;
		m_show_sphr_top = false;
		m_enable_rendering = true;	// haptic rendering enabled
		printf(" %s\n", m_instruction_msg[EXP_PHASE::EXP_PHASE2]);
		m_exp_phase = EXP_PHASE::EXP_PHASE2;
	}
	else if(m_exp_phase == EXP_PHASE::EXP_PHASE2) {
		m_enable_rendering = false;		// haptic reindering disabled
		time(&m_trialEndTime);
		printf(" %s\n", m_instruction_msg[EXP_PHASE::EXP_ANSWER]);
		setAudioPhase(AUDIO_PHASE::PAUSE); 
		m_exp_phase = EXP_PHASE::EXP_ANSWER;
	}
	else if(m_exp_phase == EXP_PHASE::EXP_ANSWER) {
		printf(" Your ansewr for Trial %d was %s. %s\n", m_curr_trial_no, (m_curr_answer==0)?"reference":"comparison", m_instruction_msg[EXP_PHASE::EXP_ANSWER_CORRECTNESS]);
		exp_result curr_result;
		curr_result.trial_no = m_curr_trial_no;
		curr_result.trial_stimulus = m_curr_stimulus;
		curr_result.trial_begin_time = m_trialBeginTime;
		curr_result.trial_end_time = m_trialEndTime;
		curr_result.trial_user_ans = m_curr_answer;
		if (m_curr_stimulus == 0 && m_curr_answer == 0) m_n_H++;
		if (m_curr_stimulus == 1 && m_curr_answer == 0) m_n_F++;
		if (m_curr_stimulus == m_curr_answer) m_correct_ans++;

		m_expResult.push_back(curr_result);
		recordResult(RECORD_TYPE::REC_TRIAL);
		m_exp_phase = EXP_PHASE::EXP_ANSWER_CORRECTNESS;
	}
	else if(m_exp_phase == EXP_PHASE::EXP_ANSWER_CORRECTNESS) {
		/// check if current trial is the last one
		if(m_curr_trial_no == m_tot_trial_number) {
			/////
			// data analysis for H & F
			double z_H, z_F;
			if (m_n_ref > m_n_H) m_H = (double)m_n_H / (double)m_n_ref;
			else m_H = 1.0 - 0.5 / (double)m_n_ref;		// Avoiding d' being infinity (MacMillan and Creelman)
			if (m_n_comp > m_n_F) m_F = (double)m_n_F / (double)m_n_comp;
			else m_F = 1.0 - 0.5 / (double)m_n_comp;
			z_H = invnormsdist(m_H); z_F = invnormsdist(m_F);
			m_d = z_H - z_F;	// sensitivity index
			m_c = -0.5*(z_H + z_F);	// criterion location for response bias
			/////
			recordResult(RECORD_TYPE::REC_END);
			setAudioPhase(AUDIO_PHASE::COMPLETE);
			printf("%s\n", m_instruction_msg[EXP_PHASE::EXP_DONE]);
			m_exp_phase = EXP_PHASE::EXP_DONE;
		}
		else {
			m_curr_trial_no++;
			m_show_sphere = false;
			m_enable_rendering = false;
			m_show_hand = false;
			m_show_fingertip = true;
			m_show_sphr_top = true;
			time(&m_trialBeginTime);
			m_curr_stimulus = rand() % 2;
			if(m_curr_stimulus == 0) {
				m_sphr_r = m_ref_size;
				m_n_ref++;
			}
			else {
				m_sphr_r = m_comp_size;
				m_n_comp++;
			}
			m_sphr_pos[1] = m_top_pos-m_sphr_r;
			printf("------------------------------------------------\n");
			printf("Trial No.: %d\n", m_curr_trial_no);
			printf(" %s\n",  m_instruction_msg[EXP_PHASE::EXP_PHASE1]);
			setAudioPhase(AUDIO_PHASE::PLAY);
			m_exp_phase = EXP_PHASE::EXP_PHASE1;
		}
	}

	return 0;
}

bool check_directory(char *directory_path) {
	bool ret = true;
	if (!CreateDirectoryA(directory_path, NULL)) {
		if (GetLastError() != ERROR_ALREADY_EXISTS) {
			ret = false;
		}
	}
	return ret;
}

void recordResult(int type)
{
	FILE *pFile;
	time_t curr_time, tot_time;
	tm time_tm, time_tm2;
	errno_t err;
	if (type == RECORD_TYPE::REC_INIT) {
		err = _localtime64_s(&time_tm, &m_expBeginTime);//	err = _localtime64_s(&time_tm, &curr_time);
		printf("Subject: %s\n", m_subjectID);
		printf("Reference: %.1fmm Comparison: %.1fcm\n", m_ref_size, m_comp_size);
		printf("Experiment condition: %s", m_exp_cond==0?"lateral(4)+normal(1,bottom)":"lateral(4) only"); //0: lateral(4)+normal(1, bottom); 1: lateral(4) 
		printf("Experiment began at %02d:%02d:%02d on %04d/%02d/%02d\n", time_tm.tm_hour, time_tm.tm_min, time_tm.tm_sec,
			time_tm.tm_year + 1900, time_tm.tm_mon + 1, time_tm.tm_mday);
		//printf("-------------------------------------------------------------------------\n");
		//printf("Trial no.\tstimuli(0:ref/1:comp)\tanswer(0:ref/1:comp)\ttrial time (mm:ss)\tcoll_depth(1~7)mm\n");
		//printf("-------------------------------------------------------------------------\n");
		pFile = fopen(m_rec_filename, "a");
		if (pFile != NULL && !m_testSubject) {
			fprintf(pFile, "Subject: %s\n", m_subjectID);
			fprintf(pFile, "Reference: %.1fmm Comparison: %.1fcm\n", m_ref_size, m_comp_size);
			fprintf(pFile, "Experiment condition: %s", m_exp_cond==0?"lateral(4)+normal(1,bottom)":"lateral(4) only"); //0: lateral(4)+normal(1, bottom); 1: lateral(4) 
			fprintf(pFile, "Experiment began at %02d:%02d:%02d on %04d/%02d/%02d\n", time_tm.tm_hour, time_tm.tm_min, time_tm.tm_sec,
				time_tm.tm_year + 1900, time_tm.tm_mon + 1, time_tm.tm_mday);
			fprintf(pFile, "-------------------------------------------------------------------------\n");
			fprintf(pFile, "Trial no.\tstimuli(0:ref/1:comp)\tanswer(0:ref/1:comp)\ttrial time (mm:ss)\tcoll_depth(1~7)mm\n");
			fprintf(pFile, "-------------------------------------------------------------------------\n");
			fclose(pFile);
		}
	}
	else if (type == RECORD_TYPE::REC_TRIAL) {
		exp_result last_result = m_expResult.back();
		tot_time = difftime(last_result.trial_end_time, last_result.trial_begin_time);
		err = _localtime64_s(&time_tm, &tot_time);
//		printf("%d\t%d\t%d\t%02d:%02d\n", last_result.trial_no, last_result.trial_stimulus, last_result.trial_user_ans, time_tm.tm_min, time_tm.tm_sec);
		pFile = fopen(m_rec_filename, "a");
		if (pFile != NULL && !m_testSubject) {
			fprintf(pFile, "%d\t%d\t%d\t%02d:%02d\n", last_result.trial_no, last_result.trial_stimulus, last_result.trial_user_ans, time_tm.tm_min, time_tm.tm_sec);
			fclose(pFile);
		}
	}
	else {
		time(&curr_time);
		err = _localtime64_s(&time_tm, &curr_time);
		tot_time = difftime(curr_time, m_expBeginTime);
		err = _localtime64_s(&time_tm2, &tot_time);
		/////
		printf("===================================\n");
		printf("Experiment ended at %02d:%02d:%02d on %04d/%02d/%02d\n", time_tm.tm_hour,
			time_tm.tm_min, time_tm.tm_sec, time_tm.tm_year + 1900, time_tm.tm_mon + 1, time_tm.tm_mday);
		printf("Total experiment time: %02d:%02d\n", time_tm2.tm_min, time_tm2.tm_sec);
		printf("===================================\n");
		printf("H: %.1f F: %.1f\n", 100*m_H, 100*m_F);
		printf("d': %.3f c: %.3f \n", m_d, m_c);
		printf("percent correct: %.1f %%\n", 100.0*(float)m_correct_ans/(float)m_tot_trial_number);
		printf("===================================\n");
		pFile = fopen(m_rec_filename, "a");
		if (pFile != NULL && !m_testSubject) {
			fprintf(pFile, "===================================\n");
			fprintf(pFile, "Experiment ended at %02d:%02d:%02d on %04d/%02d/%02d\n", time_tm.tm_hour,
				time_tm.tm_min, time_tm.tm_sec, time_tm.tm_year + 1900, time_tm.tm_mon + 1, time_tm.tm_mday);
			fprintf(pFile, "Total experiment time: %02d:%02d\n", time_tm2.tm_min, time_tm2.tm_sec);
			fprintf(pFile, "===================================\n");
			fprintf(pFile, "H: %.1f F: %.1f\n", 100 * m_H, 100 * m_F);
			fprintf(pFile, "d': %.3f c: %.3f \n", m_d, m_c);
			fprintf(pFile, "percent correct: %.1f %%\n", 100*(float)m_correct_ans / (float)m_tot_trial_number);
			fprintf(pFile, "===================================\n");
			fclose(pFile);
		}
	}
}

void setAudioPhase(int audio_phase) 
{
	m_audio_phase = audio_phase;
	SetEvent(m_hAudioEvent);
}

void Unit(glm::vec3 input_vector, glm::vec3 &normalized_vector){
	float input_norm;
	input_norm = sqrt(input_vector.x*input_vector.x+input_vector.y*input_vector.y+input_vector.z*input_vector.z);
	normalized_vector.x = input_vector.x / input_norm;
	normalized_vector.y = input_vector.y / input_norm;
	normalized_vector.z = input_vector.z / input_norm;
}
void DrawArrow(glm::vec3 p0, glm::vec3 direction, double length)//, double arrow_rad)
{
	glm::vec3 norm_direction, end_pt;
	Unit(direction, norm_direction);
	end_pt = glm::vec3(p0.x+length*norm_direction.x, p0.y+length*norm_direction.y, p0.z+length*norm_direction.z);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3f(p0.x, p0.y, p0.z);
	glVertex3f(end_pt.x, end_pt.y, end_pt.z);
	glEnd();
}

glm::vec3 vec_norm3(glm::vec3 vec)
{
	glm::vec3 ret;
	float dist, dist2;
	dist2 = vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2];
	if(dist2 == 0.0) ret= glm::vec3(0, 0, 0);
	else {
		dist = sqrt(dist2);
		ret[0] = vec[0]/dist;
		ret[1] = vec[1]/dist;
		ret[2] = vec[2]/dist;
	}
	return ret;
}

#define PI  3.1415926535
unsigned __stdcall AnalogOutThread(void* arg)
{
	/// initialize Aout
	int errcode, i, j;
	uint counter = 0;	// Use this counter channel as DAC sample timer
	uint t_sample = 1000;	// sample time in microseconds. Must be fast enough to meet Nyquist. 
	uint tstamp;	// timestamp in microseconds
	uint tbegin;	// sampling begin time in microseconds
	uint DAC_val;
	double runtime;
	float amplitude;
	float piezo_env[2][5]= {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
	uint offset;
	amplitude = (uint)(32268.0*1.8/10.0);
	offset = (uint)(32268.0 + 32268.0*2.5/10.0);
	errcode = periodicTimerStart(m_board[0], counter, t_sample);
	errcode = periodicTimerWait(m_board[0], counter, &tstamp);
	tbegin = tstamp;
	while(m_run_Athread) {
		runtime = (double) (tstamp - tbegin) / 1000000.0;	// compute time elapsed since t0 in seconds
		EnterCriticalSection(&m_cs);
		for(i=0;i<2;i++) {
			for(j=0;j<4;j++) piezo_env[i][j] = m_piezo_env[i][j];
		}
		LeaveCriticalSection(&m_cs);
		if(m_enable_rendering) {
			for(i=0;i<2;i++){
				if(m_prev_coll[i]) {
					for(j=0;j<4;j++) {
						amplitude = (uint)(32268.0*2.3*piezo_env[i][j]/10.0);
						DAC_val = (uint)(offset+amplitude*sin(2.0*PI*250*runtime));
				//	printf("%d ", DAC_val);
						S826_DacDataWrite(m_board[i], j, DAC_val, 0);
					}
				}
				else {

					for(j=0;j<4;j++) S826_DacDataWrite(m_board[i], j, 32268, 0);
				}
			}
		}
		else {
			/// keyboard debugging
		/*	amplitude = (uint)(32268.0*0.4);
			DAC_val = (uint)(offset+amplitude*sin(2.0*PI*250*runtime));
			for(i=0;i<5;i++) {
				if(m_dbg_flag[i]) {
					S826_DacDataWrite(m_board[0], i, DAC_val, 0);
				//	printf("bd(%d), %d %d\n", m_board[0], i, DAC_val);
				}
				else S826_DacDataWrite(m_board[0], i, 32268, 0);
			}
			for(i=0;i<5;i++) {
				if(m_dbg_flag[i+5]) {
					S826_DacDataWrite(m_board[1], i, DAC_val, 0);
				//	printf("bd(%d), %d %d\n", m_board[1], i, DAC_val);
				}
				else S826_DacDataWrite(m_board[1], i, 32268, 0);
			}
			*/
			for(i=0;i<2;i++)
				for(j=0;j<4;j++) S826_DacDataWrite(m_board[i], j, 32268, 0);
		}
		errcode = periodicTimerWait(m_board[0], counter, &tstamp);
	}
	return 0;
}
/*
 actuator:analog output ch
------------------------------
  B:0
  RR:1
  RF:2
  LR:3
  LF:4
*/

unsigned __stdcall hapticRenderingThread(void* arg)
{
	bool curr_coll = false;//{false, false};
	glm::vec3 coll_vec[2], coll_vec2[2], coll_s_vec;
	glm::vec3 sphr_x, sphr_y, sphr_z, ft_o[2], contact_pt[2];
	glm::vec3 avec[5];//bt, rr,rf, lr, lf 
//	glm::vec3 sphr_x, sphr_y, sphr_z, ft_o[2];
	float d_c2c[2], coll_depth = 0.0;//[2] = {0.0, 0.0};
	float ac1_val, ac2_val;
	float d_close = 10+m_ft_r+3;;
	bool en_act[5], en_vib;
	float alph, beta, x_val, y_val;	// for condition 1
	int i, j, k, i_off, idx;
	float dist[3], wt[3], inv_d_sum, env_val[5], coll_depth_r;
	uint a_idx[3];
	float th;
	do {

		if(m_exp_phase== EXP_PHASE::TRAINING_PHASE || m_exp_phase == EXP_PHASE::EXP_PHASE1 || m_exp_phase == EXP_PHASE::EXP_PHASE2) {
			if(m_exp_phase == EXP_PHASE::EXP_PHASE1) {
				float chk_dist2, chk_dist22;
				chk_dist2 = (m_r_ft_pos.x - m_sphr_top_pos[0])*(m_r_ft_pos.x - m_sphr_top_pos[0]) + (m_r_ft_pos.y - m_sphr_top_pos[1])* (m_r_ft_pos.y - m_sphr_top_pos[1])
					+ (m_r_ft_pos.z - m_sphr_top_pos[2])* (m_r_ft_pos.z - m_sphr_top_pos[2]);
				chk_dist22 = (m_r_th_ft_pos.x - m_sphr_top_pos[0])*(m_r_th_ft_pos.x - m_sphr_top_pos[0]) + (m_r_th_ft_pos.y - m_sphr_top_pos[1])* (m_r_th_ft_pos.y - m_sphr_top_pos[1])
					+ (m_r_th_ft_pos.z - m_sphr_top_pos[2])* (m_r_th_ft_pos.z - m_sphr_top_pos[2]);
				if(chk_dist2 <= d_close*d_close || chk_dist22 <= d_close*d_close) moveToNextPhase();
			//	printf("%f (%f %f %f)(%f, %f, %f)\n", chk_dist2, m_r_ft_pos.x, m_r_ft_pos.y, m_r_ft_pos.z, m_sphr_top_pos[0], m_sphr_top_pos[1], m_sphr_top_pos[2]);
			}
			coll_vec[0] = glm::vec3(m_r_ft_pos.x - m_sphr_pos[0], m_r_ft_pos.y - m_sphr_pos[1], m_r_ft_pos.z - m_sphr_pos[2]);
			coll_vec[1] = glm::vec3(m_r_th_ft_pos.x - m_sphr_pos[0], m_r_th_ft_pos.y - m_sphr_pos[1], m_r_th_ft_pos.z - m_sphr_pos[2]);
			ft_o[0] = glm::vec3(m_r_ft_pos.x, m_r_ft_pos.y, m_r_ft_pos.z);
			ft_o[1] = glm::vec3(m_r_th_ft_pos.x, m_r_th_ft_pos.y, m_r_th_ft_pos.z);
			for(i=0;i<2;i++) {
				d_c2c[i] = sqrt(coll_vec[i][0]*coll_vec[i][0] + coll_vec[i][1]*coll_vec[i][1] + coll_vec[i][2]*coll_vec[i][2]);	// fingertip-center to sphere-center distance 
				coll_vec2[i] = -coll_vec[i];
				if(i==1) {
					sphr_x = vec_norm3(glm::vec3(m_r_th_jnt_rot[2].x, m_r_th_jnt_rot[2].y, m_r_th_jnt_rot[2].z));
					sphr_y = vec_norm3(glm::vec3(m_r_th_jnt_rot[0].x, m_r_th_jnt_rot[0].y, m_r_th_jnt_rot[0].z));
					sphr_z = vec_norm3(glm::vec3(m_r_th_jnt_rot[1].x, m_r_th_jnt_rot[1].y, m_r_th_jnt_rot[1].z));
				}
				else {
					sphr_x = vec_norm3(glm::vec3(m_r_jnt_rot[2].x, m_r_jnt_rot[2].y, m_r_jnt_rot[2].z));
					sphr_y = vec_norm3(glm::vec3(m_r_jnt_rot[0].x, m_r_jnt_rot[0].y, m_r_jnt_rot[0].z));
					sphr_z = vec_norm3(glm::vec3(m_r_jnt_rot[1].x, m_r_jnt_rot[1].y, m_r_jnt_rot[1].z));
				}
				avec[0] = vec_norm3(-sphr_z);
				avec[1] = vec_norm3((float)(0.707)*sphr_x-(float)(0.707)*sphr_y);	// rr
				avec[2] = vec_norm3((float)(0.707)*sphr_x+(float)(0.707)*sphr_y);	// rf
				avec[3] = vec_norm3(-(float)(0.707)*sphr_x-(float)(0.707)*sphr_y);	// lr
				avec[4] = vec_norm3(-(float)(0.707)*sphr_x+(float)(0.707)*sphr_y);	// lf
				if(d_c2c[i] <= (m_ft_r+m_sphr_r)) {	// collision
					a_idx[0] = 0;
					curr_coll = true;
					coll_depth = m_ft_r+m_sphr_r - d_c2c[i]; //coll_depth[i] = m_ft_r + m_sphr_r - d_c2c[i];
					coll_s_vec = vec_norm3(coll_vec2[i]);	// scaled to the fingertip radius
					/// rendering for exp condition 0
					ac1_val = glm::dot(avec[2], coll_s_vec);
					ac2_val = glm::dot(avec[4], coll_s_vec);
					if(ac1_val >=0) {	//rf
						if(ac2_val >= 0) {	// rf lf true
							a_idx[1] = 2;
							a_idx[2] = 4;
						}
						else {		//  rf lr true
							a_idx[1] = 2;
							a_idx[2] = 3;
						}
					}
					else {		// rr true
						if(ac2_val >=0) {		// rr lf true
							a_idx[1] = 1;
							a_idx[2] = 4;
						}
						else {					// rr lr true
							a_idx[1] = 1;
							a_idx[2] = 3;
						}
					}
					inv_d_sum = 0.0;
					for(j=0;j<3;j++) {
						dist[j] = acos(glm::dot(coll_s_vec, avec[a_idx[j]]));
						if(dist[j] != 0.0) inv_d_sum += 1/dist[j];
					}
					for(j=0;j<3;j++) {
						if(dist[j] == 0.0) wt[j] = 0.0;
						else wt[j] = sqrt((float)(1/(dist[j]*inv_d_sum)));	// envelope intensity
					}
					///////////////	rendering for experiment condition 1
					x_val = glm::dot(sphr_x, coll_s_vec);
					y_val = glm::dot(sphr_y, coll_s_vec);
					alph = (x_val+1)/2;
					beta = (y_val+1)/2;
					//////////// depth rendering
					if(coll_depth >= m_ft_r) coll_depth_r = 1.0;
					else coll_depth_r = 0.8*coll_depth/m_ft_r+0.2;
					/////////////////////////////////////////////

					if(m_exp_cond == 0)
					{
						for(j=0;j<5;j++) {
							en_vib = false;
							idx = -1;
							for(k=0;k<3;k++) {
								if(j==a_idx[k]) {
									en_vib = true;
									idx = k;
								}
							}
							if(en_vib) env_val[j] = coll_depth_r*wt[idx];
							else env_val[j] = 0.0;
						}
					}
					else {
						env_val[0] = 0.0;
						env_val[1] = coll_depth_r*alph*(1-beta);	// RR
						env_val[2] = coll_depth_r*alph*beta;	// RF
						env_val[3] = coll_depth_r*(1-alph)*(1-beta);	// LR
						env_val[4] = coll_depth_r*(1-alph)*beta;	// LF
					}
					//////
					EnterCriticalSection(&m_cs);
						for(j=0;j<4;j++) m_piezo_env[i][j] = env_val[j];
					LeaveCriticalSection(&m_cs);
					///////////////
				}
				else {	/// no collision
					curr_coll = false;
				}
				m_prev_coll[i] = curr_coll;
				m_coll_depth[i] = coll_depth;
				if(i==0) i_off = 0;
				else i_off = 5;
				for(j=0;j<5;j++) m_act_pt[j+i_off] = m_ft_r*avec[j]+ft_o[i];

			}	// for(i=0;i<2;i++)
		}
		else m_enable_rendering = false;
		Sleep(20);	// 50 Hz update
	} while(m_run_haptic);
	return 0;
}

unsigned __stdcall audioThread(void* arg)
{
//	cExp_Size_Perception *pExp = (cExp_Size_Perception*)arg;
	cMCI_sound fanfare("Fanfare.wav"), noise("WhiteNoise_15min.mp3");
	int curr_audio_phase, prev_audio_phase;
	prev_audio_phase = AUDIO_PHASE::AUDIO_INIT;
	curr_audio_phase = m_audio_phase;
	do {
		curr_audio_phase = m_audio_phase;
		if (prev_audio_phase != curr_audio_phase) {
			switch (curr_audio_phase) {
			case AUDIO_PHASE::PLAY:
				noise.play(15000);
				break;
			case AUDIO_PHASE::PAUSE:
				noise.pause();
				break;
			case AUDIO_PHASE::STOP:
				noise.stop();
				break;
			case AUDIO_PHASE::COMPLETE:
				noise.stop();
				fanfare.play();
				break;
			}
			prev_audio_phase = curr_audio_phase;
			WaitForSingleObject(m_hAudioEvent, INFINITE);
		}
	} while (m_exp_phase != EXP_PHASE::QUIT_EXP);
	return 0;
}
