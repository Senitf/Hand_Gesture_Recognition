#pragma once

#include <string>
#include <vector>
#include <array>
#include <iosfwd>
#include <cmath>
#include "HandTrackingClient/VecMath.h"
#include "HandTrackingClient/OpenCVCamera.h"

/**
 * @namespace HandTrackingClient
 * The HandTrackingClient has been namespaced
 * to avoid conflicts with other libraries.
 */

//! \file HandTrackingMessage.h
//!
//! \brief Messages parsed out by the hand tracking client.
//!
//! These are the messages that will get parsed out by the hand tracking
//! client.  You should respond to the relevant ones and ignore the ones
//! you don't need.  In particular, if you are using the raw pose layer
//! (i.e., \ref HandTrackingClient::PoseMessage), you should consider ignoring all gesture-related
//! messages (e.g. \ref HandTrackingClient::PointMessage, etc.).

namespace HandTrackingClient
{

//! The number of joints in the hand model, currently 17 (arm, wrist, and 3 each for the 5 fingers).
const static int N_JOINTS = 17;

//! The number of fingers is five (thumb, index, middle, ring, pinky).  
const static int N_FINGERS = 5;

//! We currently only support tracking two hands (left and right).
const static int N_HANDS = 2;

//! \brief The number of specifically recognized poses.  
//!
//! For a complete list, see \ref PoseMessage::HandPose
const static int N_POSES = 7;

//! The number of finger degrees of freedom per hand.
const static int N_FINGER_DOFS_PER_HAND = 16;

//! \brief Joint frames used for skinning.
//!
//! These are the frames returned by \ref PoseMessage::getJointFrames,
//! and used for skinning.  We recommend using these if you need to locate points in 3D
//! space or define 3D frames; if you want to recognize gestures of the form "finger X is
//! bending" we recommend using the \ref FingerDOF instead.
//!
//! The most stable frame is the one defined by the metacarpals
//! (here, the \ref WRIST_JOINT).  If you need a 3D frame for something, we recommend
//! using this one.  
//!
//! For more details on the hand model, consult the 
//! <a href="../handModel.html">hand model docs</a>. 
enum JointFrameIndex
{
    ROOT_JOINT = 0,           ///< The frame of the user's forearm (this is where the hand model is rooted).
    WRIST_JOINT = 1,          ///< The frame of the back of the hand (anatomically, this is the frame of the carpals).  
    THUMB_PROXIMAL = 2,       ///< Thumb proximal frame, refers to the thumb metacarpal bone.  
    THUMB_INTERMEDIATE = 3,   ///< Thumb intermediate frame, refers to the thumb proximal phalange.
    THUMB_DISTAL = 4,         ///< Thumb distal frame, refers to the thumb distal phalange.
    INDEX_PROXIMAL = 5,       ///< Index finger proximal frame, refers to the proximal phalange.
    INDEX_INTERMEDIATE = 6,   ///< Index finger intermediate frame, refers to the intermediate phalange.
    INDEX_DISTAL = 7,         ///< Index finger distal frame, refers to the distal phalange.
    MIDDLE_PROXIMAL = 8,      ///< Middle finger proximal frame, refers to the proximal phalange.
    MIDDLE_INTERMEDIATE = 9,  ///< Middle finger intermediate frame, refers to the intermediate phalange.
    MIDDLE_DISTAL = 10,       ///< Middle finger distal frame, refers to the distal phalange.
    RING_PROXIMAL = 11,       ///< Ring finger proximal frame, refers to the proximal phalange.
    RING_INTERMEDIATE = 12,   ///< Ring finger intermediate frame, refers to the intermediate phalange.
    RING_DISTAL = 13,         ///< Ring finger distal frame, refers to the distal phalange.
    PINKY_PROXIMAL = 14,      ///< Pinky finger proximal frame, refers to the proximal phalange.
    PINKY_INTERMEDIATE = 15,  ///< Pinky finger intermediate frame, refers to the intermediate phalange.
    PINKY_DISTAL = 16,        ///< Pinky finger distal frame, refers to the distal phalange.
};

//! Included for backwards compatibility.
typedef JointFrameIndex JointIndex;

//! \brief Degrees of freedom of the hand model, as reported by the 
//!        \ref HandTrackingClient::PoseMessage::getFingerDOFs function.
//!
//! Each of these is a rotation in radians that measures how far
//! the corresponding finger is bent from its rest pose.  For 
//! flexion/extension joints, positive angles indicate flexion
//! while negative angles indicate extension.  
//!
//! We recommend using these joint angles for detecting gestures of 
//! the form "finger X is bending."  If you need to know the global
//! position/orientation of the hand, you'll need to use the joint
//! frames returned by HandTrackingClient::PoseMessage::getJointFrames
//! instead.
//! 
//! For more details on the hand model, consult the 
//! <a href="../handModel.html">hand model docs</a>. 
enum FingerDOF
{
    THUMB_CMC_AA  = 0,     ///< Thumb carpal-metacarpal joint, adduction/abduction
    THUMB_CMC_FE  = 1,     ///< Thumb carpal-metacarpal joint, flexion/extension
    THUMB_MCP     = 2,     ///< Thumb metacarpal-phalangeal joint, flexion/extension
    THUMB_IP      = 3,     ///< Thumb interphalangeal joint, flexion/extension
    INDEX_MCP_AA  = 4,     ///< Index finger metacarpal-phalangeal joint, adduction/abduction
    INDEX_MCP_FE  = 5,     ///< Index finger metacarpal-phalangeal joint, flexion/extension
    INDEX_PIP     = 6,     ///< Index finger proximal interphalangeal joint, flexion/extension
    MIDDLE_MCP_AA = 7,     ///< Middle finger metacarpal-phalangeal joint, adduction/abduction
    MIDDLE_MCP_FE = 8,     ///< Middle finger metacarpal-phalangeal joint, flexion/extension
    MIDDLE_PIP    = 9,     ///< Middle finger proximal interphalangeal joint, flexion/extension
    RING_MCP_AA   = 10,    ///< Ring finger metacarpal-phalangeal joint, adduction/abduction
    RING_MCP_FE   = 11,    ///< Ring finger metacarpal-phalangeal joint, flexion/extension
    RING_PIP      = 12,    ///< Ring finger proximal interphalangeal joint, flexion/extension
    PINKY_MCP_AA  = 13,    ///< Pinky finger metacarpal-phalangeal joint, adduction/abduction
    PINKY_MCP_FE  = 14,    ///< Pinky finger metacarpal-phalangeal joint, flexion/extension
    PINKY_PIP     = 15     ///< Pinky finger proximal interphalangeal joint, flexion/extension
};

//! \brief Structure holding the position, rotational frame and other information
//!        relevant to the state of the hand.
//!
//! This is the _overall_ position and rotation estimate for the hand.  
//! It is used to pass around location information for clicks.  Users
//! needing more detailed information about individual finger joints
//! should look at the \ref PoseMessage.  
class HandState
{
public:
    //! Constructs a hand state with the given position and rotation.  
    HandState(const Vector3f& position, 
              const Quaternionf& rotation, 
              const int clickCount) 
        : _position(position), 
          _rotation(rotation),
          _clickCount(clickCount) { }

    //! Constructs a hand state with position = (0,0,0) and the identity rotation.  
    HandState() { }

    //! \brief Position estimate for the hand.  
    //!
    //! This is defined by the rigid frame given by the back of
    //! the hand, so it will be considerably more stable than
    //! the individual finger position estimates given by the \ref PoseMessage.
    const Vector3f& getPosition() const { return _position; }

    //! Overall orientation estimate for the hand.  
    //!
    //! This is defined by the rigid frame given by the back of
    //! the hand, so it will be considerably more stable than
    //! the individual finger position estimates given by the \ref PoseMessage.
    const Quaternionf& getRotation() const { return _rotation; }

    //! Returns 1 if the hand is single-clicked, 2 if the hand is
    //! double-clicked, and 0 if neither (no clicks active).  
    const int getClickCount() const { return _clickCount; }

private:
    Vector3f _position;
    Quaternionf _rotation;
    int _clickCount;
};

//! Most events (like click events) only apply to
//! one hand at a time, but simultaneous click and
//! release events will have the BOTH_HANDS identifier set.  
enum Hand
{
    LEFT_HAND = 0,        ///< Message applies only to the left hand.
    RIGHT_HAND = 1,       ///< Message applies only to the right hand.
    BOTH_HANDS = 2,       ///< Message applies to both hands (e.g. a simultaneous click).  
    INVALID_HAND = 10000  ///< You should never see this in callback messages; it is used internally when network deserialization fails.  
};

//! Returns a string representation for the Hand enum.
//! Used for serializing and deserializing from the 
//! network.  
const char* handToString(Hand hand);

//! Given a string ("left" or "right"), returns the
//! correct Hand.  
//! Used for serializing and deserializing from the 
//! network.  
Hand stringToHand(const std::string& str);

//! Finger flicks are detected in the left and right directions
//! and hand swipes are detected in left, right, up or down
//! For use with front-facing mounted sensor only.
enum FlickDirection
{
    LEFT_DIRECTION = 0,         ///< Swipe left
    RIGHT_DIRECTION = 1,        ///< Swipe right
    UP_DIRECTION = 2,           ///< Swipe up
    DOWN_DIRECTION = 3,         ///< Swipe down
    INVALID_DIRECTION = 10000   ///< Invalid swipe direction; should never be encountered by clients.
};

//! Returns a string representation for the \ref FlickDirection enum.
//! Used for serializing and deserializing from the 
//! network.  
//! For use with front-facing mounted sensor only.
const char* flickFirectionToString(FlickDirection direction);

//! Given a string ("left" or "right"), returns the
//! correct \ref Hand.  
//! Used for serializing and deserializing from the 
//! network.  
//! For use with front-facing mounted sensor only.
FlickDirection stringToFlickDirection(const std::string& str);

//! Exception thrown when parsing a message fails.  
class ParseException
    : public std::exception
{
public:
    ParseException (const char* message)
        : _msg (message) {}
    virtual ~ParseException() throw() {}
    
    virtual const char* what() const throw() { return _msg; }

private:
    const char* _msg;
};

///
/// Base class for all hand-tracking messages / events. 
///
class HandTrackingMessage
{
public:
    //! An enum describing the message type.  
    //! Used for distinguishing between similar messages, e.g., between
    //! PRESSED and RELEASED, which are both PinchMessages with a top-down
    //! mounted sensor, or between
    //! TRIGGER_PRESSED and TRIGGER_RELEASED, which are both PinchMessages
    //! with a front-facing mounted sensor.
    enum MessageType  
    {
        WELCOME = 0,                    ///< Provides server and protocol versions. Type of WelcomeMessage.
        USER = 100,                     ///< Provides information about the user, such as profile name and skinning information. Type of \ref UserMessage.
        CALIBRATION = 101,              ///< User is in a spread / calibrating pose, provides info about scale and percent complete. Type of \ref CalibrationMessage.
        POSE = 200,                     ///< Provides full skeleton pose information. Type of \ref PoseMessage.
        
        PRESSED = 300,                  ///< User pinched index and thumb together. Type of \ref PinchMessage. Top-down mounted sensor only.
        DRAGGED = 301,                  ///< User moved hand while holding a pinch. Type of \ref PinchMessage. Top-down mounted sensor only.
        RELEASED = 302,                 ///< User released the pinch. Type of PinchMessage. Top-down mounted sensor only.
        
        MOVED = 303,                    ///< User moved without pinching. Type of PinchMessage.
        
        SIMULTANEOUSLY_PRESSED = 400,   ///< User pinched thumb and index finger of both hands at (approximately) the same time. Type of \ref BimanualPinchMessage. Top-down mounted sensor only.
        INDIVIDUALLY_PRESSED = 401,     ///< User pinched thumb and index finger of one hand (compare to SIMULTANEOUSLY_PRESSED). Type of \ref BimanualPinchMessage. Top-down mounted sensor only.
        SIMULTANEOUSLY_RELEASED = 402,  ///< User released thumb and index finger of both hands at the same time. Type of \ref BimanualPinchMessage. Top-down mounted sensor only.
        INDIVIDUALLY_RELEASED = 403,    ///< User released thumb and index finger of one hand (compare to SIMULTANEOUSLY_RELEASED). Type of \ref BimanualPinchMessage. Top-down mounted sensor only.
        DRAGGED_BIMANUAL = 404,         ///< User moved hands while both were pinching. Type of \ref BimanualPinchMessage. Top-down mounted sensor only.
        
        POINT = 500,                    ///< User pointed at something with the index finger. Type of \ref PointMessage.
        
        TRIGGER_PRESSED = 501,          ///< User bent their index finger into a trigger pose. Type of \ref PinchMessage. Front-facing mounted sensor only.
        TRIGGER_RELEASED = 502,         ///< User ended the trigger pose. Type of \ref PinchMessage. Front-facing mounted sensor only.
        PEACE_BEGAN = 503,              ///< User extended the middle and index fingers (ignoring thumb) to make a peace sign. Type of \ref PinchMessage. Front-facing mounted sensor only.
        PEACE_ENDED = 504,              ///< User ended the peace sign (ignoring thumb). Type of \ref PinchMessage. Front-facing mounted sensor only.
        FINGER_FLICK = 505,             ///< User extended the middle and index fingers (ignoring thumb) and made a quick left or right motion. Type of \ref FlickMessage. Front-facing mounted sensor only.
        HAND_SWIPE = 506,               ///< User extended all five fingers, moved the hand left or right, then retracted the fingers. Type of \ref FlickMessage. Front-facing mounted sensor only.
        
        INVALID_DATA = 10000            ///< This should never happen; in practice it will be ignored
    };

    //! Convert from a MessageType to its string representation; used for serialization.  
    static const char* messageTypeToString(MessageType m);

    //! Convert from a string to a MessageType; used in deserialization.  
    static MessageType stringToMessageType(const std::string& str);

    //! \brief Given a line of text sent over the network connection, 
    //!        converts it to a valid hand message.
    //!
    //! Callers are responsible for freeing the memory.  
    static HandTrackingMessage* deserialize(const std::string& data);

    //! Returns the message type (e.g. PRESSED or DRAGGED).  
    //! The message type is useful for distinguishing between similar messages, e.g., between
    //! PRESSED and RELEASED, which are both PinchMessages with a top-down mounted sensor, or between
    //! TRIGGER_PRESSED and TRIGGER_RELEASED, which are both PinchMessages with a front-facing mounted sensor.
    MessageType getType() const;

    //! \brief Serializes the message to send over the network.  
    //! 
    //! Developers should not need to worry about this function 
    //! unless they write their own networking code.  
    virtual std::string serialize() const = 0;

    virtual ~HandTrackingMessage();
protected:
    //! Constructor is protected; HandTrackingMessage should only
    //! be constructed from network data using the deserialize() function. 
    HandTrackingMessage(MessageType msgType);

private:
    // Make it non-copyable (in C++11 we'd use delete here).
    HandTrackingMessage& operator= (const HandTrackingMessage& rhs);
    HandTrackingMessage (const HandTrackingMessage& rhs);

    const MessageType _type;
};

//! The basic form of the message includes a type, which hand was updated,
//! and the position and rotational frame of each hand. 
class BasicMessage : public HandTrackingMessage
{
public:
    //! \brief Returns the overall general hand state, excluding fingers.
    //!
    //! This is the most rigid, stable frame returned by the hand tracking
    //! and is probably the one you want to use for selection.  It is defined by
    //! the position and rotation of the back of the hand (excluding the wrist).  
    const HandState& getHandState(int hand) const { return _hands[hand]; }

    //! Constructs a BasicMessage.  Should not generally be called by users of the API.  
    BasicMessage(const MessageType type, 
                 const Vector3f& positionLeft,
                 const Quaternionf& rotationLeft,
                 const int clickCountLeft,
                 const Vector3f& positionRight,
                 const Quaternionf& rotationRight,
                 const int clickCountRight);
    virtual ~BasicMessage();

    //! Serializes the message to a string, which could then be output
    //! over the network. 
    std::string serialize() const;

private:
    std::array<HandState, N_HANDS> _hands;
};

//! \brief Messages relating to pressing, releasing, dragging and moving of each hand with a top-down mounted sensor or
//!        messages relating to moving, triggering, and signing peace with each hand with a front-facing mounted sensor.
//!
//! Pinch messages are always a single hand at a time. That is, if the user pinches left and right simultaneously
//! with a top-down mounted sensor, you will still get two separate PRESS messages (one for the right and one for the left).
//! Similarly, if the user triggers left and right simultaneously with a front-facing mounted sensor, you will still
//! get two separate TRIGGER_PRESSED messages (one for the right and one for the left).
//!
//! As a result, with a top-down mounted sensor there is less lag for pinch messages (which can be reported as soon as they
//! happen) than for the "higher-level" \ref BimanualPinchMessage messages available only with a top-down mounted sensor.
//!
//! We do not recommend responding to both the PinchMessage
//! and the \ref BimanualPinchMessage; applications should generally
//! pick one or the other.  Responding to both can lead to very 
//! confusing interactions (since most pinches will be detected at least
//! twice).  
class PinchMessage : public BasicMessage
{
public:
    //! Constructs a PinchMessage.  Should not generally be called by users of the API.  
    PinchMessage(const MessageType type, 
                 const Hand hand,
                 const Vector3f& positionLeft,
                 const Quaternionf& rotationLeft,
                 const int clickCountLeft,
                 const Vector3f& positionRight,
                 const Quaternionf& rotationRight,
                 const int clickCountRight);

    //! @return which hand(s) this message applies to (can be LEFT or RIGHT, never BOTH).  
    const Hand getHand() const { return _hand; }

    std::string serialize() const;

private:
    Hand _hand;
};

//! \brief Messages relating to simultaneous or individual pinching with a top-down mounted sensor.
//!
//! Bimanual pinch messages are a ``higher-level'' message than the regular
//! pinch message, and can be used to distinguish whether one hand pinched
//! or whether both pinched simultaneously.  If the user presses both hands
//! at the same time, for example, you will only get a single Bimanual
//! SIMULTANEOUSLY_PRESSED message (instead of two PRESSED messages, as you
//! would with the PinchMessage). The cost
//! of this higher-level knowledge is a bit of additional lag: to determine
//! whether two pinches happen at the same time we have to wait 100ms or so 
//! (since the pinches are never going to be _exactly_ simultaneous).  You 
//! can decide whether the added functionality is worth the lag and choose which
//! of the two interfaces you prefer.  
//!
//! Note that we do not recommend responding to both the BimanualPinchMessage
//! and the \ref PinchMessage; applications should generally
//! pick one or the other.  Responding to both can lead to very 
//! confusing interactions (since most pinches will be detected at least
//! twice).  
//! 
//! To help understanding the difference between the PinchMessage and the
//! BimanualPinchMessage, consider the following two scenarios:
//!
//! First, suppose the user presses her left hand and then, a second later,
//! her right hand.  You will receive the following sequence of events:
//!
//!    \li PinchMessage(PRESSED, left hand)
//!    \li ...
//!    \li BimanualPinchMessage(INDIVIDUALLY_PRESSED, left hand)
//!    \li ...
//!    \li PinchMessage(PRESSED, right hand)
//!    \li ...
//!    \li BimanualPinchMessage(INDIVIDUALLY_PRESSED, right hand)
//!
//! Note how the BimanualPinchMessage always lags slightly behind the 
//! PinchMessage (you will receive it a couple frames later).  
//! Now, suppose the user presses her left and right hands simultaneously.
//!
//!    \li PinchMessage(PRESSED, left hand)
//!    \li PinchMessage(PRESSED, right hand)
//!    \li BimanualPinchMessage(SIMULTANEOUSLY_PRESSED, both hands)
//! In the case of the double pinch, it can be reported immediately. 
class BimanualPinchMessage : public BasicMessage
{
public:
    //! Constructs a BimanualPinchMessage.  Should not generally be called by users of the API.  
    BimanualPinchMessage(const MessageType type, 
                         const Hand hand,
                         const Vector3f& positionLeft,
                         const Quaternionf& rotationLeft,
                         const int clickCountLeft,
                         const Vector3f& positionRight,
                         const Quaternionf& rotationRight,
                         const int clickCountRight);

    //! @return which hand(s) this message applies to (can be BOTH, for simultaneous pinch/release).
    const Hand getHand() const { return _hand; }

    std::string serialize() const;

private:
    Hand _hand;
};

///
/// Messages relating to pointing
///
class PointMessage : public HandTrackingMessage
{
public:
    //! Constructs a PointMessage.  Should not generally be called by users of the API.  
    PointMessage(const Hand hand,
                 const Vector3f& pointStart,
                 const Vector3f& pointEnd,
                 const float confidence);

    //! @return which hand this message applies to (can only LEFT or RIGHT, never BOTH).  
    const Hand getHand() const { return _hand; }

    std::string serialize() const;

    //! This is our estimate of the start of the index finger at the first knuckle.
    Vector3f getPointStart() const { return _pointStart; }

    //! This is our estimate of the end (tip) of the index finger.  
    Vector3f getPointEnd() const { return _pointEnd; }

    //! This is the direction of the index finger.  
    Vector3f getPointDir() const { return (_pointEnd - _pointStart).normalized(); }

    //! This is the confidence (between 0 and 1) that the user is actually pointing at something.  
    float getConfidence() const  { return _confidence; }

private:
    Hand _hand;
    Vector3f _pointStart;
    Vector3f _pointEnd;
    float _confidence;
};

///
/// Messages giving the full hand pose (all joint angles)
///
class PoseMessage : public BasicMessage
{

public:
    //! An enum distinguishing between the seven current pose types that are recognized by
    //! the hand tracking.
    enum HandPose
    {
        HAND_CURLED = 0,       ///< The hand curled up, ready to pinch
        HAND_ELL = 1,          ///< The sign language "L" shape
        HAND_OKAY = 2,         ///< The "okay" symbol; thumb and index touching and other three fingers spread
        HAND_PINCH = 3,        ///< Thumb and forefinger touching
        HAND_POINTING = 4,     ///< Index finger point
        HAND_RELAXEDOPEN = 5,  ///< What we consider a "neutral" pose, with fingers slightly curled
        HAND_SPREAD = 6        ///< All five fingers spread
    };

    //! Constructs a PoseMessage.  Should not generally be called by users of the API.  
    PoseMessage(const Vector3f& positionLeft,
                const Quaternionf& rotationLeft,
                const int clickCountLeft,
                const Vector3f& positionRight,
                const Quaternionf& rotationRight,
                const int clickCountRight,
                const std::array<float, N_HANDS>& confidenceEstimates,
                const std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS>& jointRotations,
                const std::array<std::array<Vector3f, N_JOINTS>, N_HANDS>& jointTranslations,
                const std::array<std::array<float, N_FINGER_DOFS_PER_HAND>, N_HANDS>& fingerDOFs,
                const std::array<std::array<Vector3f, N_FINGERS>, N_HANDS>& fingerTips,
                const std::array<std::array<float, N_POSES>, N_HANDS>& handPoseConfidences,
                const Transformf cameraTransform);

    std::string serialize() const;

    //! \brief How confident we are about the pose.  
    //!
    //! This is a value between 0 and 1.  Currently only values of 
    //! zero (0) and one (1) are ever returned, but we expect this
    //! to change in future versions.
    float getConfidenceEstimate(size_t hand) const { return _confidenceEstimates[hand]; }

    //! The joint frames of all the bones (as \ref Matrix4); used for skinning. 
    //! 
    //! These should be indexed using the \ref JointFrameIndex enum.
    //!
    //! For more details on the hand model, consult the 
    //! <a href="../handModel.html">hand model docs</a>. 
    std::array<Matrix4f, N_JOINTS> getJointFrames(size_t hand) const;

    //! The joint frames of all the bones (as \ref Transform); used for skinning.  
    //! 
    //! These should be indexed using the \ref JointFrameIndex enum.
    //!
    //! For more details on the hand model, consult the 
    //! <a href="../handModel.html">hand model docs</a>. 
    std::array<Transformf, N_JOINTS> getJointTransforms(size_t hand) const;

    //! \brief Joint angles for the fingers.
    //!
    //! These are the actual joint angles for each finger, which can be used
    //! to determine which fingers are bent for gesture recognition.  Index this using
    //! the \ref FingerDOF enum.
    //!
    //! For more details on the hand model, consult the 
    //! <a href="../handModel.html">hand model docs</a>. 
    const std::array<float, N_FINGER_DOFS_PER_HAND>& getFingerDOFs(size_t hand) const { return _fingerDOFs[hand]; }

    //! The points at the end of each of the fingers.  
    const std::array<Vector3f, N_FINGERS>& getFingerTips(size_t hand) const { return _fingerTips[hand]; }

    //! \brief Experimental function that returns how confident we are that the hand is in a given pose.
    //!
    //! Returns the confidence score for each pose.  Each confidence score
    //! is a value between 0 and 1 and they sum to 1.  Please use the
    //! the \ref PoseMessage::HandPose enum to index into the array.
    const std::array<float, N_POSES>& getHandPoseConfidences(size_t hand) const { return _handPoseConfidences[hand]; }

    //! Returns the camera transform for the pose.  This is an additional transform
    //! that is due to the camera moving in 3-space, due to being mounted on a VR display.
    Transformf getCameraTransform() const { return _cameraTransform; }

private:
    std::array<float, N_HANDS> _confidenceEstimates;
    std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS> _jointRotations;
    std::array<std::array<Vector3f, N_JOINTS>, N_HANDS> _jointTranslations;
    std::array<std::array<float, N_FINGER_DOFS_PER_HAND>, N_HANDS> _fingerDOFs;
    std::array<std::array<Vector3f, N_FINGERS>, N_HANDS> _fingerTips;
    std::array<std::array<float, N_POSES>, N_HANDS> _handPoseConfidences;

    Transformf _cameraTransform;
};

///
/// Messages relating to finger flicking and hand swipes
///
class FlickMessage : public HandTrackingMessage
{
public:
    //! Constructs a FlickMessage with a front-facing mounted sensor.
    //! Should not generally be called by users of the API.  
    FlickMessage(const MessageType type, 
                 const Hand hand,
                 const FlickDirection direction,
                 const float distance);

    //! @return which hand this message applies to (can only be LEFT or RIGHT, never BOTH).  
    const Hand getHand() const { return _hand; }

    //! This is the direction the finger flick (can be LEFT, RIGHT)
    //! or hand swipe is in (can be LEFT, RIGHT, UP, DOWN).
    const FlickDirection getDirection() const { return _direction; }

    std::string serialize() const;

    //! This is the distance of the hand swipe.  
    float getDistance() const  { return _distance; }

private:
    Hand _hand;
    FlickDirection _direction;
    float _distance;
};

///
/// A message sent upon connecting to the server, indicating the server and
/// protocol version.
///
class WelcomeMessage : public HandTrackingMessage
{
public:
    //! Constructs a WelcomeMessage.  Should not generally be called by users of the API.  
    WelcomeMessage (const std::string& serverVersion, 
                    const std::string& protocolVersion,
                    const std::vector<OpenCVCamera>& cameras,
                    const std::string& cameraType,
                    const std::string& mountingPosition,
                    const std::string& dataPath);

    //! Current version of the hand tracking server.  
    const std::string getServerVersion() const { return _serverVersion; }

    //! Current version of the protocol (should change infrequently), 
    //! could be used to detect incompatibility.  
    const std::string getProtocolVersion() const { return _protocolVersion; }

    //! The type of the camera being used
    const std::string getCameraType() const { return _cameraType; }

    //! The location of the camera.
    const std::string getMountingLocation() const { return _mountingLocation; }

    const std::string getDataPath() const { return _dataPath;}

    std::string serialize() const;

    //! Returns the parameters for the cameras that are being used.  
    //! In most cases, you should expect to see only one camera; 
    //! two camera support has been deprecated.  
    const std::vector<OpenCVCamera>& getCameras() const { return _cameras; }

private:
    std::string _serverVersion;
    std::string _protocolVersion;
    std::vector<OpenCVCamera> _cameras;
    std::string _cameraType;
    std::string _mountingLocation;
    std::string _dataPath;
};

//! \brief Message that exposes the user name and skinning information for the user's
//!        calibrated hands.
//!
//! This message will be sent every time the user profile changes.
//! This will happen every time a new client connects, but also if
//! the scale of the user's hands appears to change.  
//!
//! You can use this data to display an active cursor of the user's hand.  
//! For client applications, however, we have moved away from displaying
//! the whole skinned hand (which can be distracting) in favor of more 
//! abstract cursors.  
//!
//! The skinning used by our system is "Linear blend skinning"
//! also known as "Smooth skinning."
//! 
//! http://graphics.ucsd.edu/courses/cse169_w05/3-Skin.htm
//!
class UserMessage : public HandTrackingMessage 
{
public:
    //! A triangle is just 3 ints (one for each vertex).  
    typedef std::array<int,3> Triangle;

    // A list of bone influence indices (for a single vertex).
    typedef std::vector<int> IndicesVector;

    // A list of bone influence weights (for a single vertex).
    typedef std::vector<float> WeightsVector;

    //! \brief The name of the user's profile.
    //! 
    //! In older versions of the software, we required each user to set up a unique
    //! profile that captured various parameters of his/her hand.  Although this is
    //! officially still supported, we're moving away from this model, meaning that
    //! more often than not the profile will be "GenericProfile" more often than not.
    const std::string& getUserProfileName() const { return _userProfileName; };

    //! \brief A number between 0 and 1 representing the scale of the user's hands
    const float getHandScale() const { return _handScale; }

    //! \brief The positions of the hand in its rest pose (with the bones
    //! given by \ref getRestJointFrames).  Indexed by \ref JointFrameIndex.
    const std::vector<Vector3f>& getRestPositions(int hand) const { return _restPositions[hand]; } 

    //! The topology of the skinned hand mesh.  Indexed by \ref JointFrameIndex.
    const std::vector<Triangle>& getTriangles(int hand) const { return _triangles[hand]; }

    //! Skinning indices for the hand.  See \ref getSkinningWeights for an example of how this is used.  
    const std::vector<IndicesVector>& getSkinningIndices(int hand) const { return _skinningIndices[hand]; }

    //! \brief Skinning weights for the hand.  
    //! 
    //! Given a UserMessage and a JointMessage, here is how to generate a skinned
    //! hand model (using linear blend skinning):
    //! \code{.cpp}
    //! UserMessage* userMessage = ...;   // Received once when the profile is set; needs to be cached.
    //! PoseMessage* poseMessage = ...;   // Received once per tracking frame.
    //! for (size_t iHand = 0; iHand < NUMBER_OF_HANDS; ++iHand)
    //! {
    //!     const std::vector<IndicesVector>& skinningIndices = 
    //!         userMessage->getSkinningIndices(iHand);
    //!     const std::vector<WeightsVector>& skinningWeights = 
    //!         userMessage->getWeightsVector(iHand);
    //!     const std::vector<Vector3f>& restPositions = 
    //!         userMessage->getRestPositions(iHand);
    //!     const std::array<Matrix4f, N_JOINTS> currentPoseTransforms = 
    //!         poseMessage->getJointFrames(iHand);
    //!     const std::array<Matrix4f, N_JOINTS> restPoseTransforms = 
    //!         userMessage->getRestJointFrames(iHand);
    //!     
    //!     // The rest points are relative to the rest transforms,
    //!     // so to get the final points we need to _invert_ the
    //!     // the rest transforms before applying the current pose.
    //!     std::array<Matrix4f, N_JOINTS> relativeTransforms;
    //!     for (size_t j = 0; j < N_JOINTS; ++j)
    //!         relativeTransforms[i] = currentPose[j] * restPoseTransforms[j].inverse();
    //!
    //!     std::vector<Vector3f> skinnedPoints = restPositions;
    //!     const size_t nVertices = skinningIndices.size();
    //!     for (size_t jVertex = 0; jVertex != nVertices; ++jVertex)
    //!     {
    //!         // Influence bones/weights for _this_ vertex:
    //!         const IndicesVector& influenceBones = skinningIndices[jVertex];
    //!         const WeightsVector& influenceWeights = skinningWeights[jVertex];
    //!         const size_t nInfluences = influenceBones.size();
    //!         assert (influenceWeights.size() == nInfluences);
    //!
    //!         Vector3f skinnedPosition (0, 0, 0);
    //!         for (size_t kInfluence = 0; kInfluence != nInfluences; ++kInfluence)
    //!         {
    //!             const float w = influenceWeights[kInfluence];
    //!             const int b = influenceBones[kInfluence];
    //!             skinnedPosition += w * (relativeTransforms[b] * restPositions[kVertex]);
    //!         }
    //!         skinnedPoints[jVertex] = skinnedPosition;
    //!     }
    //! }
    //! \endcode
    const std::vector<WeightsVector>& getSkinningWeights(int hand) const { return _skinningWeights[hand]; }

    //! The joint frames of the hand in its rest pose (as \ref Matrix4).  
    std::array<Matrix4f, N_JOINTS> getRestJointFrames(int hand) const;

    //! The joint frames of the hand in its rest pose (as \ref Transform).  
    std::array<Transformf, N_JOINTS> getRestJointTransforms(int hand) const; 

    //! Constructs a UserMessage.  Should not generally be called by users of the API.  
    UserMessage(const std::string& userProfileName, 
                const float handScale,
                const std::array<std::vector<Vector3f>, N_HANDS>& restPositions,
                const std::array<std::vector<Triangle>, N_HANDS>& triangles,
                const std::array<std::vector<IndicesVector>, N_HANDS>& skinningIndices,
                const std::array<std::vector<WeightsVector>, N_HANDS>& skinningWeights,
                const std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS>& restJointRotations,
                const std::array<std::array<Vector3f, N_JOINTS>, N_HANDS>& restJointTranslations);

    std::string serialize() const;

private:
    std::string _userProfileName;
    float _handScale;
    std::array<std::vector<Vector3f>, N_HANDS> _restPositions;
    std::array<std::vector<Triangle>, N_HANDS> _triangles;
    std::array<std::vector<IndicesVector>, N_HANDS> _skinningIndices;
    std::array<std::vector<WeightsVector>, N_HANDS> _skinningWeights;
    std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS> _restJointRotations;
    std::array<std::array<Vector3f, N_JOINTS>, N_HANDS> _restJointTranslations;
};

//! \brief Message that indicates the progress of calibrating the
//! user's hand scale.  
//!
//! For the tracking to succeed, it is important to understand the 
//! overall size of the user's hands.  In order to infer this quickly,
//! we ask that new users spread their fingers out; calibrating then
//! takes about a second.  The CalibrationMessage provides a way for
//! applications to provide feedback to the user about the state of the
//! calibration process.
class CalibrationMessage : public HandTrackingMessage 
{
public:
    //! Constructs a CalibrationMessage.  Should not generally be called by users of the API.  
    CalibrationMessage(const float percentComplete,
                       const float currentScale);

    //! The percentage of completion of the calibration process.  Ranges from
    //! 0 (no calibration active) to 1 (calibration complete).  
    float getPercentComplete() const { return _percentComplete; }

    //! Contains the current hand scale; ranges between roughly 0.5 and 1.3.
    float getCurrentScale() const { return _currentScale; }

    std::string serialize() const;

private:
    float _percentComplete;
    float _currentScale;
};

} // namespace HandTrackingClient

