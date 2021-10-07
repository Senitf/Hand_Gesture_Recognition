#include "HandTrackingMessage.h"

#include <cstdio>

#include <array>
#include <sstream>
#include <memory>
#include <cassert>
#include <cstdint>
#include <climits>

#ifndef WIN32
#include <xlocale.h>
#endif

#ifdef WIN32
typedef _locale_t LocaleType;
#else
typedef locale_t LocaleType;
#endif

namespace HandTrackingClient
{

const char* handToString(Hand hand)
{
    switch (hand) {
    case LEFT_HAND: return "LEFT";
    case RIGHT_HAND: return "RIGHT";
    case BOTH_HANDS: return "BOTH";
    case INVALID_HAND:
    default:
        return "INVALID";
    }
}

Hand stringToHand(const std::string& str)
{
    if (str == "LEFT") return LEFT_HAND;
    if (str == "RIGHT") return RIGHT_HAND;
    if (str == "BOTH") return BOTH_HANDS;
    return INVALID_HAND;
}

const char* flickDirectionToString(FlickDirection direction)
{
    switch (direction) {
    case LEFT_DIRECTION: return "LEFT";
    case RIGHT_DIRECTION: return "RIGHT";
    case UP_DIRECTION: return "UP";
    case DOWN_DIRECTION: return "DOWN";
    case INVALID_DIRECTION:
    default:
        return "INVALID";
    }
}

FlickDirection stringToFlickDirection(const std::string& str)
{
    if (str == "LEFT") return LEFT_DIRECTION;
    if (str == "RIGHT") return RIGHT_DIRECTION;
    if (str == "UP") return UP_DIRECTION;
    if (str == "DOWN") return DOWN_DIRECTION;
    return INVALID_DIRECTION;
}

namespace
{

class TokenStream
{
public:
    typedef std::string::size_type size_type;
    typedef std::string::value_type value_type;

    TokenStream (const std::string& str);
    ~TokenStream();

    // Each of these returns false if they can't parse the expected type:
    bool getSignedInteger (int& value);
    bool getUnsignedInteger (unsigned int& value);
    bool getString (std::string& value);
    bool getRemainingString (std::string& value); // This function is particularly for getting the datapath in Welcome Message
    bool getFloat (float& value);
    bool getFloat (double& value);

    template <typename T>
    bool getQuaternion (Quaternion<T>& value);

    template <typename T>
    bool getVector3 (Vector3<T>& value);

    // Expects to find the passed-in string as the next token;
    // returns false if not found:
    bool expectedString (const char* str);

private:
    // Tells if something is whitespace using the current locale:
    bool isWhitespace (const value_type c) const;

    // Advances _curPos to eat up any whitespace between tokens:
    void eatWhitespace();

    const std::string& _line;
    const size_type _endPos;
    size_type _curPos;

    // Everything needs to be parsed in the "C" locale to ensure
    // consistency across platforms and nationalities:
    const LocaleType _locale;
};

TokenStream::TokenStream (const std::string& str)
    : _line(str), 
      _endPos(str.size()), 
      _curPos(0),
#ifdef WIN32
      _locale (_create_locale (LC_NUMERIC, "C"))
#else
      _locale (newlocale (LC_NUMERIC_MASK, "C", (locale_t) 0))
#endif
{
    if (_locale == nullptr)
        throw ParseException ("Unable to create locale.");

    eatWhitespace();
}

TokenStream::~TokenStream()
{
#ifdef WIN32
    _free_locale (_locale);
#else
    freelocale (_locale);
#endif
}

bool 
TokenStream::isWhitespace (const value_type c) const
{
#ifdef WIN32
    return (_isspace_l (c, _locale) != 0);
#else
    return (isspace_l (c, _locale) != 0);
#endif
}

void
TokenStream::eatWhitespace()
{
    while (_curPos != _endPos && isWhitespace (_line[_curPos]))
        ++_curPos;
}

bool
TokenStream::getString (std::string& value)
{
    if (_curPos == _endPos)
        return false;

    const size_t start = _curPos;
    while (_curPos != _endPos && !isWhitespace (_line[_curPos]))
        ++_curPos;
    const size_t end = _curPos;

    eatWhitespace ();

    value = std::string (_line.begin() + start, _line.begin() + end);
    return true;
}

bool TokenStream::getRemainingString (std::string& value)
{
    if (_curPos == _endPos)
        return false;

    value = std::string (_line.begin() + _curPos,  _line.begin() + _endPos);
    return true;
}

bool
TokenStream::getFloat (double& value)
{
    if (_curPos == _endPos)
        return false;

    const char* linePtr = _line.c_str();
    const char* startPtr = linePtr + _curPos;
    char* endPtr;

#ifdef WIN32
    const double result = _strtod_l (startPtr, &endPtr, _locale);
#else
    const double result = strtod_l (startPtr, &endPtr, _locale);
#endif

    // Failed to parse:
    if (endPtr == nullptr)
        return false;

    _curPos = (endPtr - linePtr);

    // Parsing terminated prematurely:
    if (_curPos != _endPos && !isWhitespace(_line[_curPos]))
        return false;

    eatWhitespace();

    value = result;
    return true;
}

bool
TokenStream::getFloat (float& value)
{
    double res;
    if (!getFloat(res))
        return false;

    value = static_cast<float> (res);
    return true;
}

bool
TokenStream::getUnsignedInteger (unsigned int& value)
{
    int result;
    if (!getSignedInteger (result))
        return false;

    if (result < 0)
        return false;

    value = (unsigned int) result;
    return true;
}

bool
TokenStream::getSignedInteger (int& value)
{
    if (_curPos == _endPos)
        return false;

    const char* linePtr = _line.c_str();
    const char* startPtr = linePtr + _curPos;
    char* endPtr;

#ifdef WIN32
    const int64_t result = _strtoi64_l (startPtr, &endPtr, 10, _locale);
#else
    const int64_t result = strtoll_l (startPtr, &endPtr, 10, _locale);
#endif

    // Failed to parse:
    if (endPtr == nullptr)
        return false;

    _curPos = (endPtr - linePtr);

    // Parsing terminated prematurely:
    if (_curPos != _endPos && !isWhitespace(_line[_curPos]))
        return false;

    eatWhitespace();

    if (result > static_cast<int64_t>(INT_MAX) || result < static_cast<int64_t>(INT_MIN))
        return false;

    value = static_cast<int> (result);
    return true;
}

bool 
TokenStream::expectedString (const char* str)
{
    std::string foundStr;
    if (!getString(foundStr))
        return false;

    return foundStr == str;
}

template <typename T>
bool 
TokenStream::getQuaternion (Quaternion<T>& value)
{
    if (!getFloat(value.v.x)) return false;
    if (!getFloat(value.v.y)) return false;
    if (!getFloat(value.v.z)) return false;
    if (!getFloat(value.w))   return false;
    return true;
}

template <typename T>
bool 
TokenStream::getVector3 (Vector3<T>& value)
{
    if (!getFloat(value.x)) return false;
    if (!getFloat(value.y)) return false;
    if (!getFloat(value.z)) return false;
    return true;
}

} // anonymous namespace

BasicMessage*           parseBasicMessage         (TokenStream& stream, HandTrackingMessage::MessageType messageType);
WelcomeMessage*         parseWelcomeMessage       (TokenStream& stream, HandTrackingMessage::MessageType messageType);
UserMessage*            parseUserMessage          (TokenStream& stream, HandTrackingMessage::MessageType messageType);
CalibrationMessage*     parseCalibrationMessage   (TokenStream& stream, HandTrackingMessage::MessageType messageType);
PoseMessage*            parsePoseMessage          (TokenStream& stream, HandTrackingMessage::MessageType messageType);
PinchMessage*           parsePinchMessage         (TokenStream& stream, HandTrackingMessage::MessageType messageType);
BimanualPinchMessage*   parseBimanualPinchMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType);
PointMessage*           parsePointMessage         (TokenStream& stream, HandTrackingMessage::MessageType messageType);
FlickMessage*           parseFlickMessage         (TokenStream& stream, HandTrackingMessage::MessageType messageType);

HandTrackingMessage::HandTrackingMessage (MessageType msgType)
    : _type (msgType)
{
}

HandTrackingMessage::MessageType
HandTrackingMessage::getType() const
{
    return _type;
}

HandTrackingMessage*
HandTrackingMessage::deserialize(const std::string& data)
{
    TokenStream stream (data);

    std::string messageTypeStr;
    if (!stream.getString(messageTypeStr))
        return nullptr;  // No message (blank line)

    MessageType messageType = stringToMessageType(messageTypeStr);
    switch (messageType)
    {
    case WELCOME:
        return parseWelcomeMessage (stream, messageType);
    case USER:
        return parseUserMessage (stream, messageType);
    case CALIBRATION:
        return parseCalibrationMessage(stream, messageType);
    case POSE:
        return parsePoseMessage (stream, messageType);
    case PRESSED:
    case DRAGGED:
    case RELEASED:
    case MOVED:
    case TRIGGER_PRESSED:
    case TRIGGER_RELEASED:
    case PEACE_BEGAN:
    case PEACE_ENDED:
        return parsePinchMessage (stream, messageType);
    case SIMULTANEOUSLY_PRESSED:
    case INDIVIDUALLY_PRESSED:
    case SIMULTANEOUSLY_RELEASED:
    case INDIVIDUALLY_RELEASED:
    case DRAGGED_BIMANUAL:
        return parseBimanualPinchMessage (stream, messageType);
    case POINT:
        return parsePointMessage (stream, messageType);
    case FINGER_FLICK:
    case HAND_SWIPE:
        return parseFlickMessage (stream, messageType);
    default:
        return nullptr;
    }
}

const char* 
HandTrackingMessage::messageTypeToString( MessageType m )
{
    switch (m)
    {
    case WELCOME: return "WELCOME";
    case USER: return "USER";
    case CALIBRATION: return "CALIBRATION";
    case POSE: return "POSE";
    case PRESSED: return "PRESSED";
    case DRAGGED: return "DRAGGED";
    case RELEASED: return "RELEASED";
    case MOVED: return "MOVED";
    case TRIGGER_PRESSED: return "TRIGGER_PRESSED";
    case TRIGGER_RELEASED: return "TRIGGER_RELEASED";
    case PEACE_BEGAN: return "PEACE_BEGAN";
    case PEACE_ENDED: return "PEACE_ENDED";
    case SIMULTANEOUSLY_PRESSED: return "SIMULTANEOUSLY_PRESSED";
    case INDIVIDUALLY_PRESSED: return "INDIVIDUALLY_PRESSED";
    case SIMULTANEOUSLY_RELEASED: return "SIMULTANEOUSLY_RELEASED";
    case INDIVIDUALLY_RELEASED: return "INDIVIDUALLY_RELEASED";
    case DRAGGED_BIMANUAL: return "DRAGGED_BIMANUAL";
    case POINT: return "POINT";
    case FINGER_FLICK: return "FINGER_FLICK";
    case HAND_SWIPE: return "HAND_SWIPE";
    default: return "INVALID_DATA";
    };
}

HandTrackingMessage::MessageType 
HandTrackingMessage::stringToMessageType( const std::string& str )
{
    if (str == "WELCOME") return WELCOME;
    if (str == "USER") return USER;
    if (str == "CALIBRATION") return CALIBRATION;
    if (str == "POSE") return POSE;
    if (str == "PRESSED") return PRESSED;
    if (str == "DRAGGED") return DRAGGED;
    if (str == "RELEASED") return RELEASED;
    if (str == "MOVED") return MOVED;
    if (str == "TRIGGER_PRESSED") return TRIGGER_PRESSED;
    if (str == "TRIGGER_RELEASED") return TRIGGER_RELEASED;
    if (str == "PEACE_BEGAN") return PEACE_BEGAN; 
    if (str == "PEACE_ENDED") return PEACE_ENDED;
    if (str == "SIMULTANEOUSLY_PRESSED") return SIMULTANEOUSLY_PRESSED;
    if (str == "INDIVIDUALLY_PRESSED") return INDIVIDUALLY_PRESSED;
    if (str == "SIMULTANEOUSLY_RELEASED") return SIMULTANEOUSLY_RELEASED;
    if (str == "INDIVIDUALLY_RELEASED") return INDIVIDUALLY_RELEASED;
    if (str == "DRAGGED_BIMANUAL") return DRAGGED_BIMANUAL;
    if (str == "POINT") return POINT;
    if (str == "FINGER_FLICK") return FINGER_FLICK;
    if (str == "HAND_SWIPE") return HAND_SWIPE;
    return INVALID_DATA;
}

HandTrackingMessage::~HandTrackingMessage() { }

BasicMessage*
parseBasicMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::array<Vector3f,N_HANDS> positions;
    std::array<Quaternionf,N_HANDS> rotations;
    std::array<int,N_HANDS> clickCount;

    // Parse the hand state of each hand
    for (int iHand=0; iHand<N_HANDS; iHand++)
    {
        if (!stream.getVector3(positions[iHand])) throw ParseException ("Error parsing BasicMessage: invalid position.");
        if (!stream.getQuaternion(rotations[iHand])) throw ParseException ("Error parsing BasicMessage: invalid position.");

        if (!stream.getSignedInteger (clickCount[iHand])) throw ParseException ("Error parsing BasicMessage: invalid click count.");
        if (clickCount[iHand] < 0 || clickCount[iHand] > 2)
            throw ParseException ("Error parsing BasicMessage: invalid click count.");
    }

    return new BasicMessage(messageType, 
                            positions[0], rotations[0], clickCount[0], 
                            positions[1], rotations[1], clickCount[1]);
}

BasicMessage::BasicMessage( 
    const MessageType type, 
    const Vector3f& positionLeft, 
    const Quaternionf& rotationLeft, 
    const int clickCountLeft, 
    const Vector3f& positionRight, 
    const Quaternionf& rotationRight, 
    const int clickCountRight) 
    : HandTrackingMessage (type)
{
    _hands[0] = HandState(positionLeft, rotationLeft, clickCountLeft);
    _hands[1] = HandState(positionRight, rotationRight, clickCountRight);
} 

std::string BasicMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeToString(getType());
    for (size_t iHand = 0; iHand < N_HANDS; ++iHand)
        buffer << " " << _hands[iHand].getPosition() << " " << _hands[iHand].getRotation() << " " << _hands[iHand].getClickCount();
    return buffer.str();
}

BasicMessage::~BasicMessage() { }

PinchMessage::PinchMessage( 
    const MessageType type, 
    const Hand hand,
    const Vector3f& positionLeft, 
    const Quaternionf& rotationLeft, 
    const int clickCountLeft, 
    const Vector3f& positionRight, 
    const Quaternionf& rotationRight, 
    const int clickCountRight) 
    : BasicMessage(type, positionLeft, rotationLeft, clickCountLeft, 
                         positionRight, rotationRight, clickCountRight),
      _hand(hand) { } 

std::string PinchMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << BasicMessage::serialize();
    buffer << " " << handToString(_hand);
    return buffer.str();
}

PinchMessage*
parsePinchMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::auto_ptr<BasicMessage> basicMessage (parseBasicMessage (stream, messageType));
    const HandState left = basicMessage->getHandState(0);
    const HandState right = basicMessage->getHandState(1);

    std::string handStr;
    if (!stream.getString(handStr))
        throw ParseException ("Error parsing PinchMessage: no hand found."); 

    return new PinchMessage (messageType,
                             stringToHand(handStr),
                             left.getPosition(),
                             left.getRotation(),
                             left.getClickCount(),
                             right.getPosition(),
                             right.getRotation(),
                             right.getClickCount());
}

BimanualPinchMessage::BimanualPinchMessage( 
    const MessageType type, 
    const Hand hand,
    const Vector3f& positionLeft, 
    const Quaternionf& rotationLeft, 
    const int clickCountLeft, 
    const Vector3f& positionRight, 
    const Quaternionf& rotationRight, 
    const int clickCountRight) 
    : BasicMessage(type, positionLeft, rotationLeft, clickCountLeft, 
                         positionRight, rotationRight, clickCountRight),
      _hand(hand) { } 

std::string 
BimanualPinchMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << BasicMessage::serialize();
    buffer << " " << handToString(_hand);
    return buffer.str();
}

BimanualPinchMessage*
parseBimanualPinchMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::auto_ptr<BasicMessage> basicMessage (parseBasicMessage (stream, messageType));
    const HandState left = basicMessage->getHandState(0);
    const HandState right = basicMessage->getHandState(1);

    std::string handStr;
    if (!stream.getString(handStr))
        throw ParseException ("Error parsing BimanualPinchMessage: no hand found."); 

    return new BimanualPinchMessage(messageType,
                                    stringToHand(handStr),
                                    left.getPosition(),
                                    left.getRotation(),
                                    left.getClickCount(),
                                    right.getPosition(),
                                    right.getRotation(),
                                    right.getClickCount());
}

PointMessage::PointMessage(const Hand hand,
                           const Vector3f& pointStart,
                           const Vector3f& pointEnd,
                           const float confidence)
    : HandTrackingMessage (POINT),
      _hand (hand),
      _pointStart (pointStart),
      _pointEnd (pointEnd),
      _confidence (confidence)
{
}

std::string 
PointMessage::serialize() const 
{
    const char* handName = handToString(_hand);
    const char* messageTypeName = messageTypeToString(getType());
    assert (handName != nullptr);
    assert (messageTypeName != nullptr);
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeName << " " << handName
        << " " << _pointStart
        << " " << _pointEnd
        << " " << _confidence;
    return buffer.str();
}

PointMessage*
parsePointMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::string handStr;
    if (!stream.getString (handStr))
        throw ParseException ("Error parsing PointMessage: no hand found.");

    const Hand hand = stringToHand(handStr);
    if (hand == INVALID_HAND) 
        throw ParseException("Error parsing PointMessage: invalid HAND.");

    Vector3f pointStart;
    if (!stream.getVector3 (pointStart))  throw ParseException("Error parsing PointMessage: invalid pointing start point.");

    Vector3f pointEnd;
    if (!stream.getVector3 (pointEnd)) throw ParseException("Error parsing PointMessage: invalid pointing endpoint.");

    float confidence;
    if (!stream.getFloat (confidence)) throw ParseException("Error parsing PointMessage: invalid confidence value.");

    if (confidence < 0.0f || confidence > 1.0f)
        throw ParseException("Error parsing PointMessage: invalid confidence value.");

    return new PointMessage(hand, pointStart, pointEnd, confidence);
}

PoseMessage::PoseMessage(const Vector3f& positionLeft,
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
                         const Transformf cameraTransform)
                         : BasicMessage(HandTrackingMessage::POSE, 
                                        positionLeft, rotationLeft, clickCountLeft, 
                                        positionRight, rotationRight, clickCountRight),
                           _confidenceEstimates(confidenceEstimates),
                           _jointRotations(jointRotations),
                           _jointTranslations(jointTranslations),
                           _fingerDOFs(fingerDOFs),
                           _fingerTips(fingerTips),
                           _handPoseConfidences(handPoseConfidences),
                           _cameraTransform (cameraTransform) { }

std::string 
PoseMessage::serialize() const 
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << BasicMessage::serialize();
    for (int iHand=0; iHand<N_HANDS; iHand++) 
    {
        buffer << " " << _confidenceEstimates[iHand];
        for (int jJoint=0; jJoint<N_JOINTS; jJoint++)
        {
            buffer << " " << _jointRotations[iHand][jJoint];
            buffer << " " << _jointTranslations[iHand][jJoint];
        }
        for (int jFinger=0; jFinger<N_FINGERS; jFinger++)
        {
            buffer << " " << _fingerTips[iHand][jFinger];
        }
    }

    for (int iHand=0; iHand<N_HANDS; iHand++) 
    {
        for(int jPose = 0; jPose < N_POSES; jPose++)
        {
            buffer << " " << _handPoseConfidences[iHand][jPose];
        }
    }

    for (int iHand=0; iHand<N_HANDS; iHand++) 
    {
        for(int jDOF = 0; jDOF < N_FINGER_DOFS_PER_HAND; jDOF++)
        {
            buffer << " " << _fingerDOFs[iHand][jDOF];
        }
    }

    buffer << " " << _cameraTransform.rotation << " " << _cameraTransform.translation << " ";

    return buffer.str();
}

PoseMessage*
parsePoseMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::auto_ptr<BasicMessage> basicMessage (parseBasicMessage (stream, messageType));

    const HandState left = basicMessage->getHandState(0);
    const HandState right = basicMessage->getHandState(1);

    std::array<float,N_HANDS> confidenceEstimates;
    std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS> jointRotations;
    std::array<std::array<Vector3f, N_JOINTS>, N_HANDS> jointTranslations;
    std::array<std::array<Vector3f, N_FINGERS>, N_HANDS> fingerTips;
    std::array<std::array<float, N_POSES>, N_HANDS> handPoseConfidences;
    std::array<std::array<float, N_FINGER_DOFS_PER_HAND>, N_HANDS> fingerDOFs;

    for (int iHand=0; iHand<N_HANDS; iHand++)
    {
        if (!stream.getFloat (confidenceEstimates[iHand])) throw ParseException ("Error parsing PoseMessage: invalid confidence.");

        if (confidenceEstimates[iHand] < 0.0f || confidenceEstimates[iHand] > 1.0f)
            throw ParseException ("Error parsing PoseMessage: invalid confidence.");

        for (int jJoint=0; jJoint<N_JOINTS; jJoint++) 
        {
            if (!stream.getQuaternion (jointRotations[iHand][jJoint])) throw ParseException ("Error parsing PoseMessage: invalid joint rotation.");
            if (!stream.getVector3 (jointTranslations[iHand][jJoint])) throw ParseException ("Error parsing PoseMessage: invalid joint translation.");
        }

        for (int jFingerTip=0; jFingerTip < N_FINGERS; jFingerTip++) 
        {
            if (!stream.getVector3 (fingerTips[iHand][jFingerTip])) throw ParseException ("Error parsing PoseMessage: invalid finger tips.");
        }
    }

    for (int iHand=0; iHand<N_HANDS; iHand++)
    {
        for (int jPose=0; jPose<N_POSES; jPose++) 
        {
            if (!stream.getFloat (handPoseConfidences[iHand][jPose])) throw ParseException ("Error parsing PoseMessage: invalid pose confidence estimates.");
        }
    }

    for (int iHand=0; iHand<N_HANDS; iHand++)
    {
        for (int jDOF=0; jDOF<N_FINGER_DOFS_PER_HAND; jDOF++) 
        {
            if (!stream.getFloat (fingerDOFs[iHand][jDOF])) throw ParseException ("Error parsing PoseMessage: invalid finger DOFs.");
        }
    }

    Transformf cameraTransform;
    if (!stream.getQuaternion (cameraTransform.rotation))
        throw ParseException ("Error parsing PoseMessage: expected camera rotation.");

    if (!stream.getVector3 (cameraTransform.translation))
        throw ParseException ("Error parsing PoseMessage: expected camera translation.");

    return new PoseMessage (left.getPosition(),
                            left.getRotation(),
                            left.getClickCount(),
                            right.getPosition(),
                            right.getRotation(),
                            right.getClickCount(),
                            confidenceEstimates,
                            jointRotations,
                            jointTranslations,
                            fingerDOFs,
                            fingerTips,
                            handPoseConfidences,
                            cameraTransform);
}

std::array<Matrix4f, N_JOINTS> 
PoseMessage::getJointFrames( size_t hand ) const
{
    std::array<Matrix4f, N_JOINTS> transforms;

    for (int jJoint=0; jJoint < N_JOINTS; jJoint++)
    {
        transforms[jJoint] = Matrix4f(_jointRotations[hand][jJoint], _jointTranslations[hand][jJoint]);
    }
    return transforms;
}

std::array<Transformf, N_JOINTS> 
PoseMessage::getJointTransforms( size_t hand ) const
{
    std::array<Transformf, N_JOINTS> transforms;

    for (int jJoint=0; jJoint < N_JOINTS; jJoint++)
    {
        transforms[jJoint] = Transformf(_jointRotations[hand][jJoint], _jointTranslations[hand][jJoint]);
    }
    return transforms;
}

FlickMessage::FlickMessage(const MessageType type,
                           const Hand hand,
                           const FlickDirection direction,
                           const float distance)
    : HandTrackingMessage (type),
      _hand (hand),
      _direction (direction),
      _distance (distance)
{
}

std::string 
FlickMessage::serialize() const 
{
    const char* handName = handToString(_hand);
    const char* directionName = flickDirectionToString(_direction);
    const char* messageTypeName = messageTypeToString(getType());
    assert (handName != nullptr);
    assert (messageTypeName != nullptr);
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeName << " " << handName
        << " " << directionName
        << " " << _distance;
    return buffer.str();
}

FlickMessage*
parseFlickMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    std::string handStr;
    if (!stream.getString (handStr))
        throw ParseException ("Error parsing FlickMessage: no hand found.");

    const Hand hand = stringToHand(handStr);
    if (hand == INVALID_HAND) 
        throw ParseException("Error parsing FlickMessage: invalid HAND.");

    std::string directionStr;
    if (!stream.getString (directionStr)) throw ParseException("Error parsing PointMessage: invalid direction value.");

    const FlickDirection direction = stringToFlickDirection(directionStr);

    float distance;
    if (!stream.getFloat (distance)) throw ParseException("Error parsing PointMessage: invalid distance value.");
    
    return new FlickMessage(messageType, hand, direction, distance);
}

WelcomeMessage::WelcomeMessage(const std::string& serverVersion, 
                               const std::string& protocolVersion,
                               const std::vector<OpenCVCamera>& cameras,
                               const std::string& cameraType,
                               const std::string& mountingLocation,
                               const std::string& dataPath) :
    HandTrackingMessage (WELCOME),
    _serverVersion (serverVersion), 
    _protocolVersion (protocolVersion),
    _cameras (cameras),
    _cameraType(cameraType),
    _mountingLocation(mountingLocation),
    _dataPath(dataPath)
{
}

std::string
WelcomeMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeToString(getType());
    buffer << " Server-Version: " << _serverVersion;
    buffer << " Protocol-Version: " << _protocolVersion;

    buffer.precision (15);
    buffer << " Cameras: " << _cameras.size();
    for (size_t iCamera = 0; iCamera < _cameras.size(); ++iCamera)
    {
        const OpenCVCamera& camera = _cameras[iCamera];
        buffer << " " << camera.imageWidth << " " << camera.imageHeight;
        buffer << " " << camera.extrinsics.rotation << " " << camera.extrinsics.translation;
        buffer << " " << camera.fx << " " << camera.fy;
        buffer << " " << camera.cx << " " << camera.cy;

        const double k4 = 0, k5 = 0, k6 = 0;
        buffer << " " << camera.k1 << " " << camera.k2;
        buffer << " " << camera.p1 << " " << camera.p2;
        buffer << " " << camera.k3 << " " << k4 << " " << k5 << " " << k6;
    }

    buffer << " Camera-Type: " << _cameraType;
    buffer << " Mounting-Location: " << _mountingLocation;

    buffer << " Data-Path: " << _dataPath;

    return buffer.str();
}

WelcomeMessage*
parseWelcomeMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    if (!stream.expectedString ("Server-Version:"))
        throw ParseException ("Error parsing welcome message: expected 'Server-Version:'.");
    std::string serverVersion;
    if (!stream.getString (serverVersion))
        throw ParseException ("Error parsing welcome message: expected server version.");

    if (!stream.expectedString ("Protocol-Version:"))
        throw ParseException ("Error parsing welcome message: expected 'Protocol-Version:'.");
    std::string protocolVersion;
    if (!stream.getString (protocolVersion))
        throw ParseException ("Error parsing welcome message: expected protocol version.");

    unsigned int nCameras;
    if (!stream.expectedString ("Cameras:"))
        throw ParseException ("Error parsing welcome message: expected 'Cameras:'.");

    if (!stream.getUnsignedInteger (nCameras))
        throw ParseException ("Error parsing welcome message: expected num cameras.");

    if (nCameras < 1 || nCameras > 2)
        throw ParseException ("Error parsing welcome message: unexpected camera count.");

    std::vector<OpenCVCamera> cameras;
    for (unsigned int iCamera = 0; iCamera < nCameras; ++iCamera)
    {
        unsigned int imageDim[2];
        for (unsigned int i = 0; i < 2; ++i)
        {
            if (!stream.getUnsignedInteger (imageDim[i]))
                throw ParseException ("Error parsing welcome message: expected image dimensions.");
        }

        Quaterniond extrinsics_rot;
        if (!stream.getQuaternion (extrinsics_rot))
            throw ParseException ("Error parsing welcome message: expected extrinsics.");

        Vector3d extrinsics_trans;
        if (!stream.getVector3 (extrinsics_trans))
            throw ParseException ("Error parsing welcome message: expected extrinsics.");

        double focalLength[2];
        for (unsigned int i = 0; i < 2; ++i)
        {
            if (!stream.getFloat (focalLength[i]))
                throw ParseException ("Error parsing welcome message: expected focal length.");
        }

        double cameraCenter[2];
        for (unsigned int i = 0; i < 2; ++i)
        {
            if (!stream.getFloat (cameraCenter[i]))
                throw ParseException ("Error parsing welcome message: expected camera center.");
        }

        // Passed in the order that OpenCV passes them,
        //    k1, k2, p1, p2, k3, k4, k5, k6
        // (see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
        // Note that k4-k6 are ignored here; we're only leaving space for them for 
        // possible forward compatibility with future cameras.
        double distortionParams[8];
        for (unsigned int i = 0; i < 8; ++i)
        {
            if (!stream.getFloat (distortionParams[i]))
                throw ParseException ("Error parsing welcome message: expected distortion parameters.");
        }

        const OpenCVCamera camera (imageDim[0], imageDim[1],
                                   Transformd (extrinsics_rot, extrinsics_trans),
                                   focalLength[0], focalLength[1],
                                   cameraCenter[0], cameraCenter[1],
                                   distortionParams[0], distortionParams[1], 
                                   distortionParams[2], distortionParams[3], 
                                   distortionParams[4]);
        cameras.push_back (camera);
    }

    if (!stream.expectedString ("Camera-Type:"))
        throw ParseException ("Error parsing welcome message: expected 'Camera-Type:'.");
    std::string cameraType;
    if (!stream.getString (cameraType))
        throw ParseException ("Error parsing welcome message: expected camera type.");

    if (!stream.expectedString ("Mounting-Location:"))
        throw ParseException ("Error parsing welcome message: expected 'Mounting-Location:'.");
    std::string mountingLocation;
    if (!stream.getString (mountingLocation))
        throw ParseException ("Error parsing welcome message: expected mounting location.");

    if (!stream.expectedString ("Data-Path:"))
        throw ParseException ("Error parsing welcome message: expected 'Data-Path:'.");
    std::string dataPath;
    if (!stream.getRemainingString (dataPath))
        throw ParseException ("Error parsing welcome message: expected data path.");

    return new WelcomeMessage(serverVersion, protocolVersion, cameras, cameraType, mountingLocation, dataPath);
}

UserMessage::UserMessage(const std::string& userProfileName, 
                         const float handScale,
                         const std::array<std::vector<Vector3f>, N_HANDS>& restPositions,
                         const std::array<std::vector<Triangle>, N_HANDS>& triangles,
                         const std::array<std::vector<IndicesVector>, N_HANDS>& skinningIndices,
                         const std::array<std::vector<WeightsVector>, N_HANDS>& skinningWeights,
                         const std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS>& restJointRotations,
                         const std::array<std::array<Vector3f, N_JOINTS>, N_HANDS>& restJointTranslations)
                         : HandTrackingMessage (USER),
                           _userProfileName(userProfileName),
                           _handScale(handScale),
                           _restPositions(restPositions),
                           _triangles(triangles),
                           _skinningIndices(skinningIndices),
                           _skinningWeights(skinningWeights),
                           _restJointRotations(restJointRotations),
                           _restJointTranslations(restJointTranslations) { }

std::string 
UserMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeToString(getType());
    buffer << " User: " << _userProfileName;
    buffer << " Scale: " << _handScale;
    for (int iHand=0; iHand<N_HANDS; iHand++)
    {
        buffer << " Hand: " << iHand;
        buffer << " Rest-Positions: " << _restPositions[iHand].size();
        for (size_t v=0; v<_restPositions[iHand].size(); v++) 
        {
            buffer << " " << _restPositions[iHand][v];
        }

        buffer << " Triangles: " << _triangles[iHand].size();
        for (size_t jTriangle=0; jTriangle<_triangles[iHand].size(); jTriangle++)
        {
            Triangle p = _triangles[iHand][jTriangle];
            buffer << " " << p[0] << " " << p[1] << " " << p[2];
        }

        buffer << " Skinning-Weights:";
        for (size_t i=0; i<_skinningIndices[iHand].size(); i++) 
        {
            IndicesVector indices = _skinningIndices[iHand][i];
            WeightsVector weights = _skinningWeights[iHand][i];

            buffer << " " << indices.size();

            for (size_t j=0; j<indices.size(); j++) 
            {
                buffer << " ";
                buffer << indices[j];
                buffer << " ";
                buffer << weights[j];
            }
        }

        buffer << " Rest-Joint-Frames:";
        for (size_t jJoint=0; jJoint<N_JOINTS; jJoint++)
        {
            buffer << " " << _restJointRotations[iHand][jJoint];
            buffer << " " << _restJointTranslations[iHand][jJoint];
        }
    }
    
    return buffer.str();
}

UserMessage*
parseUserMessage (TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    if (!stream.expectedString ("User:"))
        throw ParseException ("Error parsing User message: missing 'User'.");

    std::string userProfileName;
    if (!stream.getString(userProfileName))
        throw ParseException ("Error parsing User message: missing profile name.");

    if (!stream.expectedString ("Scale:"))
        throw ParseException ("Error parsing User message: missing 'Scale'.");

    float handScale;
    if (!stream.getFloat (handScale)) 
        throw ParseException ("Error parsing User message: unable to read hand scale.");

    typedef UserMessage::Triangle Triangle;
    typedef UserMessage::IndicesVector IndicesVector;
    typedef UserMessage::WeightsVector WeightsVector;

    std::array<std::vector<Vector3f>, N_HANDS> restPositions;
    std::array<std::vector<Triangle>, N_HANDS> triangles;
    std::array<std::vector<IndicesVector>, N_HANDS> skinningIndices;
    std::array<std::vector<WeightsVector>, N_HANDS> skinningWeights;
    std::array<std::array<Quaternionf, N_JOINTS>, N_HANDS> restJointRotations;
    std::array<std::array<Vector3f, N_JOINTS>, N_HANDS> restJointTranslations;

    const int TOO_MANY = 100000;  // Just a sanity check on the vertex count
                                  // to make sure if we get bad data we don't try to allocate
                                  // an infinite-sized array.

    for (unsigned int iHand=0; iHand<N_HANDS; iHand++) 
    {
        if (!stream.expectedString ("Hand:"))
            throw ParseException ("Error parsing User message: missing 'Hand'");

        unsigned int hand;
        if (!stream.getUnsignedInteger (hand)) throw ParseException ("Error parsing User message: invalid hand index.");

        if (hand != iHand)
            throw ParseException ("Error parsing User message: invalid hand index.");

        if (!stream.expectedString ("Rest-Positions:"))
            throw ParseException ("Error parsing User message: missing 'Rest-Positions:'");

        unsigned int nVertices;
        if (!stream.getUnsignedInteger (nVertices)) throw ParseException ("Error parsing User message: unable to parse vertex count.");

        if (nVertices <= 0 || nVertices > TOO_MANY)
            throw ParseException ("Error parsing User message: invalid vertex count.");

        restPositions[iHand].resize(nVertices);
        skinningIndices[iHand].resize(nVertices);
        skinningWeights[iHand].resize(nVertices);

        for (size_t i=0; i<restPositions[iHand].size(); i++) 
        {
            if (!stream.getVector3 (restPositions[iHand][i])) throw ParseException ("Error parsing User message: invalid rest positions.");
        }

        if (!stream.expectedString ("Triangles:"))
            throw ParseException ("Error parsing User message: missing 'Triangles:'");

        unsigned int nTriangles;
        if (!stream.getUnsignedInteger (nTriangles)) throw ParseException ("Error parsing User message: unable to parse vertex count.");

        if (nTriangles <= 0 || nTriangles > TOO_MANY)
            throw ParseException ("Error parsing User message: invalid triangle count.");;

        triangles[iHand].resize(nTriangles);

        for (int i=0; i<triangles[iHand].size(); i++) 
        {
            for (int j=0; j<triangles[iHand][i].size(); j++) 
            {
                unsigned int idx;
                if (!stream.getUnsignedInteger (idx)) throw ParseException ("Error parsing User message: unable to parse vertex index.");

                if (idx >= nVertices)
                    throw ParseException ("Error parsing User message: invalid vertex index.");

                triangles[iHand][i][j] = (int) idx;
            }
        }

        if (!stream.expectedString ("Skinning-Weights:"))
            throw ParseException ("Error parsing User message: missing 'Skinning-Weights:'");

        for (int i=0; i<skinningIndices[iHand].size(); i++) 
        {
            unsigned int nInfluences;
            if (!stream.getUnsignedInteger (nInfluences)) throw ParseException ("Error parsing User message: unable to parse joint influence count.");

            if (nInfluences > 10)
                throw ParseException ("Error parsing User message: too many joint influences.");

            skinningIndices[iHand][i].resize(nInfluences);
            skinningWeights[iHand][i].resize(nInfluences);
            for (unsigned int j=0; j<nInfluences; j++) 
            {
                int bone;
                if (!stream.getSignedInteger (bone)) throw ParseException ("Error parsing User message: unable to read joint influence bone.");
                if (bone < 0 || bone >= N_JOINTS)
                    throw ParseException ("Error parsing User message: invalid joint influence bone.");

                float weight;
                if (!stream.getFloat (weight)) throw ParseException ("Error parsing User message: unable to read joint influence weight.");
                
                if (weight < 0.0f || weight > 1.0f)
                    throw ParseException ("Error parsing User message: invalid joint influence weight.");

                skinningIndices[iHand][i][j] = bone;
                skinningWeights[iHand][i][j] = weight;
            }
        }

        if (!stream.expectedString ("Rest-Joint-Frames:"))
            throw ParseException ("Error parsing User message: missing 'Rest-Joint-Frames:'");

        for (int jJoint=0; jJoint<N_JOINTS; jJoint++)
        {
            if (!stream.getQuaternion (restJointRotations[iHand][jJoint])) throw ParseException ("Error parsing User message: invalid rest joint rotation.");
            if (!stream.getVector3 (restJointTranslations[iHand][jJoint])) throw ParseException ("Error parsing User message: invalid rest joint translation.");
        }
    }

    return new UserMessage (userProfileName, 
                            handScale,
                            restPositions, 
                            triangles, 
                            skinningIndices, 
                            skinningWeights, 
                            restJointRotations, 
                            restJointTranslations);
}

CalibrationMessage::CalibrationMessage(const float percentComplete,
                                       const float currentScale) 
    : HandTrackingMessage (CALIBRATION),
      _percentComplete(percentComplete),
      _currentScale(currentScale)
{
}

CalibrationMessage* 
parseCalibrationMessage(TokenStream& stream, HandTrackingMessage::MessageType messageType)
{
    if (!stream.expectedString ("Percent-Complete:"))
        throw ParseException ("Error parsing calibration message: expected 'Percent-Complete:'.");
    float percentComplete;
    if (!stream.getFloat (percentComplete) || percentComplete < 0.0f || percentComplete > 1.0f)
        throw ParseException ("Error parsing calibration message: invalid percent complete.");

    if (!stream.expectedString ("Current-Scale:"))
        throw ParseException ("Error parsing calibration message: expected 'Current-Scale:'.");
    float currentScale;
    if (!stream.getFloat (currentScale) || currentScale < 0.0f)
        throw ParseException ("Error parsing calibration message: invalid current scale.");

    return new CalibrationMessage(percentComplete, currentScale);
}


std::array<Matrix4f, N_JOINTS> 
UserMessage::getRestJointFrames( int hand ) const
{
    std::array<Matrix4f, N_JOINTS> jointFrames;
    for (int jJoint=0; jJoint<N_JOINTS; jJoint++)
    {
        jointFrames[jJoint] = Matrix4f(_restJointRotations[hand][jJoint], 
                                       _restJointTranslations[hand][jJoint]);
    }
    return jointFrames;
}

std::array<Transformf, N_JOINTS> 
UserMessage::getRestJointTransforms( int hand ) const
{
    std::array<Transformf, N_JOINTS> jointFrames;
    for (int jJoint=0; jJoint<N_JOINTS; jJoint++)
    {
        jointFrames[jJoint] = Transformf(_restJointRotations[hand][jJoint], 
                                         _restJointTranslations[hand][jJoint]);
    }
    return jointFrames;
}

std::string 
CalibrationMessage::serialize() const
{
    std::ostringstream buffer;
    buffer.imbue (std::locale("C")); // Listeners assume en-US locale for numbers

    buffer << messageTypeToString(getType());
    buffer << " Percent-Complete: " << _percentComplete;
    buffer << " Current-Scale: " << _currentScale;
    
    return buffer.str();
}

} // namespace HandTrackingClient
