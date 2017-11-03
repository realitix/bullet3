enum EnumSharedMemoryClientCommand
{
    CMD_LOAD_SDF,
 CMD_LOAD_URDF,
 CMD_LOAD_BULLET,
 CMD_SAVE_BULLET,
 CMD_LOAD_MJCF,
    CMD_LOAD_BUNNY,
 CMD_SEND_BULLET_DATA_STREAM,
 CMD_CREATE_BOX_COLLISION_SHAPE,
 CMD_CREATE_RIGID_BODY,
 CMD_DELETE_RIGID_BODY,
 CMD_CREATE_SENSOR,
 CMD_INIT_POSE,
 CMD_SEND_PHYSICS_SIMULATION_PARAMETERS,
 CMD_SEND_DESIRED_STATE,
 CMD_REQUEST_ACTUAL_STATE,
 CMD_REQUEST_DEBUG_LINES,
    CMD_REQUEST_BODY_INFO,
 CMD_REQUEST_INTERNAL_DATA,
    CMD_STEP_FORWARD_SIMULATION,
    CMD_RESET_SIMULATION,
    CMD_PICK_BODY,
    CMD_MOVE_PICKED_BODY,
    CMD_REMOVE_PICKING_CONSTRAINT_BODY,
    CMD_REQUEST_CAMERA_IMAGE_DATA,
    CMD_APPLY_EXTERNAL_FORCE,
 CMD_CALCULATE_INVERSE_DYNAMICS,
    CMD_CALCULATE_INVERSE_KINEMATICS,
    CMD_CALCULATE_JACOBIAN,
    CMD_USER_CONSTRAINT,
    CMD_REQUEST_CONTACT_POINT_INFORMATION,
    CMD_REQUEST_RAY_CAST_INTERSECTIONS,
 CMD_REQUEST_AABB_OVERLAP,
 CMD_SAVE_WORLD,
 CMD_REQUEST_VISUAL_SHAPE_INFO,
    CMD_UPDATE_VISUAL_SHAPE,
    CMD_LOAD_TEXTURE,
    CMD_SET_SHADOW,
 CMD_USER_DEBUG_DRAW,
 CMD_REQUEST_VR_EVENTS_DATA,
 CMD_SET_VR_CAMERA_STATE,
 CMD_SYNC_BODY_INFO,
 CMD_STATE_LOGGING,
    CMD_CONFIGURE_OPENGL_VISUALIZER,
 CMD_REQUEST_KEYBOARD_EVENTS_DATA,
 CMD_REQUEST_OPENGL_VISUALIZER_CAMERA,
 CMD_REMOVE_BODY,
 CMD_CHANGE_DYNAMICS_INFO,
 CMD_GET_DYNAMICS_INFO,
 CMD_PROFILE_TIMING,
 CMD_CREATE_COLLISION_SHAPE,
 CMD_CREATE_VISUAL_SHAPE,
 CMD_CREATE_MULTI_BODY,
 CMD_REQUEST_COLLISION_INFO,
 CMD_REQUEST_MOUSE_EVENTS_DATA,
 CMD_CHANGE_TEXTURE,
    CMD_MAX_CLIENT_COMMANDS,
};
enum EnumSharedMemoryServerStatus
{
        CMD_SHARED_MEMORY_NOT_INITIALIZED=0,
        CMD_WAITING_FOR_CLIENT_COMMAND,
        CMD_CLIENT_COMMAND_COMPLETED,
        CMD_UNKNOWN_COMMAND_FLUSHED,
  CMD_SDF_LOADING_COMPLETED,
        CMD_SDF_LOADING_FAILED,
        CMD_URDF_LOADING_COMPLETED,
        CMD_URDF_LOADING_FAILED,
  CMD_BULLET_LOADING_COMPLETED,
  CMD_BULLET_LOADING_FAILED,
  CMD_BULLET_SAVING_COMPLETED,
  CMD_BULLET_SAVING_FAILED,
  CMD_MJCF_LOADING_COMPLETED,
  CMD_MJCF_LOADING_FAILED,
  CMD_REQUEST_INTERNAL_DATA_COMPLETED,
  CMD_REQUEST_INTERNAL_DATA_FAILED,
        CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED,
        CMD_BULLET_DATA_STREAM_RECEIVED_FAILED,
        CMD_BOX_COLLISION_SHAPE_CREATION_COMPLETED,
        CMD_RIGID_BODY_CREATION_COMPLETED,
        CMD_SET_JOINT_FEEDBACK_COMPLETED,
        CMD_ACTUAL_STATE_UPDATE_COMPLETED,
        CMD_ACTUAL_STATE_UPDATE_FAILED,
        CMD_DEBUG_LINES_COMPLETED,
        CMD_DEBUG_LINES_OVERFLOW_FAILED,
        CMD_DESIRED_STATE_RECEIVED_COMPLETED,
        CMD_STEP_FORWARD_SIMULATION_COMPLETED,
        CMD_RESET_SIMULATION_COMPLETED,
        CMD_CAMERA_IMAGE_COMPLETED,
        CMD_CAMERA_IMAGE_FAILED,
        CMD_BODY_INFO_COMPLETED,
        CMD_BODY_INFO_FAILED,
  CMD_INVALID_STATUS,
  CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED,
  CMD_CALCULATED_INVERSE_DYNAMICS_FAILED,
        CMD_CALCULATED_JACOBIAN_COMPLETED,
        CMD_CALCULATED_JACOBIAN_FAILED,
  CMD_CONTACT_POINT_INFORMATION_COMPLETED,
  CMD_CONTACT_POINT_INFORMATION_FAILED,
  CMD_REQUEST_AABB_OVERLAP_COMPLETED,
  CMD_REQUEST_AABB_OVERLAP_FAILED,
  CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED,
  CMD_CALCULATE_INVERSE_KINEMATICS_FAILED,
  CMD_SAVE_WORLD_COMPLETED,
  CMD_SAVE_WORLD_FAILED,
        CMD_VISUAL_SHAPE_INFO_COMPLETED,
        CMD_VISUAL_SHAPE_INFO_FAILED,
        CMD_VISUAL_SHAPE_UPDATE_COMPLETED,
        CMD_VISUAL_SHAPE_UPDATE_FAILED,
        CMD_LOAD_TEXTURE_COMPLETED,
        CMD_LOAD_TEXTURE_FAILED,
  CMD_USER_DEBUG_DRAW_COMPLETED,
  CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED,
  CMD_USER_DEBUG_DRAW_FAILED,
  CMD_USER_CONSTRAINT_COMPLETED,
  CMD_USER_CONSTRAINT_INFO_COMPLETED,
        CMD_REMOVE_USER_CONSTRAINT_COMPLETED,
        CMD_CHANGE_USER_CONSTRAINT_COMPLETED,
  CMD_REMOVE_USER_CONSTRAINT_FAILED,
        CMD_CHANGE_USER_CONSTRAINT_FAILED,
  CMD_USER_CONSTRAINT_FAILED,
  CMD_REQUEST_VR_EVENTS_DATA_COMPLETED,
  CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED,
  CMD_SYNC_BODY_INFO_COMPLETED,
  CMD_SYNC_BODY_INFO_FAILED,
  CMD_STATE_LOGGING_COMPLETED,
  CMD_STATE_LOGGING_START_COMPLETED,
  CMD_STATE_LOGGING_FAILED,
  CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED,
  CMD_REQUEST_KEYBOARD_EVENTS_DATA_FAILED,
  CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_FAILED,
  CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_COMPLETED,
  CMD_REMOVE_BODY_COMPLETED,
  CMD_REMOVE_BODY_FAILED,
  CMD_GET_DYNAMICS_INFO_COMPLETED,
  CMD_GET_DYNAMICS_INFO_FAILED,
  CMD_CREATE_COLLISION_SHAPE_FAILED,
  CMD_CREATE_COLLISION_SHAPE_COMPLETED,
  CMD_CREATE_VISUAL_SHAPE_FAILED,
  CMD_CREATE_VISUAL_SHAPE_COMPLETED,
  CMD_CREATE_MULTI_BODY_FAILED,
  CMD_CREATE_MULTI_BODY_COMPLETED,
  CMD_REQUEST_COLLISION_INFO_COMPLETED,
  CMD_REQUEST_COLLISION_INFO_FAILED,
  CMD_REQUEST_MOUSE_EVENTS_DATA_COMPLETED,
  CMD_CHANGE_TEXTURE_COMMAND_FAILED,
        CMD_MAX_SERVER_COMMANDS
};
enum JointInfoFlags
{
    JOINT_HAS_MOTORIZED_POWER=1,
};
enum
{
 COLLISION_SHAPE_TYPE_BOX=1,
 COLLISION_SHAPE_TYPE_CYLINDER_X,
 COLLISION_SHAPE_TYPE_CYLINDER_Y,
 COLLISION_SHAPE_TYPE_CYLINDER_Z,
 COLLISION_SHAPE_TYPE_CAPSULE_X,
 COLLISION_SHAPE_TYPE_CAPSULE_Y,
 COLLISION_SHAPE_TYPE_CAPSULE_Z,
 COLLISION_SHAPE_TYPE_SPHERE
};
enum JointType {
 eRevoluteType = 0,
 ePrismaticType = 1,
 eSphericalType = 2,
 ePlanarType = 3,
 eFixedType = 4,
 ePoint2PointType = 5,
 eGearType=6
};
enum b3JointInfoFlags
{
 eJointChangeMaxForce = 1,
 eJointChangeChildFramePosition = 2,
 eJointChangeChildFrameOrientation = 4,
};
struct b3JointInfo
{
        char* m_linkName;
        char* m_jointName;
        int m_jointType;
        int m_qIndex;
        int m_uIndex;
        int m_jointIndex;
        int m_flags;
  double m_jointDamping;
  double m_jointFriction;
  double m_jointLowerLimit;
  double m_jointUpperLimit;
  double m_jointMaxForce;
  double m_jointMaxVelocity;
  double m_parentFrame[7];
  double m_childFrame[7];
  double m_jointAxis[3];
};
struct b3UserConstraint
{
    int m_parentBodyIndex;
    int m_parentJointIndex;
    int m_childBodyIndex;
    int m_childJointIndex;
    double m_parentFrame[7];
    double m_childFrame[7];
    double m_jointAxis[3];
    int m_jointType;
    double m_maxAppliedForce;
    int m_userConstraintUniqueId;
 double m_gearRatio;
 int m_gearAuxLink;
};
struct b3BodyInfo
{
 const char* m_baseName;
 const char* m_bodyName;
};
struct b3DynamicsInfo
{
 double m_mass;
 double m_localInertialPosition[3];
 double m_lateralFrictionCoeff;
};
enum SensorType {
 eSensorForceTorqueType = 1,
};
struct b3JointSensorState
{
  double m_jointPosition;
  double m_jointVelocity;
  double m_jointForceTorque[6];
  double m_jointMotorTorque;
};
struct b3DebugLines
{
    int m_numDebugLines;
    const float* m_linesFrom;
    const float* m_linesTo;
    const float* m_linesColor;
};
struct b3OverlappingObject
{
 int m_objectUniqueId;
 int m_linkIndex;
};
struct b3AABBOverlapData
{
    int m_numOverlappingObjects;
 struct b3OverlappingObject* m_overlappingObjects;
};
struct b3CameraImageData
{
 int m_pixelWidth;
 int m_pixelHeight;
 const unsigned char* m_rgbColorData;
 const float* m_depthValues;
 const int* m_segmentationMaskValues;
};
struct b3OpenGLVisualizerCameraInfo
{
    int m_width;
    int m_height;
 float m_viewMatrix[16];
 float m_projectionMatrix[16];
 float m_camUp[3];
 float m_camForward[3];
 float m_horizontal[3];
 float m_vertical[3];
 float m_yaw;
 float m_pitch;
 float m_dist;
 float m_target[3];
};
enum b3VREventType
{
 VR_CONTROLLER_MOVE_EVENT=1,
 VR_CONTROLLER_BUTTON_EVENT=2,
 VR_HMD_MOVE_EVENT=4,
 VR_GENERIC_TRACKER_MOVE_EVENT=8,
};
enum b3VRButtonInfo
{
 eButtonIsDown = 1,
 eButtonTriggered = 2,
 eButtonReleased = 4,
};
enum eVRDeviceTypeEnums
{
 VR_DEVICE_CONTROLLER=1,
 VR_DEVICE_HMD=2,
 VR_DEVICE_GENERIC_TRACKER=4,
};
enum EVRCameraFlags
{
 VR_CAMERA_TRACK_OBJECT_ORIENTATION=1,
};
struct b3VRControllerEvent
{
 int m_controllerId;
 int m_deviceType;
 int m_numMoveEvents;
 int m_numButtonEvents;
 float m_pos[4];
 float m_orn[4];
 float m_analogAxis;
 int m_buttons[64];
};
struct b3VREventsData
{
 int m_numControllerEvents;
 struct b3VRControllerEvent* m_controllerEvents;
};
struct b3KeyboardEvent
{
 int m_keyCode;
 int m_keyState;
};
struct b3KeyboardEventsData
{
 int m_numKeyboardEvents;
 struct b3KeyboardEvent* m_keyboardEvents;
};
enum eMouseEventTypeEnums
{
 MOUSE_MOVE_EVENT=1,
 MOUSE_BUTTON_EVENT=2,
};
struct b3MouseEvent
{
 int m_eventType;
 float m_mousePosX;
 float m_mousePosY;
 int m_buttonIndex;
 int m_buttonState;
};
struct b3MouseEventsData
{
 int m_numMouseEvents;
 struct b3MouseEvent* m_mouseEvents;
};
struct b3ContactPointData
{
    int m_contactFlags;
    int m_bodyUniqueIdA;
    int m_bodyUniqueIdB;
    int m_linkIndexA;
    int m_linkIndexB;
    double m_positionOnAInWS[3];
    double m_positionOnBInWS[3];
    double m_contactNormalOnBInWS[3];
    double m_contactDistance;
    double m_normalForce;
};
enum
{
 CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS = 0,
 CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS = 1,
};
enum b3StateLoggingType
{
 STATE_LOGGING_MINITAUR = 0,
 STATE_LOGGING_GENERIC_ROBOT = 1,
 STATE_LOGGING_VR_CONTROLLERS = 2,
 STATE_LOGGING_VIDEO_MP4 = 3,
 STATE_LOGGING_COMMANDS = 4,
 STATE_LOGGING_CONTACT_POINTS = 5,
 STATE_LOGGING_PROFILE_TIMINGS = 6,
};
struct b3ContactInformation
{
 int m_numContactPoints;
 struct b3ContactPointData* m_contactPointData;
};
struct b3RayHitInfo
{
 double m_hitFraction;
 int m_hitObjectUniqueId;
 int m_hitObjectLinkIndex;
 double m_hitPositionWorld[3];
 double m_hitNormalWorld[3];
};
struct b3RaycastInformation
{
 int m_numRayHits;
 struct b3RayHitInfo* m_rayHits;
};
struct b3VisualShapeData
{
 int m_objectUniqueId;
 int m_linkIndex;
 int m_visualGeometryType;
 double m_dimensions[3];
 char m_meshAssetFileName[1024];
    double m_localVisualFrame[7];
    double m_rgbaColor[4];
};
struct b3VisualShapeInformation
{
 int m_numVisualShapes;
 struct b3VisualShapeData* m_visualShapeData;
};
enum eLinkStateFlags
{
 ACTUAL_STATE_COMPUTE_LINKVELOCITY=1
};
struct b3LinkState
{
    double m_worldPosition[3];
    double m_worldOrientation[4];
    double m_localInertialPosition[3];
    double m_localInertialOrientation[4];
 double m_worldLinkFramePosition[3];
    double m_worldLinkFrameOrientation[4];
 double m_worldLinearVelocity[3];
 double m_worldAngularVelocity[3];
 double m_worldAABBMin[3];
 double m_worldAABBMax[3];
};
enum {
    CONTROL_MODE_VELOCITY=0,
    CONTROL_MODE_TORQUE,
    CONTROL_MODE_POSITION_VELOCITY_PD,
};
enum EnumExternalForceFlags
{
    EF_LINK_FRAME=1,
    EF_WORLD_FRAME=2,
};
enum EnumRenderer
{
    ER_TINY_RENDERER=(1<<16),
    ER_BULLET_HARDWARE_OPENGL=(1<<17),
};
enum b3ConfigureDebugVisualizerEnum
{
    COV_ENABLE_GUI=1,
    COV_ENABLE_SHADOWS,
    COV_ENABLE_WIREFRAME,
 COV_ENABLE_VR_TELEPORTING,
 COV_ENABLE_VR_PICKING,
 COV_ENABLE_VR_RENDER_CONTROLLERS,
 COV_ENABLE_RENDERING,
 COV_ENABLE_SYNC_RENDERING_INTERNAL,
 COV_ENABLE_KEYBOARD_SHORTCUTS,
 COV_ENABLE_MOUSE_PICKING,
};
enum b3AddUserDebugItemEnum
{
 DEB_DEBUG_TEXT_USE_ORIENTATION=1,
 DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS=2,
 DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT=4,
};
enum eCONNECT_METHOD {
  eCONNECT_GUI = 1,
  eCONNECT_DIRECT = 2,
  eCONNECT_SHARED_MEMORY = 3,
  eCONNECT_UDP = 4,
  eCONNECT_TCP = 5,
  eCONNECT_EXISTING_EXAMPLE_BROWSER=6,
};
enum eURDF_Flags
{
 URDF_USE_INERTIA_FROM_FILE=2,
 URDF_USE_SELF_COLLISION=8,
 URDF_USE_SELF_COLLISION_EXCLUDE_PARENT=16,
 URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS=32,
};
enum eUrdfGeomTypes
{
 GEOM_SPHERE=2,
 GEOM_BOX,
 GEOM_CYLINDER,
 GEOM_MESH,
 GEOM_PLANE,
 GEOM_CAPSULE,
 GEOM_UNKNOWN,
};
enum eUrdfCollisionFlags
{
 GEOM_FORCE_CONCAVE_TRIMESH=1,
};
typedef struct b3PhysicsClientHandle__ { int unused; } *b3PhysicsClientHandle;
typedef struct b3SharedMemoryCommandHandle__ { int unused; } *b3SharedMemoryCommandHandle;
typedef struct b3SharedMemoryStatusHandle__ { int unused; } *b3SharedMemoryStatusHandle;
 b3PhysicsClientHandle b3ConnectSharedMemory(int key);
b3PhysicsClientHandle b3ConnectSharedMemory2(int key);
b3PhysicsClientHandle b3ConnectPhysicsDirect();
 b3PhysicsClientHandle b3ConnectPhysicsUDP(const char* hostName, int port);
b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnect(int argc, char* argv[]);
b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnectMainThread(int argc, char* argv[]);
b3PhysicsClientHandle b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect(void* guiHelperPtr);
void b3InProcessRenderSceneInternal(b3PhysicsClientHandle clientHandle);
void b3InProcessDebugDrawInternal(b3PhysicsClientHandle clientHandle, int debugDrawMode);
int b3InProcessMouseMoveCallback(b3PhysicsClientHandle clientHandle,float x,float y);
int b3InProcessMouseButtonCallback(b3PhysicsClientHandle clientHandle, int button, int state, float x, float y);
void b3DisconnectSharedMemory(b3PhysicsClientHandle physClient);
int b3CanSubmitCommand(b3PhysicsClientHandle physClient);
b3SharedMemoryStatusHandle b3SubmitClientCommandAndWaitStatus(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);
int b3SubmitClientCommand(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);
b3SharedMemoryStatusHandle b3ProcessServerStatus(b3PhysicsClientHandle physClient);
int b3GetStatusType(b3SharedMemoryStatusHandle statusHandle);
int b3GetStatusBodyIndices(b3SharedMemoryStatusHandle statusHandle, int* bodyIndicesOut, int bodyIndicesCapacity);
int b3GetStatusBodyIndex(b3SharedMemoryStatusHandle statusHandle);
int b3GetStatusActualState(b3SharedMemoryStatusHandle statusHandle,
                           int* bodyUniqueId,
                           int* numDegreeOfFreedomQ,
                           int* numDegreeOfFreedomU,
                           const double* rootLocalInertialFrame[],
                           const double* actualStateQ[],
                           const double* actualStateQdot[],
                           const double* jointReactionForces[]);
b3SharedMemoryCommandHandle b3RequestCollisionInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
int b3GetStatusAABB(b3SharedMemoryStatusHandle statusHandle, int linkIndex, double aabbMin[3], double aabbMax[3]);
b3SharedMemoryCommandHandle b3InitSyncBodyInfoCommand(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle b3InitRemoveBodyCommand(b3PhysicsClientHandle physClient, int bodyUniqueId);
int b3GetNumBodies(b3PhysicsClientHandle physClient);
int b3GetBodyUniqueId(b3PhysicsClientHandle physClient, int serialIndex);
int b3GetBodyInfo(b3PhysicsClientHandle physClient, int bodyUniqueId, struct b3BodyInfo* info);
int b3GetNumJoints(b3PhysicsClientHandle physClient, int bodyIndex);
int b3GetJointInfo(b3PhysicsClientHandle physClient, int bodyIndex, int jointIndex, struct b3JointInfo* info);
b3SharedMemoryCommandHandle b3GetDynamicsInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex);
int b3GetDynamicsInfo(b3SharedMemoryStatusHandle statusHandle, struct b3DynamicsInfo* info);
b3SharedMemoryCommandHandle b3InitChangeDynamicsInfo(b3PhysicsClientHandle physClient);
int b3ChangeDynamicsInfoSetMass(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double mass);
int b3ChangeDynamicsInfoSetLateralFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double lateralFriction);
int b3ChangeDynamicsInfoSetSpinningFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction);
int b3ChangeDynamicsInfoSetRollingFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction);
int b3ChangeDynamicsInfoSetRestitution(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double restitution);
int b3ChangeDynamicsInfoSetLinearDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId,double linearDamping);
int b3ChangeDynamicsInfoSetAngularDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId,double angularDamping);
int b3ChangeDynamicsInfoSetContactStiffnessAndDamping(b3SharedMemoryCommandHandle commandHandle,int bodyUniqueId,int linkIndex,double contactStiffness, double contactDamping);
int b3ChangeDynamicsInfoSetFrictionAnchor(b3SharedMemoryCommandHandle commandHandle,int bodyUniqueId,int linkIndex, int frictionAnchor);
b3SharedMemoryCommandHandle b3InitCreateUserConstraintCommand(b3PhysicsClientHandle physClient, int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, struct b3JointInfo* info);
int b3GetStatusUserConstraintUniqueId(b3SharedMemoryStatusHandle statusHandle);
b3SharedMemoryCommandHandle b3InitChangeUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId);
int b3InitChangeUserConstraintSetPivotInB(b3SharedMemoryCommandHandle commandHandle, double jointChildPivot[3]);
int b3InitChangeUserConstraintSetFrameInB(b3SharedMemoryCommandHandle commandHandle, double jointChildFrameOrn[4]);
int b3InitChangeUserConstraintSetMaxForce(b3SharedMemoryCommandHandle commandHandle, double maxAppliedForce);
int b3InitChangeUserConstraintSetGearRatio(b3SharedMemoryCommandHandle commandHandle, double gearRatio);
int b3InitChangeUserConstraintSetGearAuxLink(b3SharedMemoryCommandHandle commandHandle, int gearAuxLink);
b3SharedMemoryCommandHandle b3InitRemoveUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId);
int b3GetNumUserConstraints(b3PhysicsClientHandle physClient);
int b3GetUserConstraintInfo(b3PhysicsClientHandle physClient, int constraintUniqueId, struct b3UserConstraint* info);
int b3GetUserConstraintId(b3PhysicsClientHandle physClient, int serialIndex);
b3SharedMemoryCommandHandle b3InitRequestDebugLinesCommand(b3PhysicsClientHandle physClient, int debugMode);
void b3GetDebugLines(b3PhysicsClientHandle physClient, struct b3DebugLines* lines);
b3SharedMemoryCommandHandle b3InitConfigureOpenGLVisualizer(b3PhysicsClientHandle physClient);
void b3ConfigureOpenGLVisualizerSetVisualizationFlags(b3SharedMemoryCommandHandle commandHandle, int flag, int enabled);
void b3ConfigureOpenGLVisualizerSetViewMatrix(b3SharedMemoryCommandHandle commandHandle, float cameraDistance, float cameraPitch, float cameraYaw, const float cameraTargetPosition[3]);
b3SharedMemoryCommandHandle b3InitRequestOpenGLVisualizerCameraCommand(b3PhysicsClientHandle physClient);
int b3GetStatusOpenGLVisualizerCamera(b3SharedMemoryStatusHandle statusHandle, struct b3OpenGLVisualizerCameraInfo* camera);
b3SharedMemoryCommandHandle b3InitUserDebugDrawAddLine3D(b3PhysicsClientHandle physClient, double fromXYZ[3], double toXYZ[3], double colorRGB[3], double lineWidth, double lifeTime);
b3SharedMemoryCommandHandle b3InitUserDebugDrawAddText3D(b3PhysicsClientHandle physClient, const char* txt, double positionXYZ[3], double colorRGB[3], double textSize, double lifeTime);
void b3UserDebugTextSetOptionFlags(b3SharedMemoryCommandHandle commandHandle, int optionFlags);
void b3UserDebugTextSetOrientation(b3SharedMemoryCommandHandle commandHandle, double orientation[4]);
void b3UserDebugItemSetParentObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex);
b3SharedMemoryCommandHandle b3InitUserDebugAddParameter(b3PhysicsClientHandle physClient, const char* txt, double rangeMin, double rangeMax, double startValue);
b3SharedMemoryCommandHandle b3InitUserDebugReadParameter(b3PhysicsClientHandle physClient, int debugItemUniqueId);
int b3GetStatusDebugParameterValue(b3SharedMemoryStatusHandle statusHandle, double* paramValue);
b3SharedMemoryCommandHandle b3InitUserDebugDrawRemove(b3PhysicsClientHandle physClient, int debugItemUniqueId);
b3SharedMemoryCommandHandle b3InitUserDebugDrawRemoveAll(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle b3InitDebugDrawingCommand(b3PhysicsClientHandle physClient);
void b3SetDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex, double objectColorRGB[3]);
void b3RemoveDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex);
int b3GetDebugItemUniqueId(b3SharedMemoryStatusHandle statusHandle);
b3SharedMemoryCommandHandle b3InitRequestCameraImage(b3PhysicsClientHandle physClient);
void b3RequestCameraImageSetCameraMatrices(b3SharedMemoryCommandHandle command, float viewMatrix[16], float projectionMatrix[16]);
void b3RequestCameraImageSetPixelResolution(b3SharedMemoryCommandHandle command, int width, int height );
void b3RequestCameraImageSetLightDirection(b3SharedMemoryCommandHandle commandHandle, const float lightDirection[3]);
void b3RequestCameraImageSetLightColor(b3SharedMemoryCommandHandle commandHandle, const float lightColor[3]);
void b3RequestCameraImageSetLightDistance(b3SharedMemoryCommandHandle commandHandle, float lightDistance);
void b3RequestCameraImageSetLightAmbientCoeff(b3SharedMemoryCommandHandle commandHandle, float lightAmbientCoeff);
void b3RequestCameraImageSetLightDiffuseCoeff(b3SharedMemoryCommandHandle commandHandle, float lightDiffuseCoeff);
void b3RequestCameraImageSetLightSpecularCoeff(b3SharedMemoryCommandHandle commandHandle, float lightSpecularCoeff);
void b3RequestCameraImageSetShadow(b3SharedMemoryCommandHandle commandHandle, int hasShadow);
void b3RequestCameraImageSelectRenderer(b3SharedMemoryCommandHandle commandHandle, int renderer);
void b3GetCameraImageData(b3PhysicsClientHandle physClient, struct b3CameraImageData* imageData);
void b3ComputeViewMatrixFromPositions(const float cameraPosition[3], const float cameraTargetPosition[3], const float cameraUp[3], float viewMatrix[16]);
void b3ComputeViewMatrixFromYawPitchRoll(const float cameraTargetPosition[3], float distance, float yaw, float pitch, float roll, int upAxis, float viewMatrix[16]);
void b3ComputePositionFromViewMatrix(const float viewMatrix[16], float cameraPosition[3], float cameraTargetPosition[3], float cameraUp[3]);
void b3ComputeProjectionMatrix(float left, float right, float bottom, float top, float nearVal, float farVal, float projectionMatrix[16]);
void b3ComputeProjectionMatrixFOV(float fov, float aspect, float nearVal, float farVal, float projectionMatrix[16]);
void b3RequestCameraImageSetViewMatrix(b3SharedMemoryCommandHandle command, const float cameraPosition[3], const float cameraTargetPosition[3], const float cameraUp[3]);
void b3RequestCameraImageSetViewMatrix2(b3SharedMemoryCommandHandle commandHandle, const float cameraTargetPosition[3], float distance, float yaw, float pitch, float roll, int upAxis);
void b3RequestCameraImageSetProjectionMatrix(b3SharedMemoryCommandHandle command, float left, float right, float bottom, float top, float nearVal, float farVal);
void b3RequestCameraImageSetFOVProjectionMatrix(b3SharedMemoryCommandHandle command, float fov, float aspect, float nearVal, float farVal);
b3SharedMemoryCommandHandle b3InitRequestContactPointInformation(b3PhysicsClientHandle physClient);
void b3SetContactFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA);
void b3SetContactFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB);
void b3SetContactFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
void b3SetContactFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
void b3GetContactPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointInfo);
b3SharedMemoryCommandHandle b3InitClosestDistanceQuery(b3PhysicsClientHandle physClient);
void b3SetClosestDistanceFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA);
void b3SetClosestDistanceFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
void b3SetClosestDistanceFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB);
void b3SetClosestDistanceFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
void b3SetClosestDistanceThreshold(b3SharedMemoryCommandHandle commandHandle, double distance);
void b3GetClosestPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointInfo);
b3SharedMemoryCommandHandle b3InitAABBOverlapQuery(b3PhysicsClientHandle physClient, const double aabbMin[3],const double aabbMax[3]);
void b3GetAABBOverlapResults(b3PhysicsClientHandle physClient, struct b3AABBOverlapData* data);
b3SharedMemoryCommandHandle b3InitRequestVisualShapeInformation(b3PhysicsClientHandle physClient, int bodyUniqueIdA);
void b3GetVisualShapeInformation(b3PhysicsClientHandle physClient, struct b3VisualShapeInformation* visualShapeInfo);
b3SharedMemoryCommandHandle b3InitLoadTexture(b3PhysicsClientHandle physClient, const char* filename);
int b3GetStatusTextureUniqueId(b3SharedMemoryStatusHandle statusHandle);
b3SharedMemoryCommandHandle b3CreateChangeTextureCommandInit(b3PhysicsClientHandle physClient, int textureUniqueId, int width, int height, const char* rgbPixels);
b3SharedMemoryCommandHandle b3InitUpdateVisualShape(b3PhysicsClientHandle physClient, int bodyUniqueId, int jointIndex, int shapeIndex, int textureUniqueId);
void b3UpdateVisualShapeRGBAColor(b3SharedMemoryCommandHandle commandHandle, double rgbaColor[4]);
void b3UpdateVisualShapeSpecularColor(b3SharedMemoryCommandHandle commandHandle, double specularColor[3]);
b3SharedMemoryCommandHandle b3InitPhysicsParamCommand(b3PhysicsClientHandle physClient);
int b3PhysicsParamSetGravity(b3SharedMemoryCommandHandle commandHandle, double gravx,double gravy, double gravz);
int b3PhysicsParamSetTimeStep(b3SharedMemoryCommandHandle commandHandle, double timeStep);
int b3PhysicsParamSetDefaultContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultContactERP);
int b3PhysicsParamSetDefaultNonContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultNonContactERP);
int b3PhysicsParamSetDefaultFrictionERP(b3SharedMemoryCommandHandle commandHandle, double frictionERP);
int b3PhysicsParamSetNumSubSteps(b3SharedMemoryCommandHandle commandHandle, int numSubSteps);
int b3PhysicsParamSetRealTimeSimulation(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation);
int b3PhysicsParamSetNumSolverIterations(b3SharedMemoryCommandHandle commandHandle, int numSolverIterations);
int b3PhysicsParamSetCollisionFilterMode(b3SharedMemoryCommandHandle commandHandle, int filterMode);
int b3PhysicsParamSetUseSplitImpulse(b3SharedMemoryCommandHandle commandHandle, int useSplitImpulse);
int b3PhysicsParamSetSplitImpulsePenetrationThreshold(b3SharedMemoryCommandHandle commandHandle, double splitImpulsePenetrationThreshold);
int b3PhysicsParamSetContactBreakingThreshold(b3SharedMemoryCommandHandle commandHandle, double contactBreakingThreshold);
int b3PhysicsParamSetMaxNumCommandsPer1ms(b3SharedMemoryCommandHandle commandHandle, int maxNumCmdPer1ms);
int b3PhysicsParamSetEnableFileCaching(b3SharedMemoryCommandHandle commandHandle, int enableFileCaching);
int b3PhysicsParamSetRestitutionVelocityThreshold(b3SharedMemoryCommandHandle commandHandle, double restitutionVelocityThreshold);
int b3PhysicsParamSetInternalSimFlags(b3SharedMemoryCommandHandle commandHandle, int flags);
b3SharedMemoryCommandHandle b3InitStepSimulationCommand(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle b3InitResetSimulationCommand(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle b3LoadUrdfCommandInit(b3PhysicsClientHandle physClient, const char* urdfFileName);
int b3LoadUrdfCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int b3LoadUrdfCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int b3LoadUrdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);
int b3LoadUrdfCommandSetUseFixedBase(b3SharedMemoryCommandHandle commandHandle, int useFixedBase);
int b3LoadUrdfCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);
b3SharedMemoryCommandHandle b3LoadBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
b3SharedMemoryCommandHandle b3SaveBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
b3SharedMemoryCommandHandle b3LoadMJCFCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
void b3LoadMJCFCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);
b3SharedMemoryCommandHandle b3CalculateInverseDynamicsCommandInit(b3PhysicsClientHandle physClient, int bodyIndex,
 const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations);
int b3GetStatusInverseDynamicsJointForces(b3SharedMemoryStatusHandle statusHandle,
 int* bodyUniqueId,
 int* dofCount,
 double* jointForces);
b3SharedMemoryCommandHandle b3CalculateJacobianCommandInit(b3PhysicsClientHandle physClient, int bodyIndex, int linkIndex, const double* localPosition, const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations);
int b3GetStatusJacobian(b3SharedMemoryStatusHandle statusHandle, double* linearJacobian, double* angularJacobian);
b3SharedMemoryCommandHandle b3CalculateInverseKinematicsCommandInit(b3PhysicsClientHandle physClient, int bodyIndex);
void b3CalculateInverseKinematicsAddTargetPurePosition(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[3]);
void b3CalculateInverseKinematicsAddTargetPositionWithOrientation(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[3], const double targetOrientation[4]);
void b3CalculateInverseKinematicsPosWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[3], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose);
void b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[3], const double targetOrientation[4], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose);
void b3CalculateInverseKinematicsSetJointDamping(b3SharedMemoryCommandHandle commandHandle, int numDof, const double* jointDampingCoeff);
int b3GetStatusInverseKinematicsJointPositions(b3SharedMemoryStatusHandle statusHandle,
 int* bodyUniqueId,
 int* dofCount,
 double* jointPositions);
b3SharedMemoryCommandHandle b3LoadSdfCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName);
int b3LoadSdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);
b3SharedMemoryCommandHandle b3SaveWorldCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName);
b3SharedMemoryCommandHandle b3JointControlCommandInit(b3PhysicsClientHandle physClient, int controlMode);
b3SharedMemoryCommandHandle b3JointControlCommandInit2(b3PhysicsClientHandle physClient, int bodyUniqueId, int controlMode);
int b3JointControlSetDesiredPosition(b3SharedMemoryCommandHandle commandHandle, int qIndex, double value);
int b3JointControlSetKp(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
int b3JointControlSetKd(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
int b3JointControlSetDesiredVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
int b3JointControlSetMaximumForce(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
int b3JointControlSetDesiredForceTorque(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
b3SharedMemoryCommandHandle b3CreateCollisionShapeCommandInit(b3PhysicsClientHandle physClient);
int b3CreateCollisionShapeAddSphere(b3SharedMemoryCommandHandle commandHandle,double radius);
int b3CreateCollisionShapeAddBox(b3SharedMemoryCommandHandle commandHandle,double halfExtents[3]);
int b3CreateCollisionShapeAddCapsule(b3SharedMemoryCommandHandle commandHandle,double radius, double height);
int b3CreateCollisionShapeAddCylinder(b3SharedMemoryCommandHandle commandHandle,double radius, double height);
int b3CreateCollisionShapeAddPlane(b3SharedMemoryCommandHandle commandHandle, double planeNormal[3], double planeConstant);
int b3CreateCollisionShapeAddMesh(b3SharedMemoryCommandHandle commandHandle,const char* fileName, double meshScale[3]);
void b3CreateCollisionSetFlag(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, int flags);
void b3CreateCollisionShapeSetChildTransform(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, double childPosition[3], double childOrientation[4]);
int b3GetStatusCollisionShapeUniqueId(b3SharedMemoryStatusHandle statusHandle);
b3SharedMemoryCommandHandle b3CreateVisualShapeCommandInit(b3PhysicsClientHandle physClient);
int b3GetStatusVisualShapeUniqueId(b3SharedMemoryStatusHandle statusHandle);
b3SharedMemoryCommandHandle b3CreateMultiBodyCommandInit(b3PhysicsClientHandle physClient);
int b3CreateMultiBodyBase(b3SharedMemoryCommandHandle commandHandle, double mass, int collisionShapeUnique, int visualShapeUniqueId, double basePosition[3], double baseOrientation[4] , double baseInertialFramePosition[3], double baseInertialFrameOrientation[4]);
int b3CreateMultiBodyLink(b3SharedMemoryCommandHandle commandHandle, double linkMass, double linkCollisionShapeIndex,
         double linkVisualShapeIndex,
         double linkPosition[3],
         double linkOrientation[4],
         double linkInertialFramePosition[3],
         double linkInertialFrameOrientation[4],
         int linkParentIndex,
         int linkJointType,
         double linkJointAxis[3]);
void b3CreateMultiBodyUseMaximalCoordinates(b3SharedMemoryCommandHandle commandHandle);
b3SharedMemoryCommandHandle b3CreateBoxShapeCommandInit(b3PhysicsClientHandle physClient);
int b3CreateBoxCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int b3CreateBoxCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int b3CreateBoxCommandSetHalfExtents(b3SharedMemoryCommandHandle commandHandle, double halfExtentsX,double halfExtentsY,double halfExtentsZ);
int b3CreateBoxCommandSetMass(b3SharedMemoryCommandHandle commandHandle, double mass);
int b3CreateBoxCommandSetCollisionShapeType(b3SharedMemoryCommandHandle commandHandle, int collisionShapeType);
int b3CreateBoxCommandSetColorRGBA(b3SharedMemoryCommandHandle commandHandle, double red,double green,double blue, double alpha);
b3SharedMemoryCommandHandle b3CreatePoseCommandInit(b3PhysicsClientHandle physClient, int bodyIndex);
int b3CreatePoseCommandSetBasePosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int b3CreatePoseCommandSetBaseOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int b3CreatePoseCommandSetBaseLinearVelocity(b3SharedMemoryCommandHandle commandHandle, double linVel[3]);
int b3CreatePoseCommandSetBaseAngularVelocity(b3SharedMemoryCommandHandle commandHandle, double angVel[3]);
int b3CreatePoseCommandSetJointPositions(b3SharedMemoryCommandHandle commandHandle, int numJointPositions, const double* jointPositions);
int b3CreatePoseCommandSetJointPosition(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, int jointIndex, double jointPosition);
int b3CreatePoseCommandSetJointVelocities(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, int numJointVelocities, const double* jointVelocities);
int b3CreatePoseCommandSetJointVelocity(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, int jointIndex, double jointVelocity);
b3SharedMemoryCommandHandle b3CreateSensorCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
int b3CreateSensorEnable6DofJointForceTorqueSensor(b3SharedMemoryCommandHandle commandHandle, int jointIndex, int enable);
int b3CreateSensorEnableIMUForLink(b3SharedMemoryCommandHandle commandHandle, int linkIndex, int enable);
b3SharedMemoryCommandHandle b3RequestActualStateCommandInit(b3PhysicsClientHandle physClient,int bodyUniqueId);
int b3RequestActualStateCommandComputeLinkVelocity(b3SharedMemoryCommandHandle commandHandle, int computeLinkVelocity);
int b3GetJointState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int jointIndex, struct b3JointSensorState *state);
int b3GetLinkState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int linkIndex, struct b3LinkState *state);
b3SharedMemoryCommandHandle b3PickBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ);
b3SharedMemoryCommandHandle b3MovePickedBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                             double rayFromWorldY, double rayFromWorldZ,
                                             double rayToWorldX, double rayToWorldY,
                                             double rayToWorldZ);
b3SharedMemoryCommandHandle b3RemovePickingConstraint(b3PhysicsClientHandle physClient);
b3SharedMemoryCommandHandle b3CreateRaycastCommandInit(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ);
b3SharedMemoryCommandHandle b3CreateRaycastBatchCommandInit(b3PhysicsClientHandle physClient);
void b3RaycastBatchAddRay(b3SharedMemoryCommandHandle commandHandle, const double rayFromWorld[3], const double rayToWorld[3]);
void b3GetRaycastInformation(b3PhysicsClientHandle physClient, struct b3RaycastInformation* raycastInfo);
b3SharedMemoryCommandHandle b3ApplyExternalForceCommandInit(b3PhysicsClientHandle physClient);
void b3ApplyExternalForce(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double force[3], const double position[3], int flags);
void b3ApplyExternalTorque(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double torque[3], int flags);
b3SharedMemoryCommandHandle b3LoadBunnyCommandInit(b3PhysicsClientHandle physClient);
int b3LoadBunnySetScale(b3SharedMemoryCommandHandle commandHandle, double scale);
int b3LoadBunnySetMass(b3SharedMemoryCommandHandle commandHandle, double mass);
int b3LoadBunnySetCollisionMargin(b3SharedMemoryCommandHandle commandHandle, double collisionMargin);
b3SharedMemoryCommandHandle b3RequestVREventsCommandInit(b3PhysicsClientHandle physClient);
void b3VREventsSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter);
void b3GetVREventsData(b3PhysicsClientHandle physClient, struct b3VREventsData* vrEventsData);
b3SharedMemoryCommandHandle b3SetVRCameraStateCommandInit(b3PhysicsClientHandle physClient);
int b3SetVRCameraRootPosition(b3SharedMemoryCommandHandle commandHandle, double rootPos[3]);
int b3SetVRCameraRootOrientation(b3SharedMemoryCommandHandle commandHandle, double rootOrn[4]);
int b3SetVRCameraTrackingObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId);
int b3SetVRCameraTrackingObjectFlag(b3SharedMemoryCommandHandle commandHandle, int flag);
b3SharedMemoryCommandHandle b3RequestKeyboardEventsCommandInit(b3PhysicsClientHandle physClient);
void b3GetKeyboardEventsData(b3PhysicsClientHandle physClient, struct b3KeyboardEventsData* keyboardEventsData);
b3SharedMemoryCommandHandle b3RequestMouseEventsCommandInit(b3PhysicsClientHandle physClient);
void b3GetMouseEventsData(b3PhysicsClientHandle physClient, struct b3MouseEventsData* mouseEventsData);
b3SharedMemoryCommandHandle b3StateLoggingCommandInit(b3PhysicsClientHandle physClient);
int b3StateLoggingStart(b3SharedMemoryCommandHandle commandHandle, int loggingType, const char* fileName);
int b3StateLoggingAddLoggingObjectUniqueId(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId);
int b3StateLoggingSetMaxLogDof(b3SharedMemoryCommandHandle commandHandle, int maxLogDof);
int b3StateLoggingSetLinkIndexA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
int b3StateLoggingSetLinkIndexB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
int b3StateLoggingSetBodyAUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyAUniqueId);
int b3StateLoggingSetBodyBUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyBUniqueId);
int b3StateLoggingSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter);
int b3GetStatusLoggingUniqueId(b3SharedMemoryStatusHandle statusHandle);
int b3StateLoggingStop(b3SharedMemoryCommandHandle commandHandle, int loggingUniqueId);
b3SharedMemoryCommandHandle b3ProfileTimingCommandInit(b3PhysicsClientHandle physClient, const char* name);
void b3SetProfileTimingDuractionInMicroSeconds(b3SharedMemoryCommandHandle commandHandle, int duration);
void b3SetTimeOut(b3PhysicsClientHandle physClient, double timeOutInSeconds);
double b3GetTimeOut(b3PhysicsClientHandle physClient);
void b3MultiplyTransforms(const double posA[3], const double ornA[4], const double posB[3], const double ornB[4], double outPos[3], double outOrn[4]);
void b3InvertTransform(const double pos[3], const double orn[4], double outPos[3], double outOrn[4]);
