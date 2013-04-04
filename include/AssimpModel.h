#ifndef __AssimpModel_H__
#define __AssimpModel_H__

#include <vector>
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBody.h"
#include "mndlkit/params/PParams.h"
#include "AssimpLoader.h"
#include "Node.h"

typedef std::shared_ptr< class AssimpBone             > AssimpBoneRef;
typedef std::shared_ptr< class AssimpJoint            > AssimpJointRef;
typedef std::shared_ptr< class AssimpDisableCollision > AssimpDisableCollisionRef;
typedef std::shared_ptr< class AssimpHang             > AssimpHangRef;

class AssimpBone
{
public:
	AssimpBone( const mndl::NodeRef& node );

	mndl::NodeRef&    getNode();

	void              setLength( const float length );
	const float       getLength() const;

	void              setMass( const float mass );
	const float       getMass() const;

	void              setShape( btCollisionShape* shape );
	btCollisionShape* getShape() const;

	void              setRigidBody( btRigidBody* rigidBody );
	btRigidBody*      getRigidBody() const;

protected:
	mndl::NodeRef     mNode;

	float             mLength;
	float             mMass;

	btCollisionShape* mShape;
	btRigidBody*      mRigidBody;
};

class AssimpJoint
{
public:
	AssimpJoint( const AssimpBoneRef& assimBoneA, const AssimpBoneRef& assimBoneB );

	const AssimpBoneRef&   getAssimpBoneA() const;
	const AssimpBoneRef&   getAssimpBoneB() const;

	void                   setSwing1( const float swing1 );
	const float            getSwing1() const;

	void                   setSwing2( const float swing2 );
	const float            getSwing2() const;

	void                   setTwist( const float twist );
	const float            getTwist() const;

	void                   setConstraint( btConeTwistConstraint* constraint );
	btConeTwistConstraint* getConstraint() const;

protected:
	AssimpBoneRef          mAssimpBoneA;
	AssimpBoneRef          mAssimpBoneB;

	float                  mSwing1;
	float                  mSwing2;
	float                  mTwist;

	btConeTwistConstraint* mConstraint;
};

class AssimpDisableCollision
{
public:
	typedef std::vector< AssimpJointRef > AssimpJoints;

public:
	AssimpDisableCollision();

	void addAssimpJoint( const AssimpJointRef& assimpJoint );
	AssimpJoints getAssimpJoints() const;

protected:
	AssimpJoints mAssimpJoints;
};

class AssimpHang
{
public:
	enum PosType
	{
		PT_BEG,
		PT_END
	};

	AssimpHang();
	static PosType getTypeFromString( const std::string& type );

	float            mStringLength;
	float            mStickSize;

	AssimpBoneRef    mAssimpBoneFront;
	PosType          mPosTypeFront;
	AssimpBoneRef    mAssimpBoneBack;
	PosType          mPosTypeBack;
	AssimpBoneRef    mAssimpBoneLeft;
	PosType          mPosTypeLeft;
	AssimpBoneRef    mAssimpBoneRight;
	PosType          mPosTypeRight;
	AssimpBoneRef    mAssimpBoneCenter;
	PosType          mPosTypeCenter;

	ci::Vec3f        mPosFront;
	ci::Vec3f        mPosBack;
	ci::Vec3f        mPosLeft;
	ci::Vec3f        mPosRight;
	ci::Vec3f        mPosCenter;

	ci::Vec3f        mPosCross;

	btCompoundShape* mCompoundShape;
	btBoxShape*      mShapeFrontBack;
	btBoxShape*      mShapeLeftRight;
	btRigidBody*     mRigidBody;

	btSoftBody*      mRopeFront;
	btSoftBody*      mRopeBack;
	btSoftBody*      mRopeLeft;
	btSoftBody*      mRopeRight;
	btSoftBody*      mRopeCenter;
};

class AssimpModel
{
	typedef std::vector< AssimpBoneRef             > AssimpBones;
	typedef std::vector< AssimpJointRef            > AssimpJoints;
	typedef std::vector< AssimpDisableCollisionRef > AssimpDisableCollisions;

public:
	enum AnimateType
	{
		ANIMATE_SIGN
	};

public:
	AssimpModel( btDynamicsWorld *ownerWorld, btSoftBodyWorldInfo *softBodyWorldInfo, const ci::Vec3f &worldOffset, const boost::filesystem::path &fileModel, const boost::filesystem::path &fileData );
	~AssimpModel();

	void update( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );
	void draw();

	static void setupParams();

	void animate( AnimateType animateType );

protected:
	void loadData( const boost::filesystem::path& fileData );
	void loadDataBones( const ci::XmlTree& xmlNode );
	void loadDataJoints( const ci::XmlTree& xmlNode );
	void loadDataDisableCollisions( const ci::XmlTree& xmlNode );
	void loadDataDisableCollision( const ci::XmlTree& xmlNode );
	void loadDataHang( const ci::XmlTree& xmlNode );

	AssimpBoneRef  getAssimpBone( const std::string& name );
	AssimpJointRef getAssimpJoint( const std::string& nameA, const std::string& nameB );

	float getLength( const mndl::NodeRef &nodeA, const mndl::NodeRef &nodeB );

	btRigidBody *localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape );
	btSoftBody  *localCreateRope( const ci::Vec3f &from, const ci::Vec3f &to, btRigidBody *rigidBodyFrom, btRigidBody *rigidBodyTo );

	void buildBones();
	void buildBone( const AssimpBoneRef& assimpBone );
	void buildJoints();
	void buildJoint( const AssimpJointRef& assimpJoint );
	void buildDisableCollisions();
	void buildDisableCollision( const AssimpDisableCollisionRef& assimpDisableCollision );
	void buildHang();
	void updateBones();
	void updateBone( const AssimpBoneRef& assimpBone );
	void updateHang( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );

	void animateSign();

	void  printNodes();
protected:
	btDynamicsWorld         *mOwnerWorld;
	btSoftBodyWorldInfo     *mSoftBodyWorldInfo;

	mndl::assimp::AssimpLoader mAssimpLoader;

	static mndl::params::PInterfaceGl            mParamsBird;

	// rigid body
	static float       mLinearDamping;  // [0-1]
	static float       mAngularDamping; // [0-1]
	static float       mDeactivationTime;
	static float       mLinearSleepingThresholds;
	static float       mAngularSleepingThresholds;

	// cone twist constraint
	static float       mDamping;
	static float       mStopCMF;
	static float       mStopERP;
// 	static float       mLinCFM;
// 	static float       mLinERP;
// 	static float       mAngCFM;

	static mndl::params::PInterfaceGl            mParamsRope;
	static float       mRopeSize;
	static int         mRopePart;
	static float       mRopeMass;
	static float       mKVCF;           // Velocities correction factor (Baumgarte)
	static float       mKDP;            // Damping coefficient [0,1]
	static float       mKDG;            // Drag coefficient [0,+inf]
	static float       mKLF;            // Lift coefficient [0,+inf]
	static float       mKPR;            // Pressure coefficient [-inf,+inf]
	static float       mKVC;            // Volume conversation coefficient [0,+inf]
	static float       mKDF;            // Dynamic friction coefficient [0,1]
	static float       mKMT;            // Pose matching coefficient [0,1]		
	static float       mKCHR;           // Rigid contacts hardness [0,1]
	static float       mKKHR;           // Kinetic contacts hardness [0,1]
	static float       mKSHR;           // Soft contacts hardness [0,1]
	static float       mKAHR;           // Anchors hardness [0,1]
	static float       mMaxvolume;      // Maximum volume ratio for pose
	static float       mTimescale;      // Time scale
	static int         mViterations;    // Velocities solver iterations
	static int         mPiterations;    // Positions solver iterations
	static int         mDiterations;    // Drift solver iterations
	static int         mCiterations;    // Cluster solver iterations

	static bool        mDrawSkin;
	static bool        mEnableWireframe;
	static float       mForce;

// 	static float       mTau;
// 	static float       mDamping;
// 	static float       mImpulseClamp;

	AssimpBones             mAssimpBones;
	AssimpJoints            mAssimpJoints;
	AssimpDisableCollisions mAssimpDisableCollisions;
	AssimpHangRef           mAssimpHang;
	float                   mMinBoneLength;
	float                   mCapsuleRadius;
};

#endif // __AssimpModel_H__
